#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <signal.h>

#include <damiao_motor/dm_motor_device_collection.hpp>

#include "config/config_structs.hpp"
#include <glaze/glaze.hpp>
#include "redis/RedisClient.hpp"

namespace {

struct RedisSendData {
	std::vector<double> joint_positions;
	std::vector<double> joint_velocities;
	std::vector<double> joint_torques;
};

std::string getJointPosKey(const std::string& robot_name) {
	return "/rocemo/" + robot_name + "/sensors/joint_positions";
}
std::string getJointVelKey(const std::string& robot_name) {
	return "/rocemo/" + robot_name + "/sensors/joint_velocities";
}
std::string getTorqueCommandKey(const std::string& robot_name) {
	return "/rocemo/" + robot_name + "/commands/joint_torques";
}
std::string getSensedTorquesKey(const std::string& robot_name) {
	return "/rocemo/" + robot_name + "/sensors/joint_torques";
}

volatile sig_atomic_t runloop = 1;
void signal_handler(int signal) {
	if (signal == SIGINT) {
		std::cout << "SIGINT received, exiting...\n";
		runloop = 0;
	}
}

using damiao_motor::Motor;
using damiao_motor::LimitParam;
using damiao_motor::RID;

constexpr LimitParam kProbeLimits{12.5, 30.0, 10.0};

void print_usage(const char* prog) {
	std::cout << "Usage: " << prog << " [-c|--config <config_path>]\n\n"
			  << "Options:\n"
			  << "  -c, --config <path>   Path to config file\n"
			  << "  --help                Show this help\n\n"
			  << "Example:\n"
			  << "  " << prog << " -c config/custom_config.json\n";
}

bool parse_config(int argc, char** argv, DMCanRobotDriverConfig& config) {
	std::optional<std::string> config_file_path;

	for (int i = 1; i < argc; ++i) {
		const std::string arg = argv[i];

		if (arg == "--help") {
			print_usage(argv[0]);
			return false;
		}

		if (arg != "--config" && arg != "-c") {
			std::cerr << "Unknown argument: " << arg << "\n";
			return false;
		}

		if (i + 1 >= argc) {
			std::cerr << "Missing value for argument: " << arg << "\n";
			return false;
		}

		config_file_path = argv[++i];
	}

	if (config_file_path.has_value()) {
		std::string buffer;
		auto ec = glz::read_file_json(config, *config_file_path, buffer);
		if (ec) {
			std::cerr << "Failed to parse JSON config: " << *config_file_path << "\n";
			const std::string formatted = glz::format_error(ec, buffer);
			std::cerr << formatted << std::endl;
			return false;
		}
	} else {
		std::cout << "No -c/--config provided. Using default config values from config_structs.hpp\n";
	}

	if (config.robot_model_file.empty()) {
		std::cerr << "robot_model_file is required in the config\n";
		return false;
	}

	for (const auto& range : config.arm_can_id_ranges) {
		if (range.first == 0 || range.second == 0) {
			std::cerr << "Invalid config: CAN IDs must be > 0\n";
			return false;
		}
		if (range.first > range.second) {
			std::cerr << "Invalid config: arm CAN ID range start must be <= end\n";
			return false;
		}
	}

	return true;
}


bool data_matches_motor_id(const std::vector<uint8_t>& data, uint32_t send_id) {
	if (data.size() < 2) return false;
	const uint16_t id_in_payload = static_cast<uint16_t>(data[0]) |
								   (static_cast<uint16_t>(data[1]) << 8);
	return id_in_payload == static_cast<uint16_t>(send_id & 0xFFFF);
}

void send_packet(canbus::CANSocket& sock, bool use_fd,
				 const damiao_motor::CANPacket& packet) {
	if (use_fd) {
		canfd_frame frame{};
		frame.can_id = packet.send_can_id;
		frame.len = static_cast<__u8>(packet.data.size());
		frame.flags = CANFD_BRS;
		std::copy(packet.data.begin(), packet.data.end(), frame.data);
		sock.write_canfd_frame(frame);
	} else {
		can_frame frame{};
		frame.can_id = packet.send_can_id;
		frame.can_dlc = static_cast<__u8>(packet.data.size());
		std::copy(packet.data.begin(), packet.data.end(), frame.data);
		sock.write_can_frame(frame);
	}
}

bool recv_one_frame(canbus::CANSocket& sock, bool use_fd, std::vector<uint8_t>& data,
					canid_t& can_id) {
	if (use_fd) {
		canfd_frame frame{};
		if (!sock.read_canfd_frame(frame)) return false;
		can_id = frame.can_id;
		data.assign(frame.data, frame.data + frame.len);
		return true;
	}

	can_frame frame{};
	if (!sock.read_can_frame(frame)) return false;
	can_id = frame.can_id;
	data.assign(frame.data, frame.data + frame.can_dlc);
	return true;
}

std::optional<double> query_register_with_retry(canbus::CANSocket& sock, bool use_fd,
												const Motor& motor, int rid, int retries = 4) {
	using damiao_motor::CanPacketDecoder;
	using damiao_motor::CanPacketEncoder;

	for (int attempt = 0; attempt < retries; ++attempt) {
		send_packet(sock, use_fd, CanPacketEncoder::create_query_param_command(motor, rid));

		for (int spin = 0; spin < 20; ++spin) {
			if (!sock.is_data_available(2000)) continue;

			std::vector<uint8_t> data;
			canid_t can_id = 0;
			if (!recv_one_frame(sock, use_fd, data, can_id)) continue;
			(void)can_id;

			if (data.size() < 8) continue;
			if (!data_matches_motor_id(data, motor.get_send_can_id())) continue;
			if (data[3] != static_cast<uint8_t>(rid)) continue;

			const auto parsed = CanPacketDecoder::parse_motor_param_data(data);
			if (parsed.valid && parsed.rid == rid) {
				return parsed.value;
			}
		}
	}

	return std::nullopt;
}

void receive_all_can_replies(canbus::CANSocket& can_socket,
					canbus::CANDeviceCollection& device_collection,
					bool use_fd,
					int first_timeout_us = 200) {
	int timeout_us = first_timeout_us;

	if (use_fd) {
		canfd_frame response_frame{};
		while (can_socket.is_data_available(timeout_us) &&
			   can_socket.read_canfd_frame(response_frame)) {
			device_collection.dispatch_frame_callback(response_frame);
			timeout_us = 0;
		}
	} else {
		can_frame response_frame{};
		while (can_socket.is_data_available(timeout_us) &&
			   can_socket.read_can_frame(response_frame)) {
			device_collection.dispatch_frame_callback(response_frame);
			timeout_us = 0;
		}
	}
}

std::optional<LimitParam> query_motor_limits(canbus::CANSocket& can_socket, const DMCanRobotDriverConfig& config,
													  uint32_t send_id) {
	Motor probe(kProbeLimits, send_id, send_id + config.recv_offset);

	const auto pmax = query_register_with_retry(can_socket, config.use_fd, probe,
														 static_cast<int>(RID::PMAX), 3);
	const auto vmax = query_register_with_retry(can_socket, config.use_fd, probe,
														 static_cast<int>(RID::VMAX), 3);
	const auto tmax = query_register_with_retry(can_socket, config.use_fd, probe,
														 static_cast<int>(RID::TMAX), 3);

	if (!pmax.has_value() || !vmax.has_value() || !tmax.has_value()) {
		return std::nullopt;
	}

	if (*pmax <= 0.0 || *vmax <= 0.0 || *tmax <= 0.0) {
		return std::nullopt;
	}

	return LimitParam{*pmax, *vmax, *tmax};
}

}  // namespace

int main(int argc, char** argv) {
	// parse config
	DMCanRobotDriverConfig config;
	if (!parse_config(argc, argv, config)) {
		if (argc > 1 && std::string(argv[1]) == "--help") {
			return 0;
		}
		print_usage(argv[0]);
		return 1;
	}

	// TODO: load pinocchio model gor grav comp

	// TODO: validate model is coherent with number of can motors

	// setup signal handlers for graceful shutdown
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);

	try {
		canbus::CANSocket can_socket(config.iface, config.use_fd);
		std::cout << "Connected. iface=" << config.iface << "\n";

		uint32_t last_can_id = 0;
		for (const auto& range : config.arm_can_id_ranges) {
			if (range.second > last_can_id) {
				last_can_id = range.second;
			}
		}
		for (const auto& id : config.gripper_can_ids) {
			if (id > last_can_id) {
				last_can_id = id;
			}
		}

		std::cout << "Scanning motor IDs...\n";
		int num_motors_expected = 0;
		for (const auto& range : config.arm_can_id_ranges) {
			num_motors_expected += static_cast<int>(range.second - range.first + 1);
		}
		num_motors_expected += static_cast<int>(config.gripper_can_ids.size());
		std::vector<Motor> motors;
		motors.reserve(static_cast<size_t>(num_motors_expected));
		for (const auto& range : config.arm_can_id_ranges) {
			for (uint32_t send_id = range.first; send_id <= range.second; ++send_id) {
				const auto limits = query_motor_limits(can_socket, config, send_id);
				if (!limits.has_value()) {
					// No valid limits response means no motor (or no response) at this ID.
					continue;
				}
				motors.emplace_back(*limits, send_id, send_id + config.recv_offset, config.arm_uses_custom_firmware);
			}
		}
		for (const auto& id : config.gripper_can_ids) {
			const auto limits = query_motor_limits(can_socket, config, id);
			if (!limits.has_value()) {
				// No valid limits response means no motor (or no response) at this ID.
				continue;
			}
			motors.emplace_back(*limits, id, id + config.recv_offset, config.gripper_uses_custom_firmware);
		}

		if (motors.empty()) {
			std::cout << "No motors responded with valid PMAX/VMAX/TMAX in the scan range.\n";
			return 0;
		}

		damiao_motor::DMDeviceCollection dm_collection(can_socket);
		std::vector<std::shared_ptr<damiao_motor::DMCANDevice>> dm_devices;
		dm_devices.reserve(motors.size());
		for (auto& motor : motors) {
			auto dm_device = std::make_shared<damiao_motor::DMCANDevice>(
				motor, CAN_SFF_MASK, config.use_fd, motor.uses_custom_firmware());
			dm_device->set_callback_mode(damiao_motor::CallbackMode::STATE);
			dm_collection.get_device_collection().add_device(dm_device);
			dm_devices.push_back(dm_device);
		}

		std::cout << "Detected motor IDs: ";
		for (size_t i = 0; i < motors.size(); ++i) {
			std::cout << "0x" << std::hex << motors[i].get_send_can_id() << std::dec;
			if (i + 1 < motors.size()) std::cout << ", ";
		}
		std::cout << "\n";
		for (const auto& motor : motors) {
			const auto& lim = motor.get_limits();
			std::cout << "  motor 0x" << std::hex << motor.get_send_can_id() << std::dec
					  << " limits: pmax=" << lim.pMax << " vmax=" << lim.vMax
					  << " tmax=" << lim.tMax << "\n";
		}
		std::cout << "Arm firmware mode: "
				  << (config.arm_uses_custom_firmware ? "custom (16-16-16)" : "standard (16-12-12)") << "\n";
		std::cout << "Gripper firmware mode: "
				  << (config.gripper_uses_custom_firmware ? "custom (16-16-16)" : "standard (16-12-12)") << "\n";

		// --- Firmware sanity check ---
		dm_collection.enable_all();
		// dm_collection.refresh_all();
		const damiao_motor::MITParam mit_zero{0.0, 0.0, 0.0, 0.0, 0.0};
		std::vector<damiao_motor::MITParam> mit_commands(motors.size(), mit_zero);
		dm_collection.mit_control_all(mit_commands);
		usleep(10000);  // wait for all replies to be processed
		receive_all_can_replies(can_socket, dm_collection.get_device_collection(), config.use_fd);

		constexpr double kTorqueSanityThreshold = 1.0;  // Nm; firmware mismatch causes large garbage
		bool firmware_ok = true;
		for (const auto& motor : motors) {
			std::cout << "Motor 0x" << std::hex << motor.get_send_can_id() << std::dec
					  << " state: pos=" << motor.get_position()
					  << " vel=" << motor.get_velocity()
					  << " torque=" << motor.get_torque()
					  << " tmos=" << motor.get_state_tmos()
					  << " trotor=" << motor.get_state_trotor() << "\n";
			if (std::abs(motor.get_torque()) > kTorqueSanityThreshold) {
				std::cerr << "[FIRMWARE MISMATCH] Motor 0x" << std::hex
						  << motor.get_send_can_id() << std::dec
						  << " reported torque=" << motor.get_torque()
						  << " Nm at rest (threshold: " << kTorqueSanityThreshold << " Nm).\n"
						  << "  Expected firmware: "
						  << (motor.uses_custom_firmware() ? "custom (16-16-16)" : "standard (16-12-12)")
						  << ".\n"
						  << "  Try toggling --custom-firmware.\n";
				firmware_ok = false;
				break;
			}
		}
		if (!firmware_ok) {
			dm_collection.disable_all();
			receive_all_can_replies(can_socket, dm_collection.get_device_collection(), config.use_fd);
			return 1;
		}

		// redis setup
		RocemoDmCanDriver::Communication::RedisClient redis_client;
		redis_client.connect();

		std::vector<double> zero_vector(motors.size(), 0.0);

		std::string command_torques_key = getTorqueCommandKey(config.robot_name);
		std::string joint_positions_key = getJointPosKey(config.robot_name);
		std::string joint_velocities_key = getJointVelKey(config.robot_name);
		std::string sensed_torques_key = getSensedTorquesKey(config.robot_name);

		redis_client.set<std::vector<double>>(command_torques_key, zero_vector);
		redis_client.set<std::vector<double>>(joint_velocities_key, zero_vector);
		// Check if a controller is running on that robot and publishing joint torques, or if another
		// driver/simulation is running and publishing joint velocities. In both case, don't start the driver.
		std::this_thread::sleep_for(std::chrono::microseconds(50));
		if (redis_client.get<std::vector<double>>(command_torques_key) != zero_vector) {
			std::cerr << "Stop the controller on robot [" << config.robot_name << "] before running the driver\n" << "\n";
			return -1;
		}
		if (redis_client.get<std::vector<double>>(joint_velocities_key) != zero_vector) {
			std::cerr << "A simulation or another driver is already running on robot [" << config.robot_name << "] cannot start the driver\n" << "\n";
			return -1;
		}

		std::vector<double> tau_cmd(motors.size(), 0.0);
		RedisSendData redis_send_data{
			.joint_positions = zero_vector,
			.joint_velocities = zero_vector,
			.joint_torques = zero_vector
		};

		unsigned long long counter = 0;
		constexpr unsigned long long kPrintEvery = 100;  // Print at ~10Hz while loop runs at 1kHz.

		auto t_start = std::chrono::steady_clock::now();
		auto next_tick = t_start;

		std::cout << "Entering main driver loop...\n";
		while (runloop) {
			next_tick += std::chrono::milliseconds(1);

			// read commands from redis
			tau_cmd = redis_client.get<std::vector<double>>(command_torques_key);
			if (tau_cmd.size() != motors.size()) {
				std::cerr << "Received command size " << tau_cmd.size()
						  << " does not match number of motors " << motors.size() << "\n";
				runloop = 0;
				break;
			}
			for(int i = 0; i < motors.size(); ++i) {
				mit_commands[i].tau = tau_cmd[i];
			}

			// Broadcast MIT command to all motors.
			// DM motors reply with state to control commands, so no explicit refresh here.
			dm_collection.mit_control_all(mit_commands);
			receive_all_can_replies(can_socket, dm_collection.get_device_collection(), config.use_fd, 100);

			// send replies to redis
			for(size_t i = 0; i < motors.size(); ++i) {
				redis_send_data.joint_positions[i] = motors[i].get_position();
				redis_send_data.joint_velocities[i] = motors[i].get_velocity();
				redis_send_data.joint_torques[i] = motors[i].get_torque();
			}
			redis_client.setCustomStruct<RedisSendData>(
				{joint_positions_key, joint_velocities_key, sensed_torques_key},
				redis_send_data
			);


			// if (counter % kPrintEvery == 0) {
			// 	std::cout << "\nSample " << counter << "\n";
			// 	std::cout << "-----------------------------------------------\n";
			// 	for (size_t m = 0; m < motors.size(); ++m) {
			// 		const auto& motor = motors[m];
			// 		std::cout << "0x" << std::hex << std::setw(8) << motor.get_send_can_id()
			// 				  << std::dec;

			// 		std::cout << std::fixed << std::setprecision(6) << std::setw(14)
			// 				  << motor.get_position() << std::setw(14) << motor.get_velocity()
			// 				  << std::setw(14) << motor.get_torque() << std::setw(8)
			// 				  << motor.get_state_tmos() << motor.get_state_trotor() << "\n";
			// 	}
			// 	std::cout << "\n";
			// }

			++counter;
			std::this_thread::sleep_until(next_tick);
		}

		auto t_end = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed = t_end - t_start;
		std::cout << "Elapsed time: " << elapsed.count() << " seconds\n";
		std::cout << "running frequency: " << (counter / elapsed.count()) << " Hz\n";

		dm_collection.disable_all();
		receive_all_can_replies(can_socket, dm_collection.get_device_collection(), config.use_fd, 1000);

	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << "\n";
		return 1;
	}

	return 0;
}
