#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <signal.h>

#include <damiao_motor/dm_motor_device_collection.hpp>

#include "config/config_structs.hpp"
#include <glaze/glaze.hpp>

namespace {

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

bool parse_config(int argc, char** argv, DMCanRobotDriverConfig& cfg) {
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
		auto ec = glz::read_file_json(cfg, *config_file_path, buffer);
		if (ec) {
			std::cerr << "Failed to parse JSON config: " << *config_file_path << "\n";
			const std::string formatted = glz::format_error(ec, buffer);
			std::cerr << formatted << std::endl;
			return false;
		}
	} else {
		std::cout << "No -c/--config provided. Using default config values from config_structs.hpp\n";
	}

	if (cfg.first_can_id > cfg.last_can_id) {
		std::cerr << "Invalid config: first_can_id must be <= last_can_id\n";
		return false;
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

std::optional<LimitParam> query_motor_limits(canbus::CANSocket& can_socket, const DMCanRobotDriverConfig& cfg,
													  uint32_t send_id) {
	Motor probe(kProbeLimits, send_id, send_id + cfg.recv_offset);

	const auto pmax = query_register_with_retry(can_socket, cfg.use_fd, probe,
														 static_cast<int>(RID::PMAX), 3);
	const auto vmax = query_register_with_retry(can_socket, cfg.use_fd, probe,
														 static_cast<int>(RID::VMAX), 3);
	const auto tmax = query_register_with_retry(can_socket, cfg.use_fd, probe,
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
	DMCanRobotDriverConfig cfg;
	if (!parse_config(argc, argv, cfg)) {
		if (argc > 1 && std::string(argv[1]) == "--help") {
			return 0;
		}
		print_usage(argv[0]);
		return 1;
	}

	// setup signal handlers for graceful shutdown
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);

	try {
		canbus::CANSocket can_socket(cfg.iface, cfg.use_fd);
		std::cout << "Connected. iface=" << cfg.iface << " scan_range=[" << cfg.first_can_id << ", "
				  << cfg.last_can_id << "] recv_offset=0x" << std::hex << cfg.recv_offset << std::dec
				  << " fd=" << (cfg.use_fd ? "on" : "off") << "\n";

		std::cout << "Scanning motor IDs...\n";
		std::vector<Motor> motors;
		motors.reserve(static_cast<size_t>(cfg.last_can_id - cfg.first_can_id + 1));
		for (uint32_t send_id = cfg.first_can_id; send_id <= cfg.last_can_id; ++send_id) {
			const auto limits = query_motor_limits(can_socket, cfg, send_id);
			if (!limits.has_value()) {
				// No valid limits response means no motor (or no response) at this ID.
				continue;
			}
			motors.emplace_back(*limits, send_id, send_id + cfg.recv_offset);
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
				motor, CAN_SFF_MASK, cfg.use_fd, cfg.custom_firmware);
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
		std::cout << "Firmware mode: "
				  << (cfg.custom_firmware ? "custom (16-16-16)" : "standard (16-12-12)") << "\n";

		// --- Firmware sanity check ---
		dm_collection.enable_all();
		receive_all_can_replies(can_socket, dm_collection.get_device_collection(), cfg.use_fd, 2000);
		dm_collection.refresh_all();
		receive_all_can_replies(can_socket, dm_collection.get_device_collection(), cfg.use_fd, 2000);

		constexpr double kTorqueSanityThreshold = 1.0;  // Nm; firmware mismatch causes large garbage
		bool firmware_ok = true;
		for (const auto& motor : motors) {
			if (std::abs(motor.get_torque()) > kTorqueSanityThreshold) {
				std::cerr << "[FIRMWARE MISMATCH] Motor 0x" << std::hex
						  << motor.get_send_can_id() << std::dec
						  << " reported torque=" << motor.get_torque()
						  << " Nm at rest (threshold: " << kTorqueSanityThreshold << " Nm).\n"
						  << "  Expected firmware: "
						  << (cfg.custom_firmware ? "custom (16-16-16)" : "standard (16-12-12)")
						  << ".\n"
						  << "  Try toggling --custom-firmware.\n";
				firmware_ok = false;
				break;
			}
		}
		if (!firmware_ok) {
			dm_collection.disable_all();
			receive_all_can_replies(can_socket, dm_collection.get_device_collection(), cfg.use_fd, 1000);
			return 1;
		}

		dm_collection.enable_all();
		receive_all_can_replies(can_socket, dm_collection.get_device_collection(), cfg.use_fd, 1000);

		const damiao_motor::MITParam mit_zero{0.0, 0.0, 0.0, 0.0, 0.0};
		std::vector<damiao_motor::MITParam> mit_zeros(motors.size(), mit_zero);

		unsigned long long counter = 0;
		constexpr unsigned long long kPrintEvery = 100;  // Print at ~10Hz while loop runs at 1kHz.

		auto t_start = std::chrono::steady_clock::now();
		auto next_tick = t_start;

		while (runloop) {
			next_tick += std::chrono::milliseconds(1);

			// Broadcast zero MIT command to all motors.
			// DM motors reply with state to control commands, so no explicit refresh here.
			dm_collection.mit_control_all(mit_zeros);
			receive_all_can_replies(can_socket, dm_collection.get_device_collection(), cfg.use_fd, 200);

			if (counter % kPrintEvery == 0) {
				std::cout << "\nSample " << counter << "\n";
				std::cout << "-----------------------------------------------\n";
				for (size_t m = 0; m < motors.size(); ++m) {
					const auto& motor = motors[m];
					std::cout << "0x" << std::hex << std::setw(8) << motor.get_send_can_id()
							  << std::dec;

					std::cout << std::fixed << std::setprecision(6) << std::setw(14)
							  << motor.get_position() << std::setw(14) << motor.get_velocity()
							  << std::setw(14) << motor.get_torque() << std::setw(8)
							  << motor.get_state_tmos() << motor.get_state_trotor() << "\n";
				}
				std::cout << "\n";
			}

			++counter;
			std::this_thread::sleep_until(next_tick);
		}

		auto t_end = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed = t_end - t_start;
		std::cout << "Elapsed time: " << elapsed.count() << " seconds\n";
		std::cout << "running frequency: " << (counter / elapsed.count()) << " Hz\n";

		dm_collection.disable_all();
		receive_all_can_replies(can_socket, dm_collection.get_device_collection(), cfg.use_fd, 1000);

	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << "\n";
		return 1;
	}

	return 0;
}
