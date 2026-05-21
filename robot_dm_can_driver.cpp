#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <signal.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <canbus/can_socket.hpp>
#include <damiao_motor/dm_motor.hpp>
#include <damiao_motor/dm_motor_constants.hpp>
#include <damiao_motor/dm_motor_control.hpp>
#include <damiao_motor/dm_motor_device.hpp>
#include <damiao_motor/dm_motor_device_collection.hpp>

#include "config/config_structs.hpp"
#include <glaze/glaze.hpp>

namespace {

bool runloop = true;
void signal_handler(int signal) {
	if (signal == SIGINT) {
		std::cout << "SIGINT received, exiting...\n";
		runloop = false;
	}
}

using damiao_motor::Motor;
using damiao_motor::LimitParam;
using damiao_motor::RID;
using damiao_motor::StateResult;

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

	if (cfg.min_id > cfg.max_id) {
		std::cerr << "Invalid config: min_id must be <= max_id\n";
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

std::optional<StateResult> read_state_sample(canbus::CANSocket& sock, bool use_fd,
											 const Motor& motor, bool custom_firmware = false) {
	using damiao_motor::CanPacketDecoder;
	using damiao_motor::CanPacketEncoder;

	send_packet(sock, use_fd, CanPacketEncoder::create_refresh_command(motor));

	std::optional<StateResult> latest;
	for (int spin = 0; spin < 20; ++spin) {
		if (!sock.is_data_available(2000)) continue;

		std::vector<uint8_t> data;
		canid_t can_id = 0;
		if (!recv_one_frame(sock, use_fd, data, can_id)) continue;

		if (data.size() < 8) continue;

		const uint8_t payload_motor_id = data[0] & 0x0F;
		if (payload_motor_id != static_cast<uint8_t>(motor.get_send_can_id() & 0x0F)) continue;

		const auto parsed = CanPacketDecoder::parse_motor_state_data(motor, data, custom_firmware);
		if (parsed.valid) {
			latest = parsed;
		}

		(void)can_id;
	}

	return latest;
}

void collect_state_batch(canbus::CANSocket& can_socket,
						 damiao_motor::DMDeviceCollection& dm_collection,
						 size_t motor_count,
						 bool use_fd,
						 int timeout_us_per_poll = 5000,
						 int max_polls = 100) {
	std::set<canid_t> seen_ids;

	for (int i = 0; i < max_polls && seen_ids.size() < motor_count; ++i) {
		if (!can_socket.is_data_available(timeout_us_per_poll)) continue;

		if (use_fd) {
			canfd_frame frame{};
			if (!can_socket.read_canfd_frame(frame)) continue;
			dm_collection.get_device_collection().dispatch_frame_callback(frame);
			seen_ids.insert(frame.can_id);
		} else {
			can_frame frame{};
			if (!can_socket.read_can_frame(frame)) continue;
			dm_collection.get_device_collection().dispatch_frame_callback(frame);
			seen_ids.insert(frame.can_id);
		}
	}
}

std::vector<uint32_t> scan_present_motor_ids(canbus::CANSocket& can_socket, const DMCanRobotDriverConfig& cfg) {
	std::vector<uint32_t> motor_ids;
	motor_ids.reserve(static_cast<size_t>(cfg.max_id - cfg.min_id + 1));

	for (uint32_t send_id = cfg.min_id; send_id <= cfg.max_id; ++send_id) {
		Motor candidate(kProbeLimits, send_id, send_id + cfg.recv_offset);

		// Probe a stable register first to identify active motors in the chain.
		const auto probe = query_register_with_retry(
			can_socket, cfg.use_fd, candidate, static_cast<int>(RID::MST_ID), 2);
		if (probe.has_value()) {
			motor_ids.push_back(send_id);
		}
	}

	return motor_ids;
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
	DMCanRobotDriverConfig cfg;
	if (!parse_config(argc, argv, cfg)) {
		if (argc > 1 && std::string(argv[1]) == "--help") {
			return 0;
		}
		print_usage(argv[0]);
		return 1;
	}

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);

	try {
		canbus::CANSocket can_socket(cfg.iface, cfg.use_fd);
		std::cout << "Connected. iface=" << cfg.iface << " scan_range=[" << cfg.min_id << ", "
				  << cfg.max_id << "] recv_offset=0x" << std::hex << cfg.recv_offset << std::dec
				  << " fd=" << (cfg.use_fd ? "on" : "off") << "\n";

		std::cout << "Scanning motor IDs...\n";
		const std::vector<uint32_t> detected_ids = scan_present_motor_ids(can_socket, cfg);
		if (detected_ids.empty()) {
			std::cout << "No motors responded in the scan range.\n";
			return 0;
		}

		std::vector<Motor> motors;
		motors.reserve(detected_ids.size());
		for (uint32_t send_id : detected_ids) {
			const auto limits = query_motor_limits(can_socket, cfg, send_id);
			if (!limits.has_value()) {
				std::cerr << "Failed to read PMAX/VMAX/TMAX for motor 0x" << std::hex << send_id
						  << std::dec << ". Skipping this motor.\n";
				continue;
			}
			motors.emplace_back(*limits, send_id, send_id + cfg.recv_offset);
		}

		if (motors.empty()) {
			std::cerr << "No motors had valid PMAX/VMAX/TMAX limits. Aborting.\n";
			return 1;
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
		// Enable motors briefly to get a state reply, then verify that torque
		// reads back as (near) zero at rest. A non-zero torque strongly suggests
		// the chosen firmware encoding does not match what the motor is running.
		for (const auto& motor : motors) {
			send_packet(can_socket, cfg.use_fd,
						damiao_motor::CanPacketEncoder::create_enable_command(motor));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		constexpr double kTorqueSanityThreshold = 1.0;  // Nm; firmware mismatch causes large garbage
		bool firmware_ok = true;
		for (const auto& motor : motors) {
			const auto state = read_state_sample(can_socket, cfg.use_fd, motor, cfg.custom_firmware);
			if (!state.has_value()) continue;
			if (std::abs(state->torque) > kTorqueSanityThreshold) {
				std::cerr << "[FIRMWARE MISMATCH] Motor 0x" << std::hex
						  << motor.get_send_can_id() << std::dec
						  << " reported torque=" << state->torque
						  << " Nm at rest (threshold: " << kTorqueSanityThreshold << " Nm).\n"
						  << "  Expected firmware: "
						  << (cfg.custom_firmware ? "custom (16-16-16)" : "standard (16-12-12)")
						  << ".\n"
						  << "  Try toggling --custom-firmware.\n";
				firmware_ok = false;
			}
		}
		if (!firmware_ok) {
			for (const auto& motor : motors) {
				send_packet(can_socket, cfg.use_fd,
							damiao_motor::CanPacketEncoder::create_disable_command(motor));
			}
			return 1;
		}

		std::map<uint32_t, std::map<int, double>> register_values;

		// Query all known register IDs for each motor in the chain.
		for (const auto& motor : motors) {
			for (int rid = 0; rid < static_cast<int>(RID::COUNT); ++rid) {
				const auto val = query_register_with_retry(can_socket, cfg.use_fd, motor, rid);
				if (val.has_value()) {
					register_values[motor.get_send_can_id()][rid] = *val;
				}
			}
		}

		std::cout << "\n=== Register values that responded ===\n";
		for (const auto& motor : motors) {
			const uint32_t send_id = motor.get_send_can_id();
			std::cout << "\nMotor send_id=0x" << std::hex << send_id << std::dec << "\n";
			std::cout << std::left << std::setw(12) << "RID" << std::setw(16) << "Name"
					  << "Value\n";
			std::cout << "------------------------------------------------\n";

			int printed = 0;
			const auto it_map = register_values.find(send_id);
			if (it_map != register_values.end()) {
				for (int rid = 0; rid < static_cast<int>(RID::COUNT); ++rid) {
					const auto it = it_map->second.find(rid);
					if (it == it_map->second.end()) continue;

					std::cout << std::left << std::setw(12) << rid << std::setw(16)
							  << damiao_motor::rid_name(rid) << std::setprecision(12) << it->second << "\n";
					++printed;
				}
			}

			if (printed == 0) {
				std::cout << "No register response for this motor.\n";
			}
		}

		for (const auto& motor : motors) {
			send_packet(can_socket, cfg.use_fd,
						damiao_motor::CanPacketEncoder::create_enable_command(motor));
		}
		// Motors were already enabled by the firmware check; this re-enables
		// any that may have been left disabled after the sanity check pass.
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		std::cout << "\n=== Live state (position / velocity / torque) ===\n";
		std::cout << std::left << std::setw(10) << "MotorID" << std::setw(14) << "Pos(rad)"
				  << std::setw(14) << "Vel(rad/s)" << std::setw(14) << "Tau(Nm)"
				  << std::setw(8) << "Tmos" << "Trotor\n";

		const damiao_motor::MITParam mit_zero{0.0, 0.0, 0.0, 0.0, 0.0};
		std::vector<damiao_motor::MITParam> mit_zeros(motors.size(), mit_zero);

		unsigned long long counter = 0;

		auto t_start = std::chrono::high_resolution_clock::now();

		while (runloop) {
			// Broadcast zero MIT command to all motors, then refresh and collect replies.
			dm_collection.mit_control_all(mit_zeros);
			// dm_collection.refresh_all();
			collect_state_batch(can_socket, dm_collection, motors.size(), cfg.use_fd);

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

			// if (cfg.interval_ms > 0) {
			// 	std::this_thread::sleep_for(std::chrono::milliseconds(cfg.interval_ms));
			// }
			++counter;
		}

		auto t_end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = t_end - t_start;
		std::cout << "Elapsed time: " << elapsed.count() << " seconds\n";
		std::cout << "running frequency: " << (counter / elapsed.count()) << " Hz\n";

		for (const auto& motor : motors) {
			send_packet(can_socket, cfg.use_fd,
						damiao_motor::CanPacketEncoder::create_disable_command(motor));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << "\n";
		return 1;
	}

	return 0;
}
