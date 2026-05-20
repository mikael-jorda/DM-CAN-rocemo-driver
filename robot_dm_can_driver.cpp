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
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <canbus/can_socket.hpp>
#include <damiao_motor/dm_motor.hpp>
#include <damiao_motor/dm_motor_constants.hpp>
#include <damiao_motor/dm_motor_control.hpp>

namespace {

using damiao_motor::Motor;
using damiao_motor::MotorType;
using damiao_motor::RID;
using damiao_motor::StateResult;

struct Config {
	std::string iface = "can0";
	bool use_fd = true;
	MotorType motor_type = MotorType::DM4310;
	uint32_t min_id = 1;
	uint32_t max_id = 32;
	uint32_t recv_offset = 0x10;
	int samples = 50;
	int interval_ms = 100;
	bool custom_firmware = false;  // 16-16-16 encoding instead of standard 16-12-12
};

const std::map<std::string, MotorType> kMotorTypeMap = {
	{"DM3507", MotorType::DM3507},         {"DM4310", MotorType::DM4310},
	{"DM4310_48V", MotorType::DM4310_48V}, {"DM4340", MotorType::DM4340},
	{"DM4340_48V", MotorType::DM4340_48V}, {"DM6006", MotorType::DM6006},
	{"DM8006", MotorType::DM8006},         {"DM8009", MotorType::DM8009},
	{"DM10010L", MotorType::DM10010L},     {"DM10010", MotorType::DM10010},
	{"DMH3510", MotorType::DMH3510},       {"DMH6215", MotorType::DMH6215},
	{"DMG6220", MotorType::DMG6220},
};

void print_usage(const char* prog) {
	std::cout << "Usage: " << prog << " [options]\n\n"
			  << "Options:\n"
			  << "  --iface <name>         CAN interface (default: can0)\n"
			  << "  --min-id <value>       Minimum motor ID to scan (default: 1)\n"
			  << "  --max-id <value>       Maximum motor ID to scan (default: 32)\n"
			  << "  --recv-offset <value>  recv_id = send_id + offset (default: 0x10)\n"
			  << "  --motor-type <name>    Motor type (default: DM4310)\n"
			  << "  --samples <n>          Number of state samples (default: 50)\n"
			  << "  --interval-ms <n>      Interval between samples (default: 100)\n"
			  << "  --no-fd                Use classic CAN instead of CAN-FD\n"		  
			  << "  --custom-firmware      Use 16-16-16 state encoding (default: 16-12-12)\n"			  << "  --help                 Show this help\n\n"
			  << "Example:\n"
			  << "  " << prog
			  << " --iface can0 --min-id 1 --max-id 32 --motor-type DM4310 "
				 "--samples 100 --interval-ms 50\n";
}

bool parse_u32(const std::string& s, uint32_t& out) {
	try {
		size_t idx = 0;
		const unsigned long v = std::stoul(s, &idx, 0);
		if (idx != s.size()) return false;
		out = static_cast<uint32_t>(v);
		return true;
	} catch (...) {
		return false;
	}
}

bool parse_i32(const std::string& s, int& out) {
	try {
		size_t idx = 0;
		const int v = std::stoi(s, &idx, 0);
		if (idx != s.size()) return false;
		out = v;
		return true;
	} catch (...) {
		return false;
	}
}

bool parse_args(int argc, char** argv, Config& cfg) {
	for (int i = 1; i < argc; ++i) {
		const std::string arg = argv[i];

		if (arg == "--help") {
			print_usage(argv[0]);
			return false;
		}

		if (arg == "--no-fd") {
			cfg.use_fd = false;
			continue;
		}

		if (arg == "--custom-firmware") {
			cfg.custom_firmware = true;
			continue;
		}

		if (i + 1 >= argc) {
			std::cerr << "Missing value for argument: " << arg << "\n";
			return false;
		}

		const std::string value = argv[++i];
		if (arg == "--iface") {
			cfg.iface = value;
		} else if (arg == "--min-id") {
			if (!parse_u32(value, cfg.min_id) || cfg.min_id == 0) {
				std::cerr << "Invalid --min-id: " << value << "\n";
				return false;
			}
		} else if (arg == "--max-id") {
			if (!parse_u32(value, cfg.max_id) || cfg.max_id == 0) {
				std::cerr << "Invalid --max-id: " << value << "\n";
				return false;
			}
		} else if (arg == "--recv-offset") {
			if (!parse_u32(value, cfg.recv_offset)) {
				std::cerr << "Invalid --recv-offset: " << value << "\n";
				return false;
			}
		} else if (arg == "--motor-type") {
			const auto it = kMotorTypeMap.find(value);
			if (it == kMotorTypeMap.end()) {
				std::cerr << "Invalid --motor-type: " << value << "\n";
				return false;
			}
			cfg.motor_type = it->second;
		} else if (arg == "--samples") {
			if (!parse_i32(value, cfg.samples) || cfg.samples <= 0) {
				std::cerr << "Invalid --samples: " << value << "\n";
				return false;
			}
		} else if (arg == "--interval-ms") {
			if (!parse_i32(value, cfg.interval_ms) || cfg.interval_ms < 0) {
				std::cerr << "Invalid --interval-ms: " << value << "\n";
				return false;
			}
		} else {
			std::cerr << "Unknown argument: " << arg << "\n";
			return false;
		}
	}

	if (cfg.min_id > cfg.max_id) {
		std::cerr << "Invalid range: --min-id must be <= --max-id\n";
		return false;
	}

	return true;
}

std::string rid_name(int r) {
	switch (static_cast<RID>(r)) {
		case RID::UV_Value:
			return "UV_Value";
		case RID::KT_Value:
			return "KT_Value";
		case RID::OT_Value:
			return "OT_Value";
		case RID::OC_Value:
			return "OC_Value";
		case RID::ACC:
			return "ACC";
		case RID::DEC:
			return "DEC";
		case RID::MAX_SPD:
			return "MAX_SPD";
		case RID::MST_ID:
			return "MST_ID";
		case RID::ESC_ID:
			return "ESC_ID";
		case RID::TIMEOUT:
			return "TIMEOUT";
		case RID::CTRL_MODE:
			return "CTRL_MODE";
		case RID::Damp:
			return "Damp";
		case RID::Inertia:
			return "Inertia";
		case RID::hw_ver:
			return "hw_ver";
		case RID::sw_ver:
			return "sw_ver";
		case RID::SN:
			return "SN";
		case RID::NPP:
			return "NPP";
		case RID::Rs:
			return "Rs";
		case RID::LS:
			return "LS";
		case RID::Flux:
			return "Flux";
		case RID::Gr:
			return "Gr";
		case RID::PMAX:
			return "PMAX";
		case RID::VMAX:
			return "VMAX";
		case RID::TMAX:
			return "TMAX";
		case RID::I_BW:
			return "I_BW";
		case RID::KP_ASR:
			return "KP_ASR";
		case RID::KI_ASR:
			return "KI_ASR";
		case RID::KP_APR:
			return "KP_APR";
		case RID::KI_APR:
			return "KI_APR";
		case RID::OV_Value:
			return "OV_Value";
		case RID::GREF:
			return "GREF";
		case RID::Deta:
			return "Deta";
		case RID::V_BW:
			return "V_BW";
		case RID::IQ_c1:
			return "IQ_c1";
		case RID::VL_c1:
			return "VL_c1";
		case RID::can_br:
			return "can_br";
		case RID::sub_ver:
			return "sub_ver";
		case RID::u_off:
			return "u_off";
		case RID::v_off:
			return "v_off";
		case RID::k1:
			return "k1";
		case RID::k2:
			return "k2";
		case RID::m_off:
			return "m_off";
		case RID::dir:
			return "dir";
		case RID::p_m:
			return "p_m";
		case RID::xout:
			return "xout";
		default:
			return "RID_" + std::to_string(r);
	}
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

std::vector<Motor> scan_present_motors(canbus::CANSocket& can_socket,
										   const Config& cfg) {
	std::vector<Motor> motors;
	motors.reserve(static_cast<size_t>(cfg.max_id - cfg.min_id + 1));

	for (uint32_t send_id = cfg.min_id; send_id <= cfg.max_id; ++send_id) {
		Motor candidate(cfg.motor_type, send_id, send_id + cfg.recv_offset);

		// Probe a stable register first to identify active motors in the chain.
		const auto probe = query_register_with_retry(
			can_socket, cfg.use_fd, candidate, static_cast<int>(RID::MST_ID), 2);
		if (probe.has_value()) {
			motors.push_back(candidate);
		}
	}

	return motors;
}

}  // namespace

int main(int argc, char** argv) {
	Config cfg;
	if (!parse_args(argc, argv, cfg)) {
		if (argc > 1 && std::string(argv[1]) == "--help") {
			return 0;
		}
		print_usage(argv[0]);
		return 1;
	}

	try {
		canbus::CANSocket can_socket(cfg.iface, cfg.use_fd);
		std::cout << "Connected. iface=" << cfg.iface << " scan_range=[" << cfg.min_id << ", "
				  << cfg.max_id << "] recv_offset=0x" << std::hex << cfg.recv_offset << std::dec
				  << " fd=" << (cfg.use_fd ? "on" : "off") << "\n";

		std::cout << "Scanning motor IDs...\n";
		std::vector<Motor> motors = scan_present_motors(can_socket, cfg);
		if (motors.empty()) {
			std::cout << "No motors responded in the scan range.\n";
			return 0;
		}

		std::cout << "Detected motor IDs: ";
		for (size_t i = 0; i < motors.size(); ++i) {
			std::cout << "0x" << std::hex << motors[i].get_send_can_id() << std::dec;
			if (i + 1 < motors.size()) std::cout << ", ";
		}
		std::cout << "\n";
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
							  << rid_name(rid) << std::setprecision(12) << it->second << "\n";
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

		for (int i = 0; i < cfg.samples; ++i) {
			std::vector<std::optional<StateResult>> batch_states;
			batch_states.reserve(motors.size());

			for (const auto& motor : motors) {
				batch_states.push_back(read_state_sample(can_socket, cfg.use_fd, motor, cfg.custom_firmware));
			}

			std::cout << "\nSample " << i << "\n";
			std::cout << "-----------------------------------------------\n";
			for (size_t m = 0; m < motors.size(); ++m) {
				const auto& motor = motors[m];
				const auto& state = batch_states[m];
				std::cout << "0x" << std::hex << std::setw(8) << motor.get_send_can_id()
						  << std::dec;

				if (state.has_value()) {
					std::cout << std::fixed << std::setprecision(6) << std::setw(14)
							  << state->position << std::setw(14) << state->velocity
							  << std::setw(14) << state->torque << std::setw(8) << state->t_mos
							  << state->t_rotor << "\n";
				} else {
					std::cout << std::setw(14) << "n/a" << std::setw(14) << "n/a"
							  << std::setw(14) << "n/a" << std::setw(8) << "n/a"
							  << "n/a\n";
				}
			}
			std::cout << "\n";

			if (cfg.interval_ms > 0) {
				std::this_thread::sleep_for(std::chrono::milliseconds(cfg.interval_ms));
			}
		}

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
