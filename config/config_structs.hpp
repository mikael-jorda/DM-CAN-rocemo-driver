#pragma once

#include <string>
#include <vector>
#include <cstdint>

struct DMCanRobotDriverConfig {
	// can related config
	std::string iface = "can0";
	bool use_fd = true;
	std::vector<std::pair<uint32_t, uint32_t>> arm_can_id_ranges = {{1, 7}};  // inclusive ranges of CAN IDs for the arm motors
	std::vector<uint32_t> gripper_can_ids = {8};  // CAN IDs for the gripper motors
	uint32_t recv_offset = 0x10;   // unused in custom arm
	bool arm_uses_custom_firmware = true;  // 16-16-16 encoding instead of standard 16-12-12
	bool gripper_uses_custom_firmware = false;
	// robot related config
	std::string robot_model_file;
	std::string robot_name = "DMRobot";
};