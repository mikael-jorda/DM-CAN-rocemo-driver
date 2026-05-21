#pragma once

#include <string>

struct DMCanRobotDriverConfig {
	std::string iface = "can0";
	bool use_fd = true;
	uint32_t first_can_id = 1;
	uint32_t last_can_id = 7;
	uint32_t recv_offset = 0x10;
	bool custom_firmware = false;  // 16-16-16 encoding instead of standard 16-12-12
};