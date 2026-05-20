#pragma once

#include <string>

struct DMCanRobotDriverConfig {
	std::string iface = "can0";
	bool use_fd = true;
	uint32_t min_id = 1;
	uint32_t max_id = 32;
	uint32_t recv_offset = 0x10;
	int samples = 50;
	int interval_ms = 100;
	bool custom_firmware = false;  // 16-16-16 encoding instead of standard 16-12-12
};