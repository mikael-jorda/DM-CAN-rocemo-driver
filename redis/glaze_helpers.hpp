#pragma once

#include <Eigen/Dense>
#include <glaze/glaze.hpp>

template <>
struct glz::meta<Eigen::Quaterniond> {
	static constexpr auto value = glz::object(
		// Use reference-preserving lambdas with explicit return types
		"w", [](auto& self) -> auto& { return self.w(); }, "x",
		[](auto& self) -> auto& { return self.x(); }, "y",
		[](auto& self) -> auto& { return self.y(); }, "z",
		[](auto& self) -> auto& { return self.z(); });
};
