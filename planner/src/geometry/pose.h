#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "core/hash.h"

namespace Planner {
	struct Pose {
		Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
		Pose(int x, int y, int theta) : x(x), y(y), theta(theta) {}

		double x = 0.0;
		double y = 0.0;
		double theta = 0.0;
		
		double WrapTheta() const { return fmod(theta + M_PI, M_2_PI) - M_PI; }

		Pose operator +(const Pose& other) const { return { x + other.x, y + other.y, theta + other.theta }; }
		Pose operator -() const { return { -x, -y, -theta }; }
		Pose operator -(const Pose& b) const { return *this + (-b); }
	};
}

namespace std {
	template <>
	struct hash<Planner::Pose> {
		std::size_t operator()(const Planner::Pose& pose) const
		{
			std::size_t seed = 0;
			HashCombine(seed, pose.x);
			HashCombine(seed, pose.y);
			HashCombine(seed, fmod(pose.theta, 2 * M_PI));
			return seed;
		}
	};
}
