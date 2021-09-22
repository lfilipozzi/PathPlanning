#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "core/hash.h"

namespace Planner {
	template <typename T = double>
	struct Pose2D {
		Pose2D() = default;
		Pose2D(T x, T y, T theta) :
			x(x), y(y), theta(theta) { }

		T x = 0.0;
		T y = 0.0;
		T theta = 0.0;

		T WrapTheta() const;

		Pose2D operator+(const Pose2D& rhs) const { return { x + rhs.x, y + rhs.y, theta + rhs.theta }; }
		Pose2D operator-() const { return { -x, -y, -theta }; }
		Pose2D operator-(const Pose2D& rhs) const { return *this + (-rhs); }
		bool operator==(const Pose2D& rhs) const
		{
			return x == rhs.x && y == rhs.y && theta == rhs.theta;
		}
	};

	template <>
	double Pose2D<double>::WrapTheta() const
	{
		return fmod(theta + M_PI, 2 * M_PI) - M_PI;
	}

	template <>
	int Pose2D<int>::WrapTheta() const
	{
		return theta;
	}
}

namespace std {
	template <>
	struct hash<Planner::Pose2D<>> {
		std::size_t operator()(const Planner::Pose2D<>& pose) const
		{
			std::size_t seed = 0;
			HashCombine(seed, pose.x);
			HashCombine(seed, pose.y);
			HashCombine(seed, pose.WrapTheta());
			return seed;
		}
	};
}

namespace std {
	template <>
	struct hash<Planner::Pose2D<int>> {
		std::size_t operator()(const Planner::Pose2D<int>& pose) const
		{
			std::size_t seed = 0;
			HashCombine(seed, pose.x);
			HashCombine(seed, pose.y);
			HashCombine(seed, pose.WrapTheta());
			return seed;
		}
	};
}