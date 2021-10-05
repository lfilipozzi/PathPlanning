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
	};

	template <typename T>
	T Pose2D<T>::WrapTheta() const
	{
		return fmod(theta + M_PI, 2 * M_PI) - M_PI;
	}

	template <>
	int Pose2D<int>::WrapTheta() const
	{
		return theta;
	}

	template <typename T>
	Pose2D<T> operator-(const Pose2D<T>& lhs) { return { -lhs.x, -lhs.y, -lhs.theta }; }

	template <typename T>
	Pose2D<T> operator-(const Pose2D<T>& lhs, const Pose2D<T>& rhs)
	{
		Pose2D<T> out;

		// Translate
		auto tx = lhs.x - rhs.x;
		auto ty = lhs.y - rhs.y;
		auto& ttheta = rhs.theta;

		// Rotate
		out.x = tx * cos(ttheta) - ty * sin(ttheta);
		out.y = tx * sin(ttheta) + ty * cos(ttheta);
		out.theta = lhs.theta - rhs.theta;

		return out;
	}

	template <typename T>
	Pose2D<T> operator*(float lhs, const Pose2D<T>& rhs)
	{
		Pose2D<T> out;

		out.x = lhs * rhs.x;
		out.y = lhs * rhs.y;
		out.theta = lhs * rhs.theta;

		return out;
	}

	template <typename T>
	Pose2D<T> operator*(const Pose2D<T>& lhs, float rhs)
	{
		return rhs * lhs;
	}

	template <typename T>
	bool operator==(const Pose2D<T>& lhs, const Pose2D<T>& rhs)
	{
		return lhs.x == rhs.x && lhs.y == rhs.y && lhs.theta == rhs.theta;
	}

	template <typename T>
	bool operator!=(const Pose2D<T>& lhs, const Pose2D<T>& rhs)
	{
		return !(lhs == rhs);
	}
}

namespace std {
	template <typename T>
	struct hash<Planner::Pose2D<T>> {
		std::size_t operator()(const Planner::Pose2D<T>& pose) const
		{
			std::size_t seed = 0;
			HashCombine(seed, pose.x);
			HashCombine(seed, pose.y);
			HashCombine(seed, pose.WrapTheta());
			return seed;
		}
	};
}
