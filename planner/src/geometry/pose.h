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

	/// @brief Add the Pose @rhs after @lhs.
	/// @warning This operation is not commutative.
	template <typename T>
	Pose2D<T> operator+(const Pose2D<T>& lhs, const Pose2D<T>& rhs)
	{
		Pose2D<T> out;

		// Rotate
		auto& theta = lhs.theta;
		out.x = rhs.x * cos(theta) - rhs.y * sin(theta);
		out.y = rhs.x * sin(theta) + rhs.y * cos(theta);
		out.theta = rhs.theta;

		// Translate
		out.x += lhs.x;
		out.y += lhs.y;
		out.theta += lhs.theta;

		return out;
	}

	/// @brief Compute the pose p such that @rhs + p = @lhs.
	template <typename T>
	Pose2D<T> operator-(const Pose2D<T>& lhs, const Pose2D<T>& rhs)
	{
		Pose2D<T> out;

		// Translate
		auto x = lhs.x - rhs.x;
		auto y = lhs.y - rhs.y;
		auto& theta = rhs.theta;

		// Rotate
		out.x = x * cos(theta) + y * sin(theta);
		out.y = -x * sin(theta) + y * cos(theta);
		out.theta = lhs.theta - rhs.theta;

		return out;
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
