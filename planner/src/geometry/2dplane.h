#pragma once

#include "core/hash.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace Planner {
	template <typename T>
	using Point2 = Eigen::Matrix<T, 2, 1>;
	using Point2d = Eigen::Vector2d;
	using Point2i = Eigen::Vector2i;

	template <typename T>
	struct Pose2 {
		Pose2() = default;
		Pose2(Point2<T> pos, T orientation) :
			position(pos), theta(orientation) { theta = WrapTheta(); }
		Pose2(T x, T y, T orientation) :
			position({ x, y }), theta(orientation) { theta = WrapTheta(); }

		Point2<T> position = Point2<T>();
		T theta = 0.0;

		inline T& x() { return position.x(); }
		inline const T& x() const { return position.x(); }

		inline T& y() { return position.y(); }
		inline const T& y() const { return position.y(); }

		T WrapTheta() const;
	};

	template <typename T>
	T Pose2<T>::WrapTheta() const
	{
		T t = theta;
		while (t > M_PI)
			t -= 2 * M_PI;
		while (t < -M_PI)
			t += 2 * M_PI;
		return t;
	}

	/// @brief Add the Pose @rhs after @lhs.
	/// @warning This operation is not commutative.
	template <typename T>
	Pose2<T> operator+(const Pose2<T>& lhs, const Pose2<T>& rhs)
	{
		// Rotate
		Pose2<T> out(
			Eigen::Rotation2D<T>(lhs.theta) * rhs.position,
			lhs.theta + rhs.theta);

		// Translate
		out.position += lhs.position;

		out.theta = out.WrapTheta();
		return out;
	}

	/// @brief Compute the pose p such that @rhs + p = @lhs.
	template <typename T>
	Pose2<T> operator-(const Pose2<T>& lhs, const Pose2<T>& rhs)
	{
		// Translate
		auto deltaPosition = lhs.position - rhs.position;

		// Rotate
		Pose2<T> out(
			Eigen::Rotation2D<T>(-rhs.theta) * deltaPosition,
			lhs.theta - rhs.theta);

		out.theta = out.WrapTheta();
		return out;
	}

	template <typename T>
	bool operator==(const Pose2<T>& lhs, const Pose2<T>& rhs)
	{
		return lhs.position == rhs.position && lhs.theta == rhs.theta;
	}

	template <typename T>
	bool operator!=(const Pose2<T>& lhs, const Pose2<T>& rhs)
	{
		return !(lhs == rhs);
	}

	using Pose2d = Pose2<double>;
	using Pose2i = Pose2<int>;
}

namespace std {
	template <typename T>
	struct hash<Planner::Point2<T>> {
		std::size_t operator()(const Planner::Point2<T>& state) const
		{
			std::size_t seed = 0;
			HashCombine(seed, state.x());
			HashCombine(seed, state.y());
			return seed;
		}
	};

	template <typename T>
	struct equal_to<Planner::Point2<T>> {
		bool operator()(const Planner::Point2<T>& lhs, const Planner::Point2<T>& rhs) const
		{
			return lhs.x() == rhs.x() && lhs.y() == rhs.y();
		}
	};

	template <typename T>
	struct hash<Planner::Pose2<T>> {
		std::size_t operator()(const Planner::Pose2<T>& pose) const
		{
			std::size_t seed = 0;
			HashCombine(seed, pose.position);
			HashCombine(seed, pose.WrapTheta());
			return seed;
		}
	};

	template <typename T>
	struct equal_to<Planner::Pose2<T>> {
		bool operator()(const Planner::Pose2<T>& lhs, const Planner::Pose2<T>& rhs) const
		{
			return lhs.position == rhs.position && lhs.WrapTheta() == rhs.WrapTheta();
		}
	};
}
