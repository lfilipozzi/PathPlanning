#pragma once

#include <eigen3/Eigen/Dense>

#include "core/hash.h"

using Point2D = Eigen::Vector2d;
using Point2DInt = Eigen::Vector2i;

namespace std {
	template <>
	struct hash<Point2D> {
		std::size_t operator()(const Point2D& state) const
		{
			std::size_t seed = 0;
			HashCombine(seed, state.x());
			HashCombine(seed, state.y());
			return seed;
		}
	};

	template <>
	struct hash<Point2DInt> {
		std::size_t operator()(const Point2DInt& state) const
		{
			std::size_t seed = 0;
			HashCombine(seed, state.x());
			HashCombine(seed, state.y());
			return seed;
		}
	};
}
