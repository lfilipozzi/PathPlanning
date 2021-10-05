#pragma once

#include "geometry/2dplane.h"
#include "a_star.h"

namespace Planner {
	class AStarStateSpace2D : public AStarStateSpace<Point2DInt> {
	public:
		virtual std::vector<std::tuple<Point2DInt, double>> GetNeighborStates(const Point2DInt& state) override
		{
			double delta = 1.0;
			std::vector<std::tuple<Point2DInt, double>> vec;
			vec.reserve(8);
			vec.push_back({ { state.x() + 1.0 * delta, state.y() + 1.0 * delta }, delta * std::sqrt(2.0) });
			vec.push_back({ { state.x() + 1.0 * delta, state.y() + 0.0 * delta }, delta });
			vec.push_back({ { state.x() + 1.0 * delta, state.y() - 1.0 * delta }, delta * std::sqrt(2.0) });
			vec.push_back({ { state.x() + 0.0 * delta, state.y() - 1.0 * delta }, delta });
			vec.push_back({ { state.x() - 1.0 * delta, state.y() - 1.0 * delta }, delta * std::sqrt(2.0) });
			vec.push_back({ { state.x() - 1.0 * delta, state.y() + 0.0 * delta }, delta });
			vec.push_back({ { state.x() - 1.0 * delta, state.y() + 1.0 * delta }, delta * std::sqrt(2.0) });
			vec.push_back({ { state.x() + 0.0 * delta, state.y() + 1.0 * delta }, delta });
			return vec;
		}
	};
}
