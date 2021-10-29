#pragma once

#include "geometry/2dplane.h"
#include "algo/a_star.h"

namespace Planner {
	class AStarStatePropagator2D : public AStarStatePropagator<Point2i> {
	public:
		virtual std::vector<std::tuple<Point2i, double>> GetNeighborStates(const Point2i& state) override
		{
			double delta = 1.0;
			std::vector<std::tuple<Point2i, double>> vec;
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
