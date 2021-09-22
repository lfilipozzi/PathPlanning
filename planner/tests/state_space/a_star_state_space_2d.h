#pragma once

#include "geometry/2dplane.h"
#include "state_space_2d.h"
#include "a_star.h"

namespace Planner {
	class AStarStateSpace2D : public AStarStateSpace<Point2DInt>, public StateSpace2D<Point2DInt> {
	public:
		virtual std::tuple<double, bool> GetTransition(const Point2DInt& source, const Point2DInt& target) override
		{
			auto delta = source - target;
			double dist = sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
			return { dist, true };
		}

		virtual std::vector<Point2DInt> GetNeighborStates(const Point2DInt& state) override
		{
			double delta = 1.0;
			std::vector<Point2DInt> neighbors;
			neighbors.reserve(8);
			neighbors.push_back({ state.x() + 1.0 * delta, state.y() + 1.0 * delta });
			neighbors.push_back({ state.x() + 1.0 * delta, state.y() + 0.0 * delta });
			neighbors.push_back({ state.x() + 1.0 * delta, state.y() - 1.0 * delta });
			neighbors.push_back({ state.x() + 0.0 * delta, state.y() - 1.0 * delta });
			neighbors.push_back({ state.x() - 1.0 * delta, state.y() - 1.0 * delta });
			neighbors.push_back({ state.x() - 1.0 * delta, state.y() + 0.0 * delta });
			neighbors.push_back({ state.x() - 1.0 * delta, state.y() + 1.0 * delta });
			neighbors.push_back({ state.x() + 0.0 * delta, state.y() + 1.0 * delta });
			return neighbors;
		}
	};
}
