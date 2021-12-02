#pragma once

#include "geometry/2dplane.h"
#include "algo/a_star.h"

namespace Planner {
	class AStarStatePropagator2D : public AStarStatePropagator<Point2i, NullAction> {
	public:
		virtual std::vector<std::tuple<Point2i, NullAction, double>> GetNeighborStates(const Point2i& state) override
		{
			constexpr int delta = 1;
			std::vector<std::tuple<Point2i, NullAction, double>> vec;
			vec.reserve(8);
			vec.push_back({ { state.x() + 1 * delta, state.y() + 1 * delta }, NullAction(), delta * std::sqrt(2.0) });
			vec.push_back({ { state.x() + 1 * delta, state.y() + 0 * delta }, NullAction(), delta });
			vec.push_back({ { state.x() + 1 * delta, state.y() - 1 * delta }, NullAction(), delta * std::sqrt(2.0) });
			vec.push_back({ { state.x() + 0 * delta, state.y() - 1 * delta }, NullAction(), delta });
			vec.push_back({ { state.x() - 1 * delta, state.y() - 1 * delta }, NullAction(), delta * std::sqrt(2.0) });
			vec.push_back({ { state.x() - 1 * delta, state.y() + 0 * delta }, NullAction(), delta });
			vec.push_back({ { state.x() - 1 * delta, state.y() + 1 * delta }, NullAction(), delta * std::sqrt(2.0) });
			vec.push_back({ { state.x() + 0 * delta, state.y() + 1 * delta }, NullAction(), delta });
			return vec;
		}
	};
}
