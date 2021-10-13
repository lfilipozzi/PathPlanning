#pragma once

#include "core/base.h"
#include "geometry/pose.h"
#include "state_space.h"

#include <random>

namespace Planner {

	/**
	 * @brief Configuration space in the special Euclidean group SE(2).
	 */
	class StateSpaceSE2 : public StateSpace<Pose2D<>, 3> {
		using State = Pose2D<>;

	public:
		StateSpaceSE2(std::array<std::array<double, 2>, 3> bounds = { { { -100, 100 }, { -100, 100 }, { -M_PI, M_PI } } }) :
			StateSpace<Pose2D<>, 3>(bounds) { }
		~StateSpaceSE2() = default;

		virtual double ComputeDistance(const State& from, const State& to) override
		{
			State delta = to - from;
			return sqrt(powf(delta.x, 2) + powf(delta.y, 2));
		}
	};
}
