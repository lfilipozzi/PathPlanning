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
		StateSpaceSE2() :
			StateSpace<Pose2D<>, 3>({ { { -100, 100 }, { -100, 100 }, { -M_PI, M_PI } } }) { }
		~StateSpaceSE2() = default;

		virtual State Interpolate(const State& from, const State& to, float ratio) override
		{
			State delta = to - from;
			State interp = {
				from.x + ratio * delta.x,
				from.y + ratio * delta.y,
				from.theta + ratio * delta.theta,
			};
			return interp;
		}

		virtual double ComputeDistance(const State& from, const State& to) override
		{
			State delta = to - from;
			return sqrt(powf(delta.x, 2) + powf(delta.y, 2));
		}
	};
}
