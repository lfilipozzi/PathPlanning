#pragma once

#include "core/base.h"
#include "geometry/2dplane.h"
#include "state_space.h"

namespace Planner {

	/// @brief Configuration space in the special Euclidean group SE(2).
	class StateSpaceSE2 : public PlanarStateSpace {
	public:
		StateSpaceSE2(std::array<Pose2d, 2> bounds = { Pose2d(-100, -100, -M_PI), Pose2d(100, 100, M_PI) }) :
			PlanarStateSpace(bounds) { }
		~StateSpaceSE2() = default;

		virtual double ComputeDistance(const Pose2d& from, const Pose2d& to) override
		{
			auto delta = to.position - from.position;
			return delta.norm();
		}
	};
}
