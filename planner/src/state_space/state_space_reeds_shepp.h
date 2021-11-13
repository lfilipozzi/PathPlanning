#pragma once

#include "state_space/state_space.h"
#include "paths/path_reeds_shepp.h"

namespace Planner {
	class StateSpaceReedsShepp : public PlanarStateSpace {
	public:
		StateSpaceReedsShepp(const std::array<Pose2d, 2>& bounds, double minTurningRadius = 1.0) :
			PlanarStateSpace(bounds),
			minTurningRadius(minTurningRadius) { }
		StateSpaceReedsShepp(const Pose2d& lb, const Pose2d& ub, double minTurningRadius = 1.0) :
			StateSpaceReedsShepp(std::array<Pose2d, 2>({ lb, ub }), minTurningRadius) { }
		~StateSpaceReedsShepp() = default;

		/// @brief Compute the shortest distance from the pose @from to @to
		/// when driving Reeds-Shepp paths.
		virtual double ComputeDistance(const Pose2d& from, const Pose2d& to) override
		{
			return ReedsShepp::Solver::GetShortestDistance(from, to, minTurningRadius);
		}

	public:
		const double minTurningRadius;
	};
}
