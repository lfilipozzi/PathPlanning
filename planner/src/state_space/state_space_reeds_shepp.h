#pragma once

#include "state_space/state_space.h"
#include "paths/path_reeds_shepp.h"

namespace Planner {
	class StateSpaceReedsShepp : public PlanarStateSpace {
	public:
		StateSpaceReedsShepp(std::array<Pose2d, 2> bounds = { Pose2d(-100, -100, -M_PI), Pose2d(100, 100, M_PI) },
			double minTurningRadius = 1.0) :
			PlanarStateSpace(bounds),
			minTurningRadius(minTurningRadius) { }
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
