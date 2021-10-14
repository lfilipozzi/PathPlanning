#pragma once

#include "state_space/state_space.h"
#include "paths/path_reeds_shepp.h"

namespace Planner {
	class StateSpaceReedsShepp : public StateSpace<Pose2D<>, 3> {
	public:
		StateSpaceReedsShepp(double minTurningRadius = 1.0, double directionSwitchingCost = 0.0,
			double reverseCostMultiplier = 1.0, double forwardCostMultiplier = 1.0,
			std::array<std::array<double, 2>, 3> bounds = { { { -100, 100 }, { -100, 100 }, { -M_PI, M_PI } } }) :
			StateSpace<Pose2D<>, 3>(bounds),
			minTurningRadius(minTurningRadius), directionSwitchingCost(directionSwitchingCost),
			reverseCostMultiplier(reverseCostMultiplier), forwardCostMultiplier(forwardCostMultiplier) { }
		~StateSpaceReedsShepp() = default;

		/// @brief Compute the shortest distance from the pose @from to @to
		/// when driving Reeds-Shepp paths.
		virtual double ComputeDistance(const Pose2D<>& from, const Pose2D<>& to) override
		{
			return ReedsShepp::Solver::GetShortestDistance(from, to, minTurningRadius);
		}
		/// @brief Compute the shortest path between two poses
		Ref<PathReedsShepp> ComputeShortestPath(const Pose2D<>& from, const Pose2D<>& to)
		{
			auto pathSegment = ReedsShepp::Solver::GetShortestPath(from, to, minTurningRadius);
			return makeRef<PathReedsShepp>(from, pathSegment, minTurningRadius);
		}

		/// @brief Compute the cost associated the the most optimal Reeds-Shepp
		/// path.
		double ComputeCost(const Pose2D<>& from, const Pose2D<>& to) const
		{
			auto path = ReedsShepp::Solver::GetOptimalPath(from, to, minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
			return path.ComputeCost(minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
		}
		/// @brief Compute the optimal path between two poses
		Ref<PathReedsShepp> ComputeOptimalPath(const Pose2D<>& from, const Pose2D<>& to) const
		{
			auto pathSegment = ReedsShepp::Solver::GetOptimalPath(from, to, minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
			return makeRef<PathReedsShepp>(from, pathSegment, minTurningRadius);
		}

	public:
		const double minTurningRadius;
		const double directionSwitchingCost, reverseCostMultiplier, forwardCostMultiplier;
	};
}
