#pragma once

#include "state_space/state_space.h"
#include "geometry/reeds_shepp.h"

namespace Planner {
	class StateSpaceReedsShepp : public StateSpace<Pose2D<>, 3> {
		using Pose = Pose2D<>;

	public:
		StateSpaceReedsShepp(std::array<std::array<double, 2>, 3> bounds = { { { -100, 100 }, { -100, 100 }, { -M_PI, M_PI } } }) :
			StateSpace<Pose, 3>(bounds) { }
		~StateSpaceReedsShepp() = default;

		/// @brief Compute the distance of a Reeds-Shepp path.
		double ComputeDistance(const ReedsShepp::PathSegment& path)
		{
			return path.GetLength(minTurningRadius);
		}
		/// @brief Compute the shortest distance from the pose @from to @to
		/// when driving Reeds-Shepp paths.
		virtual double ComputeDistance(const Pose& from, const Pose& to) override
		{
			return ReedsShepp::Solver::GetShortestDistance(from, to, minTurningRadius);
		}
		/// @brief Compute the shortest path between two poses
		Ref<PathReedsShepp> ComputeShortestPath(const Pose& from, const Pose& to)
		{
			auto pathSegment = ReedsShepp::Solver::GetShortestPath(from, to, minTurningRadius);
			return makeRef<PathReedsShepp>(from, pathSegment, minTurningRadius);
		}

		/// @brief Compute the cost of a Reeds-Shepp path.
		double ComputeCost(const ReedsShepp::PathSegment& path) const
		{
			return path.ComputeCost(minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
		}
		/// @brief Compute the cost associated the the most optimal Reeds-Shepp
		/// path.
		double ComputeCost(const Pose2D<>& from, const Pose2D<>& to) const
		{
			auto path = ReedsShepp::Solver::GetOptimalPath(from, to, minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
			return ComputeCost(path);
		}
		/// @brief Compute the optimal path between two poses
		Ref<PathReedsShepp> ComputeOptimalPath(const Pose& from, const Pose& to) const
		{
			auto pathSegment = ReedsShepp::Solver::GetOptimalPath(from, to, minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
			return makeRef<PathReedsShepp>(from, pathSegment, minTurningRadius);
		}

	public:
		double minTurningRadius = 1.0;
		double reverseCostMultiplier = 1.0, forwardCostMultiplier = 1.0;
		double directionSwitchingCost = 0.0;
	};
}
