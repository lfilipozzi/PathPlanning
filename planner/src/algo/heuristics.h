#pragma once

#include "core/base.h"
#include "algo/a_star.h"
#include "geometry/2dplane.h"
#include "utils/grid.h"

namespace Planner {
	/// @brief Store the path cost of a Reeds-Shepp path which is defined by the
	/// cost factors @reverseCostMultiplier, @forwardCostMultiplier, and 
	/// @directionSwitchingCost with a minimum turning radius @minTurningRadius.
	/// The path must be contained in a configuration space with bounds @bounds.
	class ReedsSheppPathCost {
	public:
		~ReedsSheppPathCost();

		static Ref<ReedsSheppPathCost> Build(const std::array<Pose2d, 2>& bounds,
			double spatialResolution, double angularResolution, double minTurningRadius,
			double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost);

		double GetPathCost(const Pose2d& from, const Pose2d& to);

	protected:
		ReedsSheppPathCost(double spatialResolution, double angularResolution, double minTurningRadius,
			double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost,
			unsigned int numSpatialX, unsigned int numSpatialY);

	public:
		const double spatialResolution, angularResolution, minTurningRadius;
		const double reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost;
		const unsigned int numSpatialX, numSpatialY, numAngular;
		const double offsetX, offsetY;

	private:
		// TODO use a double* instead of a double***
		double*** m_values;
	};

	/// @brief A* heuristic which evaluates the cost of a Reeds-Shepp path from 
	/// the current state to the goal.
	class NonHolonomicHeuristic : public AStarConcreteHeuristic<Pose2d> {
	public:
		NonHolonomicHeuristic(const Ref<ReedsSheppPathCost>& pathCostData);

		virtual double GetHeuristicValue(const Pose2d& state) override;

	private:
		Ref<ReedsSheppPathCost> m_pathCost;
	};

	/// @brief A* heuristic which evaluates the cost of a time-flipped 
	/// Reeds-Shepp path from the current state to the goal.
	class TimeFlippedNonHolonomicHeuristic : public AStarConcreteHeuristic<Pose2d> {
	public:
		TimeFlippedNonHolonomicHeuristic(const Ref<ReedsSheppPathCost>& pathCostData);

		virtual double GetHeuristicValue(const Pose2d& state) override;

	private:
		Ref<ReedsSheppPathCost> m_pathCost;
	};

	class OccupancyMap;

	/// @brief A* heuristic which evaluate the length of the path from the
	/// current state to the goal (multiplied by a cost predefined cost 
	/// multiplier) given an occupancy map.
	class ObstaclesHeuristic : public AStarConcreteHeuristic<Pose2d> {
	private:
		struct CompareCell {
			bool operator()(const GridCell<float>& lhs, const GridCell<float>& rhs) const
			{
				return lhs > rhs;
			}
		};

		struct HashCell {
			std::size_t operator()(const GridCell<float>& cell) const
			{
				return std::hash<Planner::GridCellPosition>()(cell.position);
			}
		};

		struct EqualCell {
			bool operator()(const GridCell<float>& lhs, const GridCell<float>& rhs) const
			{
				return lhs.position == rhs.position;
			}
		};

	public:
		ObstaclesHeuristic(const Ref<OccupancyMap>& map, double costMultiplier);

		void Update(const Pose2d& goal);

		virtual double GetHeuristicValue(const Pose2d& state) override;

		void Visualize(const std::string& filename) const;

	public:
		const float diagonalResolution;
		const float costMultiplier;

	private:
		static constexpr float s_discount = 0.92621f;
		Ref<OccupancyMap> m_map;
		Grid<float> m_cost;
		Grid<bool> m_explored;
	};
}
