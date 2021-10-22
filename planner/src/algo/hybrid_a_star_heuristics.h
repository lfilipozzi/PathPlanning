#pragma once

#include "core/base.h"
#include "algo/a_star.h"
#include "algo/hybrid_a_star.h"
#include "geometry/2dplane.h"
#include "utils/grid.h"

namespace Planner {
	class NonHolonomicHeuristic : public AStarHeuristic<HybridAStar::State> {
	public:
		~NonHolonomicHeuristic();

		/// @brief Generate the NonHolonomicHeuristic heuristic.
		static Ref<NonHolonomicHeuristic> Build(const Ref<HybridAStar::StateSpace>& stateSpace);

		virtual double GetHeuristicValue(const HybridAStar::State& from, const HybridAStar::State& to) override;

	protected:
		NonHolonomicHeuristic(double spatialResolution, double angularResolution, double minTurningRadius,
			double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost,
			unsigned int numSpatialX, unsigned int numSpatialY);

	public:
		const double spatialResolution, angularResolution, minTurningRadius;
		const double reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost;
		const unsigned int numSpatialX, numSpatialY, numAngular;
		const double offsetX, offsetY;

	private:
		double*** m_values;
	};

	class OccupancyMap;

	class ObstaclesHeuristic : public AStarHeuristic<HybridAStar::State> {
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
		ObstaclesHeuristic(const Ref<OccupancyMap>& map, double reverseCostMultiplier, double forwardCostMultiplier);

		void Update(const Pose2d& goal);

		virtual double GetHeuristicValue(const HybridAStar::State& from, const HybridAStar::State& to) override;

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
