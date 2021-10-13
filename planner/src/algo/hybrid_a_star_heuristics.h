#pragma once

#include "algo/a_star.h"
#include "algo/hybrid_a_star.h"

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

	class ObstaclesHeuristic : public AStarHeuristic<HybridAStar::State> {
	public:
		ObstaclesHeuristic() = default;

		virtual double GetHeuristicValue(const HybridAStar::State& from, const HybridAStar::State& to) override;

	private:
	};
}
