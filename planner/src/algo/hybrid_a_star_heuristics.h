#pragma once

#include "algo/a_star.h"
#include "algo/hybrid_a_star.h"

namespace Planner {
	class NonHolonomicHeuristic : public AStarHeuristic<HybridAStar::State> {
	public:
		// FIXME Should not be public, look at https://stackoverflow.com/questions/8147027/how-do-i-call-stdmake-shared-on-a-class-with-only-protected-or-private-const
		NonHolonomicHeuristic(double spatialResolution, double angularResolution, double minTurningRadius,
			double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost,
			unsigned int numSpatial);
		~NonHolonomicHeuristic();

		/// @brief Generate the NonHolonomicHeuristic heuristic.
		static Ref<NonHolonomicHeuristic> Build(const Ref<HybridAStar::StateSpace>& stateSpace, double spatialSize);

		virtual double GetHeuristicValue(const HybridAStar::State& from, const HybridAStar::State& to) override;

	public:
		const double spatialResolution, angularResolution, minTurningRadius;
		const double reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost;
		const unsigned int numSpatial, numAngular;
		const double offset;

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
