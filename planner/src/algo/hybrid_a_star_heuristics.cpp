#include "algo/hybrid_a_star_heuristics.h"

namespace Planner {
	NonHolonomicHeuristic::NonHolonomicHeuristic(double spatialResolution, double angularResolution, double minTurningRadius,
		double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost,
		unsigned int numSpatial) :
		spatialResolution(spatialResolution),
		angularResolution(angularResolution), minTurningRadius(minTurningRadius),
		reverseCostMultiplier(reverseCostMultiplier), forwardCostMultiplier(forwardCostMultiplier), directionSwitchingCost(directionSwitchingCost),
		numSpatial(numSpatial), numAngular(ceil(2 * M_PI / angularResolution)), offset(floor(numSpatial / 2.0) * spatialResolution)
	{
		m_values = new double**[numSpatial];
		for (unsigned int i = 0; i < numSpatial; ++i) {
			m_values[i] = new double*[numSpatial];
			for (unsigned int j = 0; j < numSpatial; j++) {
				m_values[i][j] = new double[numAngular];
			}
		}
	}
	NonHolonomicHeuristic::~NonHolonomicHeuristic()
	{
		for (unsigned int i = 0; i < numSpatial; i++) {
			for (unsigned int j = 0; j < numSpatial; j++) {
				delete[] m_values[i][j];
			}
			delete[] m_values[i];
		}
		delete[] m_values;
	}

	Ref<NonHolonomicHeuristic> NonHolonomicHeuristic::Build(const Ref<HybridAStar::StateSpace>& stateSpace, double spatialSize)
	{
		PP_INFO("Generating non-holonomic heuristic without obstacle.");

		const double& spatialResolution = stateSpace->spatialResolution;
		const double& angularResolution = stateSpace->angularResolution;
		const double& minTurningRadius = stateSpace->minTurningRadius;
		const double& reverseCostMultiplier = stateSpace->reverseCostMultiplier;
		const double& forwardCostMultiplier = stateSpace->forwardCostMultiplier;
		const double& directionSwitchingCost = stateSpace->directionSwitchingCost;

		unsigned int numSpatialGuess = ceil(spatialSize / stateSpace->spatialResolution);
		if (numSpatialGuess % 2 == 0)
			numSpatialGuess++;

		auto heuristic = makeRef<NonHolonomicHeuristic>(
			spatialResolution, angularResolution, minTurningRadius,
			reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost,
			numSpatialGuess);
		double***& values = heuristic->m_values;
		const double& offset = heuristic->offset;
		const unsigned int& numSpatial = heuristic->numSpatial;
		const unsigned int& numAngular = heuristic->numAngular;

		Pose2D<> goal(0.0, 0.0, 0.0);
		for (int i = 0; i < numSpatial; i++) {
			for (int j = 0; j < numSpatial; j++) {
				for (int k = 0; k < numAngular; k++) {
					Pose2D<> pose(i * spatialResolution - offset, j * spatialResolution - offset, k * angularResolution);
					values[i][j][k] = stateSpace->ComputeCost(pose, goal);
				}
			}
		}

		PP_INFO("Non-holonomic heuristic without obstacle generated.");
		return heuristic;
	}

	double NonHolonomicHeuristic::GetHeuristicValue(const HybridAStar::State& from, const HybridAStar::State& to)
	{
		auto delta = to.GetPose() - from.GetPose();
		delta.theta = delta.WrapTheta();

		int i = (int)round((delta.x + offset) / spatialResolution);
		int j = (int)round((delta.y + offset) / spatialResolution);
		int k = (int)round(delta.theta / angularResolution);
		if (k == numAngular)
			k = 0;

		if (i < 0 || i >= numSpatial || j < 0 || j >= numSpatial) {
			double euclideanDistance = sqrtf(powf(delta.x, 2) + powf(delta.y, 2));
			double distanceMultiplier = std::min(reverseCostMultiplier, forwardCostMultiplier);
			return distanceMultiplier * euclideanDistance;
		}

		return m_values[i][j][k];
	}

	double ObstaclesHeuristic::GetHeuristicValue(const HybridAStar::State& /*from*/, const HybridAStar::State& /*to*/)
	{
		// TODO use flow-fields algorithm
		return 0.0;
	}
}
