#include "algo/hybrid_a_star_heuristics.h"

namespace Planner {
	NonHolonomicHeuristic::NonHolonomicHeuristic(double spatialResolution, double angularResolution, double minTurningRadius,
		double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost,
		unsigned int numSpatialX, unsigned int numSpatialY) :
		spatialResolution(spatialResolution),
		angularResolution(angularResolution), minTurningRadius(minTurningRadius),
		reverseCostMultiplier(reverseCostMultiplier), forwardCostMultiplier(forwardCostMultiplier), directionSwitchingCost(directionSwitchingCost),
		numSpatialX(numSpatialX), numSpatialY(numSpatialY), numAngular(ceil(2 * M_PI / angularResolution)),
		offsetX(floor(numSpatialX / 2.0) * spatialResolution), offsetY(floor(numSpatialY / 2.0) * spatialResolution)
	{
		m_values = new double**[numSpatialX];
		for (unsigned int i = 0; i < numSpatialX; ++i) {
			m_values[i] = new double*[numSpatialY];
			for (unsigned int j = 0; j < numSpatialY; j++) {
				m_values[i][j] = new double[numAngular];
			}
		}
	}

	NonHolonomicHeuristic::~NonHolonomicHeuristic()
	{
		for (unsigned int i = 0; i < numSpatialX; i++) {
			for (unsigned int j = 0; j < numSpatialY; j++) {
				delete[] m_values[i][j];
			}
			delete[] m_values[i];
		}
		delete[] m_values;
	}

	// Empty structure to allow to create a Ref<NonHolonomicHeuristic>
	struct MakeRefEnabler : public NonHolonomicHeuristic {
		template <typename... Args>
		MakeRefEnabler(Args&&... args) :
			NonHolonomicHeuristic(std::forward<Args>(args)...) { }
	};

	Ref<NonHolonomicHeuristic> NonHolonomicHeuristic::Build(const Ref<HybridAStar::StateSpace>& stateSpace)
	{
		PP_INFO("Generating non-holonomic heuristic without obstacle.");

		const auto& bounds = stateSpace->GetBounds();
		const double& spatialResolution = stateSpace->spatialResolution;
		const double& angularResolution = stateSpace->angularResolution;
		const double& minTurningRadius = stateSpace->minTurningRadius;
		const double& reverseCostMultiplier = stateSpace->reverseCostMultiplier;
		const double& forwardCostMultiplier = stateSpace->forwardCostMultiplier;
		const double& directionSwitchingCost = stateSpace->directionSwitchingCost;
		const double spatialSizeX = bounds[0][1] - bounds[0][0];
		const double spatialSizeY = bounds[1][1] - bounds[1][0];

		unsigned int numSpatialGuessX = ceil(spatialSizeX / spatialResolution);
		if (numSpatialGuessX % 2 == 0)
			numSpatialGuessX++;
		unsigned int numSpatialGuessY = ceil(spatialSizeY / spatialResolution);
		if (numSpatialGuessY % 2 == 0)
			numSpatialGuessY++;

		Ref<NonHolonomicHeuristic> heuristic = makeRef<MakeRefEnabler>(
			spatialResolution, angularResolution, minTurningRadius,
			reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost,
			numSpatialGuessX, numSpatialGuessY);
		double***& values = heuristic->m_values;
		const double& offsetX = heuristic->offsetX;
		const double& offsetY = heuristic->offsetY;
		const unsigned int& numSpatialX = heuristic->numSpatialX;
		const unsigned int& numSpatialY = heuristic->numSpatialY;
		const unsigned int& numAngular = heuristic->numAngular;

		const Pose2D<> goal(0.0, 0.0, 0.0);
		for (int i = 0; i < numSpatialX; i++) {
			for (int j = 0; j < numSpatialY; j++) {
				for (int k = 0; k < numAngular; k++) {
					Pose2D<> pose(i * spatialResolution - offsetX, j * spatialResolution - offsetY, k * angularResolution);
					auto path = ReedsShepp::Solver::GetOptimalPath(pose, goal, minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
					values[i][j][k] = path.ComputeCost(minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
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

		int i = (int)round((delta.x + offsetX) / spatialResolution);
		int j = (int)round((delta.y + offsetY) / spatialResolution);
		int k = (int)round(delta.theta / angularResolution);
		if (k == numAngular)
			k = 0;

		if (i < 0 || i >= numSpatialX || j < 0 || j >= numSpatialY) {
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
