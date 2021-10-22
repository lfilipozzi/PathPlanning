#include "algo/hybrid_a_star.h"

#include "paths/path_constant_steer.h"
#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/occupancy_map.h"
#include "state_validator/gvd.h"
#include "algo/hybrid_a_star_heuristics.h"
#include "models/kinematic_bicycle_model.h"

namespace Planner {
	HybridAStar::StateSpace::StateSpace(double spatialResolution, double angularResolution,
		unsigned int numGeneratedMotion,
		double minTurningRadius, double directionSwitchingCost,
		double reverseCostMultiplier, double forwardCostMultiplier,
		double voronoiCostMultiplier,
		const std::array<Pose2d, 2>& bounds) :
		StateSpaceReedsShepp(minTurningRadius, directionSwitchingCost, reverseCostMultiplier, forwardCostMultiplier, bounds),
		voronoiCostMultiplier(voronoiCostMultiplier),
		spatialResolution(spatialResolution), angularResolution(angularResolution), numGeneratedMotion(numGeneratedMotion)
	{
		m_model = makeRef<KinematicBicycleModel>();

		m_deltas.reserve(numGeneratedMotion);
		const double deltaMax = m_model->GetSteeringAngleFromTurningRadius(minTurningRadius);
		m_deltas.push_back(0.0);
		for (unsigned int i = 0; i < numGeneratedMotion / 2; i++) {
			double delta = (i + 1) / 2.0 * deltaMax;
			m_deltas.push_back(delta);
			m_deltas.push_back(-delta);
		}
	}

	Pose2i HybridAStar::StateSpace::DiscretizePose(const Pose2d& pose) const
	{
		return {
			static_cast<int>(pose.x() / spatialResolution),
			static_cast<int>(pose.y() / spatialResolution),
			static_cast<int>(pose.WrapTheta() / angularResolution),
		};
	}

	HybridAStar::State HybridAStar::StateSpace::CreateStateFromPath(const Ref<Path<Pose2d>>& path) const
	{
		State state;
		state.discrete = DiscretizePose(path->GetFinalState());
		state.path = path;
		return state;
	}

	HybridAStar::State HybridAStar::StateSpace::CreateStateFromPose(Pose2d pose) const
	{
		State state;
		state.discrete = DiscretizePose(pose);
		state.path = makeRef<PathConstantSteer>(m_model.get(), pose);
		return state;
	}

	std::vector<std::tuple<HybridAStar::State, double>> HybridAStar::StateSpace::GetNeighborStates(const State& state)
	{
		std::vector<std::tuple<State, double>> neighbors;
		neighbors.reserve(2 * m_deltas.size());
		for (double delta : m_deltas) {
			State newState;
			double cost;

			// Find neighbor state in forward direction
			if (GetConstantSteerChild(state, delta, Direction::Forward, newState, cost))
				neighbors.push_back({ newState, cost });

			// Find neighbor state in forward direction
			if (GetConstantSteerChild(state, delta, Direction::Backward, newState, cost))
				neighbors.push_back({ newState, cost });
		}

		// Randomly add a child using a Reeds-Shepp path with a
		// probability which is function of the heuristic to the goal
		double cost = m_heuristic->GetHeuristicValue(state, m_goalState);
		if (cost < 10.0 || Random<double>::SampleUniform(0.0, 1.0) < 10.0 / (cost * cost)) {
			State reedsShepp;
			if (GetReedsSheppChild(state, reedsShepp, cost))
				neighbors.push_back({ reedsShepp, cost });
		}

		return neighbors;
	}

	double HybridAStar::StateSpace::GetTransitionCost(const Path<Pose2d>& path) const
	{
		// Path cost
		double pathCost = path.ComputeCost(directionSwitchingCost, reverseCostMultiplier, forwardCostMultiplier);

		// Voronoi cost
		float voronoiCost = 0.0;
		bool inside = true;
		float interpLength = 0.1;
		{
			const double pathLength = path.GetLength();
			double length = 0.0;
			float positionCost;
			while (length < pathLength) {
				// Get cost
				if (m_gvd->GetPathCost(path.Interpolate(length / pathLength).position, positionCost)) {
					voronoiCost += positionCost;
				} else {
					inside = false;
				}
				// Update length
				length += interpLength;
			}

			voronoiCost *= interpLength;
			if (!inside) {
				PP_WARN("Requesting path cost outside of the map");
			}
		}
		return pathCost + voronoiCostMultiplier * voronoiCost;
	}

	bool HybridAStar::StateSpace::GetConstantSteerChild(const State& state, double delta, Direction direction, State& child, double& cost) const
	{
		auto path = makeRef<PathConstantSteer>(m_model.get(), state.GetPose(), delta, spatialResolution * 1.5, direction);
		child = CreateStateFromPath(path);

		// Validate transition
		float lastValidRatio;
		if (!m_validator->IsPathValid(*(child.path), &lastValidRatio)) {
			child.path->Truncate(lastValidRatio);
			child.discrete = DiscretizePose(child.path->GetFinalState());
			// Skip if the last valid state does not belong to a new cell
			if (state == child) {
				return false;
			}
		}

		// Compute transition cost
		cost = GetTransitionCost(*child.path);
		return true;
	}

	bool HybridAStar::StateSpace::GetReedsSheppChild(const State& state, State& child, double& cost) const
	{
		auto path = ComputeOptimalPath(state.GetPose(), m_goalState.GetPose());
		child = CreateStateFromPath(path);
		if (!m_validator->IsPathValid(*(child.path)))
			return false;

		// Compute transition cost
		cost = GetTransitionCost(*child.path);
		return true;
	}

	HybridAStar::HybridAStar(const Ref<StateSpace>& stateSpace, const Ref<StateValidatorOccupancyMap>& stateValidator) :
		m_stateSpace(stateSpace), m_stateValidator(stateValidator)
	{
	}

	bool HybridAStar::Initialize()
	{
		if (!m_stateSpace || !m_stateValidator) {
			isInitialized = false;
			return false;
		}

		// Initialize state-space
		m_gvd = makeScope<GVD>(m_stateValidator->GetOccupancyMap());;

		// Initialize heuristic
		m_nonHoloHeuristic = NonHolonomicHeuristic::Build(m_stateSpace);
		m_obstacleHeuristic = makeRef<ObstaclesHeuristic>(m_stateValidator->GetOccupancyMap());
		m_heuristic = makeRef<AStarCombinedHeuristic<State>>();
		m_heuristic->Add(m_nonHoloHeuristic, m_obstacleHeuristic);

		// A* search algorithm
		m_aStarSearch = makeScope<AStar<State, State::Hash, State::Equal>>(m_stateSpace, m_heuristic);

		isInitialized = true;
		return true;
	}

	Status HybridAStar::SearchPath()
	{
		if (!isInitialized) {
			PP_ERROR("The algorithm has not been initialized successfully.");
			return Status::Failure;
		}

		// Set members of hybrid A* state-space
		m_stateSpace->m_validator = m_stateValidator.get();
		m_stateSpace->m_heuristic = m_heuristic.get();
		m_stateSpace->m_gvd = m_gvd.get();

		// Update members
		m_obstacleHeuristic->Update(this->m_goal);
		m_gvd->Update();

		// Run A* on 2D pose
		State initState = m_stateSpace->CreateStateFromPose(this->m_init);
		State goalState = m_stateSpace->SetGoalState(this->m_goal);
		m_aStarSearch->SetInitState(initState);
		m_aStarSearch->SetGoalState(goalState);
		auto status = m_aStarSearch->SearchPath();

		auto solutionStates = m_aStarSearch->GetPath();
		m_path.reserve(solutionStates.size());
		for (auto& state : solutionStates) {
			m_path.push_back(state.GetPose()); // TODO need to change type of m_path to std::vector<Path<Pose2d>>
		}

		// Smooth the path
		// TODO smooth path with variational method

		return status;
	}
}
