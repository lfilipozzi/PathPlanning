#include "algo/hybrid_a_star.h"

#include "paths/path_constant_steer.h"
#include "state_validator/state_validator.h"
#include "algo/hybrid_a_star_heuristics.h"

namespace Planner {
	Pose2D<int> HybridAStar::StateSpace::DiscretizePose(const Pose2D<>& pose) const
	{
		return {
			static_cast<int>(pose.x / spatialResolution),
			static_cast<int>(pose.y / spatialResolution),
			static_cast<int>(pose.WrapTheta() / angularResolution),
		};
	}

	HybridAStar::State HybridAStar::StateSpace::CreateStateFromPath(const Ref<Path<Pose2D<>>>& path) const
	{
		State state;
		state.discrete = DiscretizePose(path->GetFinalState());
		state.path = path;
		return state;
	}

	HybridAStar::State HybridAStar::StateSpace::CreateStateFromPose(Pose2D<> pose) const
	{
		State state;
		state.discrete = DiscretizePose(pose);
		state.path = makeRef<PathConstantSteer>(pose);
		return state;
	}

	std::vector<std::tuple<HybridAStar::State, double>> HybridAStar::StateSpace::GetNeighborStates(const State& state)
	{
		// TODO compute delta from minTurning radius and numMotion
		std::vector<double> deltas = { -10, -7.5, -5, -2.5, -0, 2.5, 5, 7.5, 10 };

		std::vector<std::tuple<State, double>> neighbors;
		neighbors.reserve(2 * deltas.size());
		for (double delta : deltas) {
			State newState;
			double cost;

			// Find neighbor state in forward direction
			if (ConstantSteer(state, delta, Direction::Forward, newState, cost))
				neighbors.push_back({ newState, cost });

			// Find neighbor state in forward direction
			if (ConstantSteer(state, delta, Direction::Backward, newState, cost))
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

	double HybridAStar::StateSpace::GetTransitionCost(const Ref<Path<Pose2D<>>>& path) const
	{
		if (!path)
			return 0.0;

		// TODO use real cost, not euclidean distance
		auto delta = path->GetFinalState() - path->GetInitialState();
		double cost = sqrtf(powf(delta.x, 2) + powf(delta.y, 2));
		return cost;
	}

	bool HybridAStar::StateSpace::ConstantSteer(const State& state, double delta, Direction direction, State& next, double& cost) const
	{
		auto path = makeRef<PathConstantSteer>(state.GetPose(), delta, spatialResolution * 1.5, direction);
		next = CreateStateFromPath(path);

		// Validate transition
		float lastValidRatio;
		if (!m_validator->IsPathValid(*(next.path), &lastValidRatio)) {
			next.path->Truncate(lastValidRatio);
			next.discrete = DiscretizePose(next.path->GetFinalState());
			// Skip if the last valid state does not belong to a new cell
			if (state == next) {
				return false;
			}
		}

		// Compute transition cost
		cost = GetTransitionCost(next.path);
		return true;
	}

	bool HybridAStar::StateSpace::GetReedsSheppChild(const State& state, State& reedsShepp, double& cost) const
	{
		auto path = ComputeOptimalPath(state.GetPose(), m_goalState.GetPose());
		reedsShepp = CreateStateFromPath(path);
		if (!m_validator->IsPathValid(*(reedsShepp.path)))
			return false;

		// Compute transition cost
		cost = GetTransitionCost(reedsShepp.path);
		return true;
	}

	HybridAStar::HybridAStar(const Ref<StateSpace>& stateSpace, const Ref<StateValidator<Pose2D<>>>& stateValidator) :
		m_stateSpace(stateSpace), m_stateValidator(stateValidator)
	{
		m_nonHoloHeuristic = NonHolonomicHeuristic::Build(m_stateSpace);
		m_obstacleHeuristic = makeRef<ObstaclesHeuristic>();
		m_heuristic = makeRef<AStarCombinedHeuristic<State>>();
		m_heuristic->Add(m_nonHoloHeuristic, m_obstacleHeuristic);

		m_aStarSearch = makeScope<AStar<State, State::Hash>>(m_stateSpace, m_heuristic);
	}

	Status HybridAStar::SearchPath()
	{
		// Set the validator of the state-space
		if (!m_stateValidator || !m_heuristic)
			return Status::Failure;
		m_stateSpace->m_validator = m_stateValidator.get();
		m_stateSpace->m_heuristic = m_heuristic.get();

		// Run A* on 2D pose
		State initState = m_stateSpace->CreateStateFromPose(this->m_init);
		State goalState = m_stateSpace->SetGoalState(this->m_goal);
		m_aStarSearch->SetInitState(initState);
		m_aStarSearch->SetGoalState(goalState);
		auto status = m_aStarSearch->SearchPath();

		auto solutionStates = m_aStarSearch->GetPath();
		m_path.reserve(solutionStates.size());
		for (auto& state : solutionStates) {
			m_path.push_back(state.GetPose()); // TODO need to change type of m_path to std::vector<Path<Pose2D<>>>
		}

		// Smooth the path
		// TODO smooth path with variational method

		return status;
	}
}
