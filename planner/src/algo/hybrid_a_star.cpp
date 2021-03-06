#include "algo/hybrid_a_star.h"

#include "paths/path_constant_steer.h"
#include "paths/path_reeds_shepp.h"
#include "paths/path_composite.h"
#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/occupancy_map.h"
#include "state_validator/gvd.h"
#include "algo/heuristics.h"
#include "models/kinematic_bicycle_model.h"

namespace Planner {
	HybridAStar::StatePropagator::StatePropagator(const SearchParameters& parameters) :
		m_param(parameters)
	{
		// The heading of the point defining the trajectory of the vehicle must
		// be tangent to the trajectory (e.g. for front-steered vehicle, the
		// point must refer to the rear wheel)
		m_model = makeRef<KinematicBicycleModel>(parameters.wheelbase, 0.0);

		m_deltas.reserve(m_param.numGeneratedMotion);
		const double deltaMax = m_model->GetSteeringAngleFromTurningRadius(m_param.minTurningRadius);
		m_deltas.push_back(0.0);
		for (unsigned int i = 0; i < m_param.numGeneratedMotion / 2; i++) {
			double delta = (i + 1) / 2.0 * deltaMax;
			m_deltas.push_back(delta);
			m_deltas.push_back(-delta);
		}
	}

	void HybridAStar::StatePropagator::Initialize(const Ref<StateValidatorOccupancyMap>& stateValidator,
		const Ref<AStarHeuristic<State>>& heuristic, const Ref<GVD>& gvd)
	{
		m_validator = stateValidator;
		m_occupancyMap = m_validator->GetOccupancyMap();
		m_heuristic = heuristic;
		m_gvd = gvd;
		m_voroFieldDiagResolution = m_gvd->resolution * std::sqrt(2.0);
	}

	HybridAStar::State HybridAStar::StatePropagator::CreateStateFromPath(const Ref<PathNonHolonomicSE2Base>& path) const
	{
		State state;
		state.direction = path->GetDirection(1.0);
		state.continuousPose = path->GetFinalState();
		state.discretePose = DiscretizePose(state.continuousPose);
		return state;
	}

	HybridAStar::State HybridAStar::StatePropagator::CreateStateFromPose(const Pose2d& pose) const
	{
		State state;
		state.direction = Direction::NoMotion;
		state.continuousPose = pose;
		state.discretePose = DiscretizePose(state.continuousPose);
		return state;
	}

	std::vector<std::tuple<HybridAStar::State, HybridAStar::Action, double>> HybridAStar::StatePropagator::GetNeighborStates(const State& state)
	{
		PP_PROFILE_FUNCTION();

		std::vector<std::tuple<State, Action, double>> neighbors;
		neighbors.reserve(2 * m_deltas.size());
		for (double delta : m_deltas) {
			State newState;
			Action action;
			double cost;

			// Find neighbor state in forward direction
			if (GetConstantSteerChild(state, delta, Direction::Forward, newState, action, cost))
				neighbors.push_back({ newState, action, cost });

			// Find neighbor state in forward direction
			if (GetConstantSteerChild(state, delta, Direction::Backward, newState, action, cost))
				neighbors.push_back({ newState, action, cost });
		}

		// Randomly add a child using a Reeds-Shepp path with a
		// probability which is function of the heuristic to the goal
		double hCost = m_heuristic->GetHeuristicValue(state);
		if (hCost < 10.0 || Random<double>::SampleUniform(0.0, 1.0) < 10.0 / (hCost * hCost)) {
			State newState;
			Action action;
			double cost;
			if (GetReedsSheppChild(state, newState, action, cost))
				neighbors.push_back({ newState, action, cost });
		}

		return neighbors;
	}

	double HybridAStar::StatePropagator::GetVoronoiCost(const PathNonHolonomicSE2Base& path) const
	{
		PP_PROFILE_FUNCTION();

		float voronoiCost = 0.0;
		float interpLength = m_voroFieldDiagResolution;
		const double pathLength = path.GetLength();
		for (double length = 0.0; length < pathLength; length += interpLength) {
			auto position = path.Interpolate(length / pathLength).position;
			auto cell = m_occupancyMap->WorldPositionToGridCell(position, false);
			PP_ASSERT(m_occupancyMap->IsInsideMap(cell), "The cell is not inside the occupancy map");
			voronoiCost = m_gvd->GetPathCost(cell);
		}

		voronoiCost *= interpLength;
		return m_param.voronoiCostMultiplier * voronoiCost;
	}

	bool HybridAStar::StatePropagator::GetConstantSteerChild(const State& state, double delta, Direction direction, State& child, Action& action, double& cost) const
	{
		PP_PROFILE_FUNCTION();

		action.path = makeRef<PathConstantSteer>(m_model, state.continuousPose, delta, m_param.spatialResolution * 1.5, direction);
		child = CreateStateFromPath(action.path);

		// Validate transition
		float lastValidRatio;
		if (!m_validator->IsPathValid(*action.path, &lastValidRatio)) {
			action.path->Truncate(lastValidRatio);
			child.continuousPose = action.path->GetFinalState();
			child.discretePose = DiscretizePose(child.continuousPose);
			// Skip if the last valid state does not belong to a new cell
			if (State::Equal()(state, child)) {
				return false;
			}
		}

		// Compute transition cost
		double pathCost;
		switch (direction) {
		case Direction::Forward:
			pathCost = m_param.forwardCostMultiplier * action.path->GetLength();
			break;
		case Direction::Backward:
			pathCost = m_param.reverseCostMultiplier * action.path->GetLength();
			break;
		default:
			pathCost = 0.0;
		}
		double switchingCost = action.path->GetDirection(1.0) == action.path->GetDirection(1.0) ? 0.0 : m_param.directionSwitchingCost;
		double voronoiCost = GetVoronoiCost(*action.path);
		cost = pathCost + switchingCost + voronoiCost;

		return true;
	}

	bool HybridAStar::StatePropagator::GetReedsSheppChild(const State& state, State& child, Action& action, double& cost) const
	{
		PP_PROFILE_FUNCTION();

		const auto& from = state.continuousPose;
		const auto& to = m_goalState.continuousPose;
		auto pathSegment = ReedsShepp::Solver::GetOptimalPath(from, to,
			m_param.minTurningRadius, m_param.reverseCostMultiplier,
			m_param.forwardCostMultiplier, m_param.directionSwitchingCost);
		auto path = makeRef<PathReedsShepp>(from, pathSegment, m_param.minTurningRadius);

		if (!m_validator->IsPathValid(*path))
			return false;

		double pathAndSwitchingCosts = path->ComputeCost(m_param.directionSwitchingCost,
			m_param.reverseCostMultiplier, m_param.forwardCostMultiplier);
		child = CreateStateFromPath(path);
		action.path = std::move(path);

		// Compute transition cost
		double voronoiCost = GetVoronoiCost(*action.path);
		cost = pathAndSwitchingCosts + voronoiCost;

		return true;
	}

	Ref<PathSE2CompositeNonHolonomic> HybridAStar::GraphSearch::GetCompositePath() const
	{
		auto path = makeRef<PathSE2CompositeNonHolonomic>();
		const auto& actions = GetActions();
		path->Reserve(actions.size());
		for (auto& action : actions)
			path->PushBack(action.path);
		return path;
	}

	std::unordered_set<Ref<PathNonHolonomicSE2Base>> HybridAStar::GraphSearch::GetExploredPathSet() const
	{
		std::unordered_set<Ref<PathNonHolonomicSE2Base>> paths;

		if (!m_rootNode)
			return paths;

		m_rootNode->PreOrderTraversal([&](const Node& node) {
			if (node->action.path)
				paths.insert(node->action.path);
		});
		return paths;
	}

	HybridAStar::HybridAStar() :
		HybridAStar(SearchParameters()) { }
	HybridAStar::HybridAStar(const SearchParameters& p)
	{
		m_propagator = makeRef<StatePropagator>(p);
	}

	bool HybridAStar::Initialize(const Ref<StateValidatorOccupancyMap>& validator)
	{
		PP_PROFILE_FUNCTION();

		if (!validator || !validator->GetStateSpace())
			return isInitialized = false;

		m_validator = validator;
		const auto& p = m_propagator->GetParameters();

		// Initialize Voronoi field and smoother
		m_gvd = makeRef<GVD>(m_validator->GetOccupancyMap());
		m_smoother.Initialize(m_validator, m_gvd, 1.0 / p.minTurningRadius);

		// Initialize heuristic
		m_nonHoloHeuristic = NonHolonomicHeuristic::Build(m_validator->GetStateSpace()->bounds,
			p.spatialResolution, p.angularResolution, p.minTurningRadius,
			p.reverseCostMultiplier, p.forwardCostMultiplier, p.directionSwitchingCost);
		m_obstacleHeuristic = makeRef<ObstaclesHeuristic>(
			m_validator->GetOccupancyMap(), p.reverseCostMultiplier, p.forwardCostMultiplier);
		auto combinedHeur = makeRef<AStarCombinedHeuristic<Pose2d>>();
		combinedHeur->Add(m_nonHoloHeuristic, m_obstacleHeuristic);
		auto heuristic = makeRef<HeuristicAdapter>(combinedHeur);

		// Initialize A* search algorithm and its state propagator
		m_propagator->Initialize(m_validator, heuristic, m_gvd);
		m_graphSearch.Initialize(m_propagator, heuristic);

		return isInitialized = true;
	}

	Status HybridAStar::SearchPath()
	{
		PP_PROFILE_FUNCTION();

		// TODO create stats saving status of each step (graph search + optimization, number of iteration...)

		if (!isInitialized) {
			PP_ERROR("The algorithm has not been initialized successfully.");
			return Status::Failure;
		}

		// Update members
		m_obstacleHeuristic->Update(this->m_goal);
		m_gvd->Update();

		// Run A* on 2D pose
		m_graphSearch.SetInitState(m_propagator->CreateStateFromPose(this->m_init));
		m_graphSearch.SetGoalState(m_propagator->SetGoalState(this->m_goal));
		m_stats.graphSearchStatus = m_graphSearch.SearchPath();
		if (m_stats.graphSearchStatus < 0)
			return m_stats.graphSearchStatus;

		// Process path before smoothing
		std::vector<Pose2d> graphSearchPath;
		std::unordered_set<int> cuspIndices;
		{
			PP_PROFILE_SCOPE("Process graph search path");

			// Get composite path
			auto path = m_graphSearch.GetCompositePath();
			const double pathLength = path->GetLength();

			// Generate ratio at which to sample the path and find indices of cusp points
			std::vector<double> ratios;
			auto cuspRatios = path->GetCuspPointRatios();
			cuspRatios.insert(0.0);
			cuspRatios.insert(1.0);
			ratios.reserve(pathLength / pathInterpolation + cuspRatios.size());
			auto cuspRatioIt = cuspRatios.begin();
			for (double length = 0.0; length <= pathLength; length += pathInterpolation) {
				// Sample cusp point
				double cuspLength = *cuspRatioIt * pathLength;
				if (length >= cuspLength - pathInterpolation / 2 && length < cuspLength + pathInterpolation / 2) {
					cuspIndices.insert(ratios.size());
					ratios.push_back(*cuspRatioIt);
					cuspRatioIt++;
				} else {
					// Sample at regular interval
					ratios.push_back(length / pathLength);
				}
			}

			// Sample points
			graphSearchPath = path->Interpolate(ratios);
		}

		// Smooth the path
		m_stats.smoothingStatus = m_smoother.Smooth(graphSearchPath, cuspIndices);
		if (m_stats.smoothingStatus < 0) {
			m_path = graphSearchPath;
			return m_stats.graphSearchStatus;
		}

		// Save the result
		m_path = m_smoother.GetPath();

		return Status::Success;
	}

	void HybridAStar::VisualizeObstacleHeuristic(const std::string& filename) const
	{
		m_obstacleHeuristic->Visualize(filename);
	}

	std::unordered_set<Ref<PathNonHolonomicSE2Base>> HybridAStar::GetGraphSearchExploredPathSet() const
	{
		return m_graphSearch.GetExploredPathSet();
	}

	std::vector<Ref<PathNonHolonomicSE2Base>> HybridAStar::GetGraphSearchPath() const
	{
		const auto& actions = m_graphSearch.GetActions();
		std::vector<Ref<PathNonHolonomicSE2Base>> paths;
		for (auto& action : actions)
			paths.push_back(action.path);
		return paths;
	}

	double HybridAStar::GetGraphSearchOptimalCost() const
	{
		return m_graphSearch.GetOptimalCost();
	}
}
