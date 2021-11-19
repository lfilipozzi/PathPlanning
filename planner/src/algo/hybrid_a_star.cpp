#include "algo/hybrid_a_star.h"

#include "paths/path_constant_steer.h"
#include "paths/path_composite.h"
#include "state_space/state_space_reeds_shepp.h"
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
		m_heuristic = heuristic;
		m_gvd = gvd;
	}

	Pose2i HybridAStar::StatePropagator::DiscretizePose(const Pose2d& pose) const
	{
		PP_PROFILE_FUNCTION();

		return {
			static_cast<int>(pose.x() / m_param.spatialResolution),
			static_cast<int>(pose.y() / m_param.spatialResolution),
			static_cast<int>(pose.WrapTheta() / m_param.angularResolution),
		};
	}

	HybridAStar::State HybridAStar::StatePropagator::CreateStateFromPath(const Ref<PlanarNonHolonomicPath>& path) const
	{
		PP_PROFILE_FUNCTION();

		State state;
		state.discrete = DiscretizePose(path->GetFinalState());
		state.path = path;
		return state;
	}

	HybridAStar::State HybridAStar::StatePropagator::CreateStateFromPose(Pose2d pose) const
	{
		PP_PROFILE_FUNCTION();

		State state;
		state.discrete = DiscretizePose(pose);
		state.path = makeRef<PathConstantSteer>(m_model, pose);
		return state;
	}

	std::vector<std::tuple<HybridAStar::State, double>> HybridAStar::StatePropagator::GetNeighborStates(const State& state)
	{
		PP_PROFILE_FUNCTION();

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
		double hCost = m_heuristic->GetHeuristicValue(state);
		if (hCost < 10.0 || Random<double>::SampleUniform(0.0, 1.0) < 10.0 / (hCost * hCost)) {
			State reedsShepp;
			double cost;
			if (GetReedsSheppChild(state, reedsShepp, cost))
				neighbors.push_back({ reedsShepp, cost });
		}

		return neighbors;
	}

	double HybridAStar::StatePropagator::GetDirectionSwitchingCost(const PlanarNonHolonomicPath& parentPath, const PlanarNonHolonomicPath& childPath) const
	{
		double switchingCost = 0.0;
		if (parentPath.GetDirection(1.0) != childPath.GetDirection(0.0))
			switchingCost = m_param.directionSwitchingCost;
		return switchingCost;
	}

	double HybridAStar::StatePropagator::GetVoronoiCost(const PlanarNonHolonomicPath& path) const
	{
		PP_PROFILE_FUNCTION();

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
		return m_param.voronoiCostMultiplier * voronoiCost;
	}

	bool HybridAStar::StatePropagator::GetConstantSteerChild(const State& state, double delta, Direction direction, State& child, double& cost) const
	{
		PP_PROFILE_FUNCTION();

		auto path = makeRef<PathConstantSteer>(m_model, state.GetPose(), delta, m_param.spatialResolution * 1.5, direction);
		child = CreateStateFromPath(path);

		// Validate transition
		float lastValidRatio;
		if (!m_validator->IsPathValid(*child.path, &lastValidRatio)) {
			child.path->Truncate(lastValidRatio);
			child.discrete = DiscretizePose(child.path->GetFinalState());
			// Skip if the last valid state does not belong to a new cell
			if (State::Equal()(state, child)) {
				return false;
			}
		}

		// Compute transition cost
		double pathCost;
		switch (path->GetDirection(0.0)) {
		case Direction::Forward:
			pathCost = m_param.forwardCostMultiplier * path->GetLength();
			break;
		case Direction::Backward:
			pathCost = m_param.reverseCostMultiplier * path->GetLength();
			break;
		default:
			pathCost = 0.0;
		}
		double switchingCost = GetDirectionSwitchingCost(*state.path, *child.path);
		double voronoiCost = GetVoronoiCost(*child.path);
		cost = pathCost + switchingCost + voronoiCost;
		return true;
	}

	bool HybridAStar::StatePropagator::GetReedsSheppChild(const State& state, State& child, double& cost) const
	{
		PP_PROFILE_FUNCTION();

		const auto& from = state.GetPose();
		const auto& to = m_goalState.GetPose();
		auto pathSegment = ReedsShepp::Solver::GetOptimalPath(from, to,
			m_param.minTurningRadius, m_param.reverseCostMultiplier,
			m_param.forwardCostMultiplier, m_param.directionSwitchingCost);
		auto path = makeRef<PathReedsShepp>(from, pathSegment, m_param.minTurningRadius);
		child = CreateStateFromPath(path);
		if (!m_validator->IsPathValid(*child.path))
			return false;

		// Compute transition cost
		double pathCost = path->ComputeCost(m_param.directionSwitchingCost,
			m_param.reverseCostMultiplier, m_param.forwardCostMultiplier);
		double switchingCost = GetDirectionSwitchingCost(*state.path, *child.path);
		double voronoiCost = GetVoronoiCost(*child.path);
		cost = pathCost + switchingCost + voronoiCost;
		return true;
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
		auto graphSearchStatus = m_graphSearch.SearchPath();
		if (graphSearchStatus < 0)
			return graphSearchStatus;

		// Process path before smoothing
		std::vector<Pose2d> graphSearchPath;
		std::unordered_set<int> cuspIndices;
		{
			PP_PROFILE_SCOPE("Process graph search path");

			// Generate composite path
			auto path = makeRef<PlanarNonHolonomicCompositePath>();
			const auto& states = m_graphSearch.GetPath();
			path->Reserve(states.size());
			for (auto& state : states)
				path->PushBack(state.path);
			double pathLength = path->GetLength();

			// Generate ratio at which to sample the path and find indices of cusp points
			std::vector<double> ratios;
			auto cuspRatios = path->GetCuspPointRatios();
			cuspRatios.insert(0.0);
			cuspRatios.insert(1.0);
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
		auto smoothStatus = m_smoother.Smooth(graphSearchPath, cuspIndices);
		if (smoothStatus < 0) {
			m_path = graphSearchPath;
			return graphSearchStatus;
		}

		// Save the result
		m_path = m_smoother.GetPath();

		return Status::Success;
	}

	void HybridAStar::VisualizeObstacleHeuristic(const std::string& filename) const
	{
		m_obstacleHeuristic->Visualize(filename);
	}

	std::unordered_set<Ref<PlanarNonHolonomicPath>> HybridAStar::GetGraphSearchExploredSet() const
	{
		const auto& exploredStates = m_graphSearch.GetExploredStates();
		std::unordered_set<Ref<PlanarNonHolonomicPath>> exploredPaths;
		for (auto& state : exploredStates)
			exploredPaths.insert(state.path);
		return exploredPaths;
	}

	std::vector<Ref<PlanarNonHolonomicPath>> HybridAStar::GetGraphSearchPath() const
	{
		const auto& graphSearchPath = m_graphSearch.GetPath();
		std::vector<Ref<PlanarNonHolonomicPath>> paths;
		for (auto& state : graphSearchPath)
			paths.push_back(state.path);
		return paths;
	}

	double HybridAStar::GetGraphSearchOptimalCost() const
	{
		return m_graphSearch.GetOptimalCost();
	}
}
