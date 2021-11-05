#include "algo/hybrid_a_star.h"

#include "paths/path_constant_steer.h"
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
		m_model = makeRef<KinematicBicycleModel>();

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
		return {
			static_cast<int>(pose.x() / m_param.spatialResolution),
			static_cast<int>(pose.y() / m_param.spatialResolution),
			static_cast<int>(pose.WrapTheta() / m_param.angularResolution),
		};
	}

	HybridAStar::State HybridAStar::StatePropagator::CreateStateFromPath(const Ref<PlanarPath>& path) const
	{
		State state;
		state.discrete = DiscretizePose(path->GetFinalState());
		state.path = path;
		return state;
	}

	HybridAStar::State HybridAStar::StatePropagator::CreateStateFromPose(Pose2d pose) const
	{
		State state;
		state.discrete = DiscretizePose(pose);
		state.path = makeRef<PathConstantSteer>(m_model.get(), pose);
		return state;
	}

	std::vector<std::tuple<HybridAStar::State, double>> HybridAStar::StatePropagator::GetNeighborStates(const State& state)
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

	double HybridAStar::StatePropagator::GetTransitionCost(const PlanarPath& path) const
	{
		// Path cost
		double pathCost = path.ComputeCost(
			m_param.directionSwitchingCost, m_param.reverseCostMultiplier, m_param.forwardCostMultiplier);

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
		return pathCost + m_param.voronoiCostMultiplier * voronoiCost;
	}

	bool HybridAStar::StatePropagator::GetConstantSteerChild(const State& state, double delta, Direction direction, State& child, double& cost) const
	{
		auto path = makeRef<PathConstantSteer>(m_model.get(), state.GetPose(), delta, m_param.spatialResolution * 1.5, direction);
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

	bool HybridAStar::StatePropagator::GetReedsSheppChild(const State& state, State& child, double& cost) const
	{
		const auto& from = state.GetPose();
		const auto& to = m_goalState.GetPose();
		auto pathSegment = ReedsShepp::Solver::GetOptimalPath(from, to,
			m_param.minTurningRadius, m_param.reverseCostMultiplier,
			m_param.forwardCostMultiplier, m_param.directionSwitchingCost);
		auto path = makeRef<PathReedsShepp>(from, pathSegment, m_param.minTurningRadius);
		child = CreateStateFromPath(path);
		if (!m_validator->IsPathValid(*(child.path)))
			return false;

		// Compute transition cost
		cost = GetTransitionCost(*child.path);
		return true;
	}

	HybridAStar::HybridAStar(const Ref<StateValidatorOccupancyMap>& validator) :
		HybridAStar(validator, SearchParameters()) { }
	HybridAStar::HybridAStar(const Ref<StateValidatorOccupancyMap>& validator, const SearchParameters& p) :
		m_validator(validator)
	{
		m_propagator = makeRef<StatePropagator>(p);
	}

	bool HybridAStar::Initialize()
	{
		if (!m_validator || !m_validator->GetStateSpace()) {
			isInitialized = false;
			return false;
		}

		const auto& p = m_propagator->GetParameters();

		// Initialize Voronoi field and smoother
		m_gvd = makeRef<GVD>(m_validator->GetOccupancyMap());
		m_smoother = makeScope<Smoother>(m_validator, m_gvd, p.minTurningRadius);

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
		m_aStarSearch = makeScope<AStarDeclType>(m_propagator, heuristic);

		isInitialized = true;
		return true;
	}

	Status HybridAStar::SearchPath()
	{
		if (!isInitialized) {
			PP_ERROR("The algorithm has not been initialized successfully.");
			return Status::Failure;
		}

		// Update members
		m_obstacleHeuristic->Update(this->m_goal);
		m_gvd->Update();

		// Run A* on 2D pose
		State initState = m_propagator->CreateStateFromPose(this->m_init);
		State goalState = m_propagator->SetGoalState(this->m_goal);
		m_aStarSearch->SetInitState(initState);
		m_aStarSearch->SetGoalState(goalState);
		auto status = m_aStarSearch->SearchPath();

		// FIXME uncomment
// 		// Process path before smoothing
// 		auto aStarPath = m_aStarSearch->GetPath();
// 		std::vector<Smoother::State> nonSmoothPath;
// 		double totalPathLength = 0;
// 		for (auto& state : aStarPath)
// 			totalPathLength += state.path->GetLength();
// 		nonSmoothPath.reserve(totalPathLength / pathInterpolation);
// 		for (auto& state : aStarPath) {
// 			const auto& pathLength = state.path->GetLength();
// 			double length = 0.0;
// 			while (length < pathLength) {
// 				nonSmoothPath.push_back({ state.path->Interpolate(length / pathLength),
// 					state.path->GetDirection(length / pathLength) });
// 				length += pathInterpolation;
// 			}
// 		}
// 
// 		// Smooth the path
// 		auto smoothPath = m_smoother->Smooth(nonSmoothPath);

		// Save the result
// 		m_path.reserve(smoothPath.size());
// 		for (auto& state : smoothPath) {
// 			m_path.push_back(state.pose);
// 		}
		auto aStarPath = m_aStarSearch->GetPath();
		for (auto& state : aStarPath) {
			const auto& pathLength = state.path->GetLength();
			double length = 0.0;
			while (length < pathLength) {
				m_path.push_back(state.path->Interpolate(length / pathLength));
				length += pathInterpolation;
			}
		}

		return status;
	}

	void HybridAStar::VisualizeObstacleHeuristic(const std::string& filename) const
	{
		m_obstacleHeuristic->Visualize(filename);
	}

	std::unordered_set<Ref<PlanarPath>> HybridAStar::GetExploredPaths() const
	{
		const auto srcRootNode = m_aStarSearch->GetTree();
		std::unordered_set<Ref<PlanarPath>> exploredPaths;
		if (!srcRootNode)
			return exploredPaths;
		auto lambda = [&](const AStarDeclType::Node& node) {
			exploredPaths.insert(node.GetState().path);
		};
		srcRootNode->PreOrderTraversal(lambda);
		return exploredPaths;
	}
}
