#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "core/base.h"
#include "algo/path_planner.h"
#include "algo/a_star.h"
#include "algo/bidirectional_a_star.h"
#include "algo/smoother.h"
#include "algo/heuristics.h"
#include "geometry/2dplane.h"
#include "paths/path_composite.h"
#include "state_validator/state_validator_occupancy_map.h"

// FIXME replace nested class by namespace

namespace Planner {

	class StateValidatorOccupancyMap;
	class KinematicBicycleModel;
	class GVD;

	/// @brief Conduct a Hybrid A* search to find the optimal path.
	/// @details The continuous state-space is discretized to perform a A*
	/// search. Two poses are considered equal if they belong to the same cell in
	/// the configuration space.
	/// The A* search provides an optimal solution that is then smoothed by a
	/// nonlinear optimization.
	class HybridAStar : public PathPlannerSE2Base {
	public:
		struct SearchParameters {
			double wheelbase = 2.6; // TODO add maxSteeringAngle to choose how to propagate bicycle model
			double minTurningRadius = 2.0;
			double directionSwitchingCost = 0.0;
			double reverseCostMultiplier = 1.0;
			double forwardCostMultiplier = 1.0;
			double voronoiCostMultiplier = 1.0;
			unsigned int numGeneratedMotion = 5;
			double spatialResolution = 1.0;
			double angularResolution = 0.0872;

			SearchParameters() = default;
			SearchParameters(double minTurningRadius, double directionSwitchingCost,
				double reverseCostMultiplier, double forwardCostMultiplier,
				double voronoiCostMultiplier, unsigned int numGeneratedMotion,
				double spatialResolution, double angularResolution) :
				minTurningRadius(minTurningRadius),
				directionSwitchingCost(directionSwitchingCost),
				reverseCostMultiplier(reverseCostMultiplier), forwardCostMultiplier(forwardCostMultiplier),
				voronoiCostMultiplier(voronoiCostMultiplier), numGeneratedMotion(numGeneratedMotion),
				spatialResolution(spatialResolution), angularResolution(angularResolution) { }
		};

		struct Stats {
			Status graphSearchStatus = Status::Failure;
			Smoother::Status smoothingStatus = Smoother::Status::Failure;
		};

		/// @brief State used for the A* search in the hybrid A* algorithm.
		struct State {
			/// @brief Direction
			Direction direction;
			/// @brief Discretized state.
			Pose2i discretePose;
			/// @brief Pose
			Pose2d continuousPose;

			/// @brief Hasher for the discretized configuration space.
			/// @details Two states are considered equal if they correspond to the
			/// same"cell", i.e. they have their position difference is less than the
			/// spatial resolution and if their heading difference is less than the
			/// angular resolution.
			struct Hash {
				std::size_t operator()(const State& state) const
				{
					std::hash<Pose2i> hasher;
					return hasher(state.discretePose);
				}
			};

			/// @brief Check if two states correspond to the same cell.
			struct Equal {
				bool operator()(const State& lhs, const State& rhs) const
				{
					return lhs.discretePose == rhs.discretePose;
				}
			};
		};

		/// @brief Action between states in hybrid A*
		struct Action {
			/// @brief Path connecting the previous state to the current state.
			Ref<PathNonHolonomicSE2Base> path;
		};

		/// @brief Define the state-space used for the A* search.
		class StatePropagator : public AStarStatePropagator<State, Action> {
		public:
			StatePropagator(const SearchParameters& parameters);

			void Initialize(const Ref<StateValidatorOccupancyMap>& stateValidator,
				const Ref<AStarHeuristic<State>>& heuristic, const Ref<GVD>& gvd);

			/// @brief Discretize a state.
			inline Pose2i DiscretizePose(const Pose2d& pose) const
			{
				return {
					static_cast<int>(pose.x() / m_param.spatialResolution),
					static_cast<int>(pose.y() / m_param.spatialResolution),
					static_cast<int>(pose.WrapTheta() / m_param.angularResolution),
				};
			}

			/// @brief Create an augmented state from a path.
			State CreateStateFromPath(const Ref<PathNonHolonomicSE2Base>& path) const;

			/// @brief Create an augmented state from a pose.
			State CreateStateFromPose(const Pose2d& pose) const;

			/// @copydoc Planner::AStarStateSpace::GetNeighborStates()
			virtual std::vector<std::tuple<State, Action, double>> GetNeighborStates(const State& state) override;

			/// @brief Set the goal state from the goal pose.
			void SetGoalState(const State& goal) { m_goalState = goal; }

			const SearchParameters& GetParameters() const { return m_param; }

		private:
			double GetVoronoiCost(const PathNonHolonomicSE2Base& path) const;

			/// @brief Update state assuming constant steer angle and bicycle
			/// kinematic model.
			/// @param[in] state The previous state.
			/// @param[in] delta The steering angle.
			/// @param[in] reverse The direction.
			/// @param[out] child The next state.
			/// @param[out] action The action connecting @state to @child.
			/// @param[out] cost The transition cost.
			/// @return Boolean to indicate if the transition is valid.
			[[nodiscard]] bool GetConstantSteerChild(const State& state, double delta, Direction direction, State& child, Action& action, double& cost) const;

			/// @brief Compute Reeds-Shepp path from a state to the goal.
			/// @param[in] state The previous state.
			/// @param[out] child The goal state containing the Reeds-Shepp
			/// path from the state to the goal.
			/// @param[out] action The action connecting @state to @child.
			/// @param[out] cost The transition cost.
			/// @return Boolean to indicate if the transition is valid.
			[[nodiscard]] bool GetReedsSheppChild(const State& state, State& child, Action& action, double& cost) const;

		private:
			SearchParameters m_param;
			Ref<StateValidatorOccupancyMap> m_validator;
			Ref<OccupancyMap> m_occupancyMap;
			Ref<AStarHeuristic<State>> m_heuristic;
			Ref<GVD> m_gvd;
			float m_voroFieldDiagResolution;
			Ref<KinematicBicycleModel> m_model;
			std::vector<double> m_deltas;
			State m_goalState;
		};

		class Adapter {
		public:
			const Pose2d& operator()(const State& state) { return state.continuousPose; }
		};
		/// @brief Adapter a heuristic for an A* search of Pose2d to an
		/// heuristic for A* search of HybrisAStar::State.
		class HeuristicAdapter : public AStarHeuristicAdapter<State, Pose2d, Adapter> {
		public:
			HeuristicAdapter(const Ref<AStarHeuristic<Pose2d>>& heuristic) :
				AStarHeuristicAdapter(heuristic, Adapter()) { }
		};

		/// @brief A* based graph search for Hybrid A*
		template <typename ExploredContainer = Explored<State, State::Hash, State::Equal>>
		class GraphSearch : public AStar<State, Action, State::Hash, State::Equal, true, ExploredContainer> {
			using AStarDeclType = AStar<State, Action, State::Hash, State::Equal, true, ExploredContainer>;

		public:
			GraphSearch() = default;

			/// @brief Initialize the graph search
			void Initialize(const Ref<StateValidatorOccupancyMap>& validator, const SearchParameters& p, const Ref<GVD>& gvd)
			{
				// Initialize heuristic
				auto reedsSheppPathCost = ReedsSheppPathCost::Build(validator->GetStateSpace()->bounds,
					p.spatialResolution, p.angularResolution, p.minTurningRadius,
					p.reverseCostMultiplier, p.forwardCostMultiplier, p.directionSwitchingCost);
				m_nonHoloHeuristic = makeRef<NonHolonomicHeuristic>(reedsSheppPathCost);
				m_obstacleHeuristic = makeRef<ObstaclesHeuristic>(validator->GetOccupancyMap(), std::min(p.reverseCostMultiplier, p.forwardCostMultiplier));
				auto combinedHeur = makeRef<AStarCombinedHeuristic<Pose2d>>();
				combinedHeur->Add(m_nonHoloHeuristic, m_obstacleHeuristic);
				auto heuristic = makeRef<HeuristicAdapter>(combinedHeur);

				// Initialize propagator
				m_propagator = makeRef<StatePropagator>(p);
				m_propagator->Initialize(validator, heuristic, gvd);

				AStarDeclType::Initialize(m_propagator, heuristic);
			}
			using AStarDeclType::Initialize;

			/// @brief Update the graph search
			void Update(const Pose2d& init, const Pose2d& goal)
			{
				this->SetInitState(m_propagator->CreateStateFromPose(init));
				this->SetGoalState(m_propagator->CreateStateFromPose(goal));
				m_obstacleHeuristic->Update(goal);
				m_propagator->SetGoalState(this->m_goal);
			}

			/// @brief Return the solution path connecting the initial state to
			/// the goal state.
			Ref<PathSE2CompositeNonHolonomic> GetCompositePath() const
			{
				auto path = makeRef<PathSE2CompositeNonHolonomic>();
				const auto& actions = this->GetActions();
				path->Reserve(actions.size());
				for (auto& action : actions)
					if (action.path)
						path->PushBack(action.path);
				return path;
			}

			/// @brief Return the set of explored path
			std::unordered_set<Ref<PathNonHolonomicSE2Base>> GetExploredPathSet() const
			{
				std::unordered_set<Ref<PathNonHolonomicSE2Base>> paths;

				if (!this->m_rootNode)
					return paths;

				this->m_rootNode->PreOrderTraversal([&](const AStarNode<State, Action>& node) {
					if (node->action.path)
						paths.insert(node->action.path);
				});
				return paths;
			}

			/// @brief Return the parameters used to conduct the search.
			const SearchParameters& GetParameters() const { return const_cast<const StatePropagator&>(*m_propagator).GetParameters(); }
			/// @brief Return the obstacle heuristic.
			const Ref<ObstaclesHeuristic>& GetObstacleHeuristic() const { return m_obstacleHeuristic; }

			/// @brief Return true if two poses are equal within some tolerance
			inline static bool IdenticalPoses(const Pose2d& lhs, const Pose2d& rhs, double tol = 1e-3)
			{
				return (lhs.position - rhs.position).norm() < tol && abs(lhs.theta - rhs.theta) < tol * M_PI / 180.0;
			}

		private:
			/// @copydoc Planner::AStar::IsSolution
			inline virtual bool IsSolution(AStarNode<State, Action>* node) override
			{
				return IdenticalPoses((*node)->state.continuousPose, this->m_goal.continuousPose);
			}

			/// @copydoc Planner::AStar::ProcessPossibleShortcut
			inline virtual void ProcessPossibleShortcut(AStarNode<State, Action>* frontierNode, Scope<AStarNode<State, Action>> childScope, AStarNode<State, Action>* node) override
			{
				// TODO FIXME check if a path with better cost exist between the two nodes if two poses are not identical?
				auto child = childScope.get();
				if (IdenticalPoses((*frontierNode)->state.continuousPose, (*child)->state.continuousPose))
					AStarDeclType::ProcessPossibleShortcut(frontierNode, std::move(childScope), node);
			}

		private:
			Ref<NonHolonomicHeuristic> m_nonHoloHeuristic;
			Ref<ObstaclesHeuristic> m_obstacleHeuristic;

			Ref<StatePropagator> m_propagator;
		};

		/// @brief A* based graph search for bidirectional Hybrid A*
		class BidirectionalGraphSearch : public BidirectionalAStar<State, Action, State::Hash, State::Equal, true, GraphSearch<ExploredMap<State, Action, State::Hash, State::Equal>>> {
			using UnidirectionalGraphSearch = GraphSearch<ExploredMap<State, Action, State::Hash, State::Equal>>;
			using BidirectionalAStarDeclType = BidirectionalAStar<State, Action, State::Hash, State::Equal, true, UnidirectionalGraphSearch>;

		public:
			BidirectionalGraphSearch() = default;

			/// @brief Initialize the graph search
			void Initialize(const Ref<StateValidatorOccupancyMap>& validator, const SearchParameters& p, const Ref<GVD>& gvd);

			/// @brief Update the graph search
			void Update(const Pose2d& init, const Pose2d& goal);

			/// @brief Return the solution path connecting the initial state to
			/// the goal state.
			Ref<PathSE2CompositeNonHolonomic> GetCompositePath() const;

			/// @brief Return the set of explored path
// 			std::unordered_set<Ref<PathNonHolonomicSE2Base>> GetExploredPathSet() const;
			std::tuple<std::unordered_set<Ref<PathNonHolonomicSE2Base>>, std::unordered_set<Ref<PathNonHolonomicSE2Base>>> GetExploredPathSet() const;

			/// @brief Return the parameters used to conduct the search.
			const SearchParameters& GetParameters() const { return const_cast<const StatePropagator&>(*m_fPropagator).GetParameters(); }
			/// @brief Return the forward obstacle heuristic.
			const Ref<ObstaclesHeuristic>& GetForwardObstacleHeuristic() const { return m_fObstacleHeuristic; }
			/// @brief Return the reverse obstacle heuristic.
			const Ref<ObstaclesHeuristic>& GetReverseObstacleHeuristic() const { return m_rObstacleHeuristic; }

		private:
			using BidirectionalAStarDeclType::Initialize;

		private:
			Ref<NonHolonomicHeuristic> m_fNonHoloHeuristic;
			Ref<TimeFlippedNonHolonomicHeuristic> m_rNonHoloHeuristic;
			Ref<ObstaclesHeuristic> m_fObstacleHeuristic, m_rObstacleHeuristic;

			Ref<StatePropagator> m_fPropagator, m_rPropagator;

		};

	public:
		HybridAStar() = default;
		virtual ~HybridAStar() = default;

		bool Initialize(const Ref<StateValidatorOccupancyMap>& validator, const SearchParameters& p);
		bool Initialize(const Ref<StateValidatorOccupancyMap>& validator)
		{
			return Initialize(validator, SearchParameters());
		}

		virtual Status SearchPath() override;

		virtual std::vector<Pose2d> GetPath() const override { return m_path; }

		Ref<StateValidatorOccupancyMap>& GetStateValidator() { return m_validator; }

		const Stats& GetStats() const { return m_stats; }
// 		const GraphSearch<>& GetGraphSearch() const { return m_graphSearch; }
// 		GraphSearch<>& GetGraphSearch() { return m_graphSearch; }
		const BidirectionalGraphSearch& GetGraphSearch() const { return m_graphSearch; }
		BidirectionalGraphSearch& GetGraphSearch() { return m_graphSearch; }
		const Smoother& GetSmoother() const { return m_smoother; }
		Smoother& GetSmoother() { return m_smoother; }

	public:
		/// @brief Distance to interpolate pose from the path
		float pathInterpolation = 0.1f;

	private:
		bool isInitialized = false;

		Ref<StateValidatorOccupancyMap> m_validator;
		Ref<GVD> m_gvd;

		// TODO create base for graph search and use a Scope
// 		GraphSearch<> m_graphSearch;
		BidirectionalGraphSearch m_graphSearch;
		Smoother m_smoother;

		Stats m_stats;
		std::vector<Pose2d> m_path;
	};
}
