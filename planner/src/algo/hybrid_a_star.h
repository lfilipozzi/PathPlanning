#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "core/base.h"
#include "algo/path_planner.h"
#include "algo/a_star.h"
#include "algo/smoother.h"
#include "geometry/2dplane.h"

namespace Planner {

	class NonHolonomicHeuristic;
	class ObstaclesHeuristic;
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
			const double wheelbase = 2.6; // TODO add maxSteeringAngle to choose how to propagate bicycle model
			const double minTurningRadius = 2.0;
			const double directionSwitchingCost = 0.0;
			const double reverseCostMultiplier = 1.0;
			const double forwardCostMultiplier = 1.0;
			const double voronoiCostMultiplier = 1.0;
			const unsigned int numGeneratedMotion = 5;
			const double spatialResolution = 1.0;
			const double angularResolution = 0.0872;

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

	private:
		/// @brief Augmented state used for the A* search in the hybrid A*
		/// algorithm.
		struct State {
			/// @brief Path connecting the previous state to the current state.
			Ref<PathNonHolonomicSE2Base> path;
			/// @brief Discretized state.
			Pose2i discrete;

			/// @brief Return the real (continuous) pose associated to the state.
			const Pose2d& GetPose() const { return path->GetFinalState(); }

			bool operator==(const State& rhs) const { return discrete == rhs.discrete; }

			/// @brief Hash to discretize the 3D configuration space.
			/// @details Two states are considered equal if they correspond to the
			/// same"cell", i.e. they have their position difference is less than the
			/// spatial resolution and if their heading difference is less than the
			/// angular resolution.
			struct Hash {
				std::size_t operator()(const State& state) const
				{
					auto pose = state.discrete;
					std::hash<Pose2i> hasher;
					return hasher(pose);
				}
			};

			struct Equal {
				bool operator()(const State& lhs, const State& rhs) const
				{
					return lhs.discrete == rhs.discrete;
				}
			};
		};

		/// @brief Define the state-space used for the A* search.
		class StatePropagator : public AStarStatePropagator<State> {
		public:
			StatePropagator(const SearchParameters& parameters);

			void Initialize(const Ref<StateValidatorOccupancyMap>& stateValidator,
				const Ref<AStarHeuristic<State>>& heuristic, const Ref<GVD>& gvd);

			/// @brief Discretize a state.
			Pose2i DiscretizePose(const Pose2d& pose) const;

			/// @brief Create an augmented state from a path.
			State CreateStateFromPath(Ref<PathNonHolonomicSE2Base>&& path) const;

			/// @brief Create an augmented state from a pose.
			State CreateStateFromPose(Pose2d pose) const;

			/// @copydoc Planner::AStarStateSpace::GetNeighborStates()
			virtual std::vector<std::tuple<State, double>> GetNeighborStates(const State& state) override;

			/// @brief Set the goal state from the goal pose.
			State& SetGoalState(const Pose2d& goal)
			{
				m_goalState = CreateStateFromPose(goal);
				return m_goalState;
			}

			SearchParameters GetParameters() { return m_param; }
			const SearchParameters& GetParameters() const { return m_param; }

		private:
			double GetDirectionSwitchingCost(const PathNonHolonomicSE2Base& parentPath, const PathNonHolonomicSE2Base& childPath) const;
			double GetVoronoiCost(const PathNonHolonomicSE2Base& path) const;

			/// @brief Update state assuming constant steer angle and bicycle
			/// kinematic model.
			/// @param[in] state The previous state.
			/// @param[in] delta The steering angle.
			/// @param[in] reverse The direction.
			/// @param[out] child The next state.
			/// @param[out] cost The transition cost.
			/// @return Boolean to indicate if the transition is valid.
			[[nodiscard]] bool GetConstantSteerChild(const State& state, double delta, Direction direction, State& child, double& cost) const;

			/// @brief Compute Reeds-Shepp path from a state to the goal.
			/// @param[in] state The previous state.
			/// @param[out] child The goal state containing the Reeds-Shepp
			/// path from the state to the goal.
			/// @param[out] cost The transition cost.
			/// @return Boolean to indicate if the transition is valid.
			[[nodiscard]] bool GetReedsSheppChild(const State& state, State& child, double& cost) const;

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
			const Pose2d& operator()(const State& state) { return state.GetPose(); }
		};
		/// @brief Adapter a heuristic for an A* search of Pose2d to an
		/// heuristic for A* search of HybrisAStar::State.
		class HeuristicAdapter : public AStarHeuristicAdapter<State, Pose2d, Adapter> {
		public:
			HeuristicAdapter(const Ref<AStarHeuristic<Pose2d>>& heuristic) :
				AStarHeuristicAdapter(heuristic, Adapter()) { }
		};

		/// @brief A* based graph search for Hybrid A*
		class GraphSearch : public AStar<State, State::Hash, State::Equal, true> {
		public:
			GraphSearch() = default;

		protected:
			/// @copydoc Planner::AStar::IsSolution
			inline virtual bool IsSolution(Node* node) override
			{
				PP_PROFILE_FUNCTION();

				return IdenticalPoses(node->GetState().GetPose(), this->m_goal.GetPose());
			}

			/// @copydoc Planner::AStar::ProcessPossibleShorterPath
			inline virtual void ProcessPossibleShorterPath(Node* frontierNode, Scope<Node> childScope, Node* node) override
			{
				PP_PROFILE_FUNCTION();

				// TODO FIXME check if a path with better cost exist between the two nodes if two poses are not identical?
				auto child = childScope.get();
				bool identicalPose = IdenticalPoses(frontierNode->GetState().GetPose(), node->GetState().GetPose());
				bool betterCost = frontierNode->meta.totalCost > child->meta.totalCost;
				if (identicalPose && betterCost) {
					// Replace the node in frontier by the newly find better node.
					auto previousChildScope = frontierNode->GetParent()->ReplaceChild(frontierNode, std::move(childScope), false);
					// Replace the node in the frontier by child
					m_frontier.Remove(previousChildScope.get());
					m_frontier.Push(child);
				}
			}

		private:
			inline bool IdenticalPoses(const Pose2d& lhs, const Pose2d& rhs, double tol = 1e-3)
			{
				return (lhs.position - rhs.position).norm() < tol && abs(lhs.theta - rhs.theta) < tol * M_PI / 180.0;
			}
		};

	public:
		HybridAStar();
		HybridAStar(const SearchParameters& parameters);
		virtual ~HybridAStar() = default;

		bool Initialize(const Ref<StateValidatorOccupancyMap>& validator);

		virtual Status SearchPath() override;

		virtual std::vector<Pose2d> GetPath() const override { return m_path; }

		Ref<StateValidatorOccupancyMap>& GetStateValidator() { return m_validator; }

		const SearchParameters& GetSearchParameters() const
		{
			return const_cast<const StatePropagator&>(*m_propagator).GetParameters();
		}
		const Smoother::Parameters& GetSmootherParameters() const
		{
			return const_cast<const Smoother&>(m_smoother).GetParameters();
		}
		void SetSmootherParameters(const Smoother::Parameters& param) { m_smoother.SetParameters(param); }

		void VisualizeObstacleHeuristic(const std::string& filename) const;

		std::unordered_set<Ref<PathNonHolonomicSE2Base>> GetGraphSearchExploredSet() const;
		std::vector<Ref<PathNonHolonomicSE2Base>> GetGraphSearchPath() const;
		double GetGraphSearchOptimalCost() const;

	public:
		/// @brief Distance to interpolate pose from the path
		float pathInterpolation = 0.1f;

	private:
		bool isInitialized = false;

		Ref<StateValidatorOccupancyMap> m_validator;
		Ref<StatePropagator> m_propagator;
		Ref<NonHolonomicHeuristic> m_nonHoloHeuristic;
		Ref<ObstaclesHeuristic> m_obstacleHeuristic;
		Ref<GVD> m_gvd;
		GraphSearch m_graphSearch;
		Smoother m_smoother;

		std::vector<Pose2d> m_path;
	};
}
