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
	class HybridAStar : public PlanarPathPlanner {
	public:
		struct SearchParameters {
			const double minTurningRadius = 1.0;
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
			Ref<PlanarPath> path;
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
			State CreateStateFromPath(const Ref<PlanarPath>& path) const;

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
			double GetTransitionCost(const PlanarPath& path) const;

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
			Ref<AStarHeuristic<State>> m_heuristic;
			Ref<GVD> m_gvd;
			Ref<KinematicBicycleModel> m_model;
			std::vector<double> m_deltas;
			State m_goalState;
		};

		/// @brief Adapter heuristic A* of Pose2d to heuristic A* for
		/// HybrisAStar::State.
		class HeuristicAdapter : public AStarHeuristic<State> {
		public:
			HeuristicAdapter(const Ref<AStarHeuristic<Pose2d>>& heuristic) :
				m_heuristic(heuristic) { }

			inline virtual double GetHeuristicValue(const State& from, const State& to) override
			{
				return m_heuristic->GetHeuristicValue(from.GetPose(), to.GetPose());
			}

		private:
			Ref<AStarHeuristic<Pose2d>> m_heuristic;
		};

		using AStarDeclType = AStar<State, State::Hash, State::Equal>;

	public:
		HybridAStar(const Ref<StateValidatorOccupancyMap>& validator);
		HybridAStar(const Ref<StateValidatorOccupancyMap>& validator, const SearchParameters& parameters);
		virtual ~HybridAStar() = default;

		bool Initialize();

		virtual Status SearchPath() override;

		virtual std::vector<Pose2d> GetPath() override { return m_path; }

		Ref<StateValidatorOccupancyMap>& GetStateValidator() { return m_validator; }

		SearchParameters GetSearchParameters() { return m_propagator->GetParameters(); }
		const SearchParameters& GetSearchParameters() const
		{
			return const_cast<const StatePropagator&>(*m_propagator).GetParameters();
		}
		Smoother::Parameters GetSmootherParameters() { return m_smoother->GetParameters(); }
		const Smoother::Parameters& GetSmootherParameters() const
		{
			return const_cast<const Smoother&>(*m_smoother).GetParameters();
		}

		void VisualizeObstacleHeuristic(const std::string& filename) const;

		Scope<GenericNode<Pose2d>> GetTree() const;

	private:
		Scope<GenericNode<Pose2d>> CopySubTreeOfPose(const AStarDeclType::Node* src) const;

	public:
		/// @brief Distance to interpolate pose from the path
		float pathInterpolation = 0.1f;

	private:
		Ref<StateValidatorOccupancyMap> m_validator;
		Ref<StatePropagator> m_propagator;
		Ref<NonHolonomicHeuristic> m_nonHoloHeuristic;
		Ref<ObstaclesHeuristic> m_obstacleHeuristic;
		Ref<GVD> m_gvd;
		Scope<AStarDeclType> m_aStarSearch;
		Scope<Smoother> m_smoother;
		std::vector<Pose2d> m_path;
		bool isInitialized = false;
	};
}
