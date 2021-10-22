#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "algo/path_planner.h"
#include "algo/a_star.h"
#include "geometry/2dplane.h"
#include "state_space/state_space_reeds_shepp.h"

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
	class HybridAStar : public PathPlanner<Pose2d> {
	public:
		/// @brief Augmented state used for the A* search in the hybrid A*
		/// algorithm.
		struct State {
			/// @brief Path connecting the previous state to the current state.
			Ref<Path<Pose2d>> path;
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
		class StateSpace : public AStarStateSpace<State>, public StateSpaceReedsShepp {
			friend HybridAStar;

		public:
			StateSpace(const std::array<Pose2d, 2>& bounds = { Pose2d(-100, -100, -M_PI), Pose2d(100, 100, M_PI) },
				double spatialResolution = 1.0, double angularResolution = 0.0872,
				unsigned int numGeneratedMotion = 5,
				double minTurningRadius = 1.0, double directionSwitchingCost = 0.0,
				double reverseCostMultiplier = 1.0, double forwardCostMultiplier = 1.0,
				double voronoiCostMultiplier = 1.0);

			/// @brief Discretize a state.
			Pose2i DiscretizePose(const Pose2d& pose) const;

			/// @brief Create an augmented state from a path.
			State CreateStateFromPath(const Ref<Path<Pose2d>>& path) const;

			/// @brief Create an augmented state from a pose.
			State CreateStateFromPose(Pose2d pose) const;

			/// @copydoc Planner::AStarStateSpace::GetNeighborStates()
			virtual std::vector<std::tuple<State, double>> GetNeighborStates(const State& state) override;

		private:
			double GetTransitionCost(const Path<Pose2d>& path) const;

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

			State& SetGoalState(const Pose2d& goal)
			{
				m_goalState = CreateStateFromPose(goal);
				return m_goalState;
			}

		public:
			const double voronoiCostMultiplier;
			const double spatialResolution, angularResolution;
			const unsigned int numGeneratedMotion;

		private:
			StateValidatorOccupancyMap* m_validator;
			AStarHeuristic<State>* m_heuristic;
			GVD* m_gvd;
			Ref<KinematicBicycleModel> m_model;
			std::vector<double> m_deltas;
			State m_goalState;
		};

	public:
		HybridAStar(const Ref<StateSpace>& stateSpace, const Ref<StateValidatorOccupancyMap>& stateValidator);
		virtual ~HybridAStar() = default;

		bool Initialize();

		virtual Status SearchPath() override;

		virtual std::vector<Pose2d> GetPath() override { return m_path; }

		Ref<StateSpace>& GetStateSpace() { return m_stateSpace; }
		Ref<StateValidatorOccupancyMap>& GetStateValidator() { return m_stateValidator; }

	private:
		Ref<StateSpace> m_stateSpace;
		Ref<StateValidatorOccupancyMap> m_stateValidator;
		Ref<NonHolonomicHeuristic> m_nonHoloHeuristic;
		Ref<ObstaclesHeuristic> m_obstacleHeuristic;
		Ref<AStarCombinedHeuristic<State>> m_heuristic;
		Scope<GVD> m_gvd;
		Scope<AStar<State, State::Hash, State::Equal>> m_aStarSearch;
		std::vector<Pose2d> m_path;
		bool isInitialized = false;
	};
}
