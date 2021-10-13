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
	template <typename T>
	class StateValidator;

	/// @brief Conduct a Hybrid A* search to find the optimal path.
	/// @details The continuous state-space is discretized to perform a A*
	/// search. Two poses are considered equal if they belong to the same cell in
	/// the configuration space.
	/// The A* search provides an optimal solution that is then smoothed by a
	/// nonlinear optimization.
	class HybridAStar : public PathPlanner<Pose2D<>> {
	public:
		/// @brief Augmented state used for the A* search in the hybrid A*
		/// algorithm.
		struct State {
			/// @brief Path connecting the previous state to the current state.
			Ref<Path<Pose2D<>>> path;
			/// @brief Discretized state.
			Pose2D<int> discrete;

			/// @brief Return the real (continuous) pose associated to the state.
			const Pose2D<>& GetPose() const { return path->GetFinalState(); }

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
					std::hash<Pose2D<int>> hasher;
					return hasher(pose);
				}
			};
		};

		/// @brief Define the state-space used for the A* search.
		class StateSpace : public AStarStateSpace<State>, public StateSpaceReedsShepp {
			friend HybridAStar;

		public:
			StateSpace(double spatialResolution = 1.0, double angularResolution = 0.0872) :
				spatialResolution(spatialResolution), angularResolution(angularResolution) { }

			/// @brief Discretize a state.
			Pose2D<int> DiscretizePose(const Pose2D<>& pose) const;

			/// @brief Create an augmented state from a path.
			State CreateStateFromPath(const Ref<Path<Pose2D<>>>& path) const;

			/// @brief Create an augmented state from a pose.
			State CreateStateFromPose(Pose2D<> pose) const;

			/// @copydoc Planner::AStarStateSpace::GetNeighborStates()
			virtual std::vector<std::tuple<State, double>> GetNeighborStates(const State& state) override;

		private:
			double GetTransitionCost(const Ref<Path<Pose2D<>>>& path) const;

			/// @brief Update state assuming constant steer angle and bicycle
			/// kinematic model.
			/// @param[in] state The previous state.
			/// @param[in] delta The steering angle.
			/// @param[in] reverse The direction.
			/// @param[out] next The next state.
			/// @param[out] cost The transition cost.
			/// @return Boolean to indicate if the transition is valid.
			[[nodiscard]] bool ConstantSteer(const State& state, double delta, Direction direction, State& next, double& cost) const;

			/// @brief Compute Reeds-Shepp path from a state to the goal.
			/// @param[in] state The previous state.
			/// @param[out] reedsShepp The goal state containing the Reeds-Shepp
			/// path from the state to the goal.
			/// @param[out] cost The transition cost.
			/// @return Boolean to indicate if the transition is valid.
			[[nodiscard]] bool GetReedsSheppChild(const State& state, State& reedsShepp, double& cost) const;

			State& SetGoalState(const Pose2D<>& goal)
			{
				m_goalState = CreateStateFromPose(goal);
				return m_goalState;
			}

		public:
			const double spatialResolution = 1.0;
			const double angularResolution = 0.0872;
			unsigned int numGeneratedMotion = 5;

		private:
			StateValidator<Pose2D<>>* m_validator;
			AStarHeuristic<State>* m_heuristic;
			State m_goalState;
		};

	public:
		HybridAStar(const Ref<StateSpace>& stateSpace, const Ref<StateValidator<Pose2D<>>>& stateValidator);
		virtual ~HybridAStar() = default;

		virtual Status SearchPath() override;

		virtual std::vector<Pose2D<>> GetPath() override { return m_path; }

		Ref<StateSpace>& GetStateSpace() { return m_stateSpace; }
		Ref<StateValidator<Pose2D<>>>& GetStateValidator() { return m_stateValidator; }

	private:
		Ref<StateSpace> m_stateSpace;
		Ref<StateValidator<Pose2D<>>> m_stateValidator;
		Ref<NonHolonomicHeuristic> m_nonHoloHeuristic;
		Ref<ObstaclesHeuristic> m_obstacleHeuristic;
		Ref<AStarCombinedHeuristic<State>> m_heuristic;
		Scope<AStar<State, State::Hash>> m_aStarSearch;
		std::vector<Pose2D<>> m_path;
	};
}
