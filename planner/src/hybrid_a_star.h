#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "path_planner.h"
#include "a_star.h"
#include "geometry/pose.h"
#include "reeds_shepp.h"
#include "state_validator.h"
#include "path_constant_steer.h"

namespace Planner {

	/**
	 * @brief Conduct a Hybrid A* search to find the optimal path.
	 * @details The continuous state-space is discretized to perform a A* 
	 * search. Two poses are considered equal if they belong to the same cell in
	 * the configuration space. 
	 * The A* search provides an optimal solution that is then smoothed by a 
	 * nonlinear optimization.
	 */
	class HybridAStar : public PathPlanner<Pose2D<>> {
	public:
		/**
		 * @brief Augmented state used for the A* search in the hybrid A* 
		 * algorithm.
		 */
		struct State {
			/// @brief Path connecting the previous state to the current state.
			Ref<Path<Pose2D<>>> path;
			/// @brief Discretized state.
			Pose2D<int> discrete;

			/// @brief Return the real (continuous) pose associated to the state.
			const Pose2D<>& GetPose() const
			{
				return path->GetFinalState();
			}

			bool operator==(const State& rhs) const
			{
				return discrete == rhs.discrete;
			}

			/**
			* @brief Hash to discretize the 3D configuration space.
			* @details Two states are considered equal if they correspond to the 
			* same"cell", i.e. they have their position difference is less than the
			* spatial resolution and if their heading difference is less than the 
			* angular resolution.
			*/
			struct Hash {
				std::size_t operator()(const State& state) const
				{
					auto pose = state.discrete;
					std::hash<Pose2D<int>> hasher;
					return hasher(pose);
				}
			};
		};

		/**
		 * @brief Define the state-space used for the A* search.
		 */
		class StateSpace : public AStarStateSpace<State> {
		public:
			StateSpace(const Ref<StateSpaceReedsShepp>& reedsSheppStateSpace, const Ref<StateValidator<Pose2D<>>>& stateValidator) :
				m_reedsSheppStateSpace(reedsSheppStateSpace), m_validator(stateValidator) { }

			Ref<StateSpaceReedsShepp>& GetReedsSheppStateSpace() { return m_reedsSheppStateSpace; }

			/**
			 * @brief Discretize a state.
			 */
			Pose2D<int> DiscretizePose(const Pose2D<>& pose) const
			{
				return {
					static_cast<int>(pose.x / spatialResolution),
					static_cast<int>(pose.y / spatialResolution),
					static_cast<int>(pose.WrapTheta() / angularResolution),
				};
			}

			/**
			 * @brief Create an augmented state from a path.
			 */
			State CreateStateFromPath(const Ref<PathConstantSteer>& path) const
			{
				State state;
				state.discrete = DiscretizePose(path->GetFinalState());
				state.path = std::move(path);
				return state;
			}

			/**
			 * @brief Create an augmented state from a pose.
			 */
			State CreateStateFromPose(Pose2D<> pose) const
			{
				State state;
				state.discrete = DiscretizePose(pose);
				state.path = makeRef<PathConstantSteer>(pose);
				return state;
			}

			/**
			 * @brief Compute the transition cost
			 */
			double GetTransitionCost(const Ref<Path<Pose2D<>>>& path) const
			{
				if (!path)
					return 0.0;

				// TODO use real cost, not euclidean distance
				auto delta = path->GetFinalState() - path->GetInitialState();
				double cost = sqrtf(powf(delta.x, 2) + powf(delta.y, 2));
				return cost;
			}

			/**
			 * @brief Update state assuming constant steer angle and bicycle
			 * kinematic model.
			 * @param[in] state The previous state.
			 * @param[in] delta The steering angle.
			 * @param[in] reverse The direction.
			 * @param[out] next The next state.
			 * @param[out] cost The transition cost.
			 * @return Boolean to indicate if the transition is valid.
			 */
			[[nodiscard]] bool ConstantSteer(const State& state, double delta, Direction direction, State& next, double& cost) const
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

			/**
			 * @copydoc Planner::AStarStateSpace::GetNeighborStates()
			 */
			virtual std::vector<std::tuple<State, double>> GetNeighborStates(const State& state) override
			{
				// TODO compute delta from minTurning radius and numMotion
				std::vector<double> deltas = { -10, -7.5, -5, -2.5, -0, 2.5, 5, 7.5, 10 };

				std::vector<std::tuple<State, double>> vec;
				vec.reserve(2 * deltas.size());
				for (double delta : deltas) {
					State newState;
					double cost;

					// Find neighbor state in forward direction
					if (ConstantSteer(state, delta, Direction::Forward, newState, cost))
						vec.push_back({ newState, cost });

					// Find neighbor state in forward direction
					if (ConstantSteer(state, delta, Direction::Backward, newState, cost))
						vec.push_back({ newState, cost });
				}

				// Randomly add a child using a Reeds-Shepp path with a
				// probability which is function of the heuristic to the goal
				// TODO add reeds shepp path

				return vec;
			}

		public:
			double spatialResolution = 1.0;
			double angularResolution = 0.0872;
			unsigned int numGeneratedMotion = 5;

		private:
			Ref<StateSpaceReedsShepp> m_reedsSheppStateSpace;
			Ref<StateValidator<Pose2D<>>> m_validator;
		};

		/**
		 * @brief Heuristic used by the A* algorithm.
		 * @details Use the maximum of two heuristic:
		 *   - Heuristic with constraints without obstacles
		 *   - Heuristic with obstacles without constraints
		 * The heuristic with obstacles but without constraints is implemented 
		 * by running a 2D flow fields algorithm.
		 */
		struct Heuristic {
			Heuristic(const Ref<StateSpaceReedsShepp>& reedsSheppStateSpace, const Ref<StateValidator<Pose2D<>>>& stateValidator) :
				m_reedsSheppStateSpace(reedsSheppStateSpace), m_validator(stateValidator) { }

			double HeuristicConstraintsWithoutObstacles(const State& from, const State& to)
			{
				// TODO compute arc
				auto delta = to.GetPose() - from.GetPose();
				double euclidean = sqrtf(powf(delta.x, 2) + powf(delta.y, 2));
				return euclidean;
			}

			double HeuristicObstaclesWithoutConstraints(const State& /*from*/, const State& /*to*/)
			{
				// TODO use flow-fields algorithm
				return 0.0;
			}

			double operator()(const State& from, const State& to)
			{
				return std::max(
					HeuristicConstraintsWithoutObstacles(from, to),
					HeuristicObstaclesWithoutConstraints(from, to));
			}

		private:
			Ref<StateSpaceReedsShepp> m_reedsSheppStateSpace;
			Ref<StateValidator<Pose2D<>>> m_validator;
		};

	public:
		HybridAStar(const Ref<StateSpaceReedsShepp>& reedsSheppStateSpace, const Ref<StateValidator<Pose2D<>>>& stateValidator) :
			m_reedsSheppStateSpace(reedsSheppStateSpace), m_stateValidator(stateValidator)
		{
			m_stateSpace = makeRef<StateSpace>(reedsSheppStateSpace, stateValidator);
			auto heuristic = Heuristic(reedsSheppStateSpace, stateValidator);
			m_aStarSearch = makeScope<AStar<State, State::Hash>>(m_stateSpace, heuristic);
		}
		virtual ~HybridAStar() = default;

		virtual Status SearchPath() override
		{
			// Run A* on 2D pose
			State init_state = m_stateSpace->CreateStateFromPose(this->m_init);
			State goal_state = m_stateSpace->CreateStateFromPose(this->m_goal);
			m_aStarSearch->SetInitState(init_state);
			m_aStarSearch->SetGoalState(goal_state);
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

		virtual std::vector<Pose2D<>> GetPath() override
		{
			return m_path;
		}

		Ref<StateSpaceReedsShepp>& GetReedsSheppStateSpace() { return m_reedsSheppStateSpace; }
		Ref<StateValidator<Pose2D<>>>& GetStateValidator() { return m_stateValidator; }
		Ref<StateSpace>& GetStateSpace() { return m_stateSpace; }

	private:
		Ref<StateSpaceReedsShepp> m_reedsSheppStateSpace;
		Ref<StateValidator<Pose2D<>>> m_stateValidator;
		Ref<StateSpace> m_stateSpace;
		Scope<AStar<State, State::Hash>> m_aStarSearch;
		std::vector<Pose2D<>> m_path;
	};
}
