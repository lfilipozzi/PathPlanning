#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "path_planner.h"
#include "a_star.h"
#include "geometry/pose.h"
#include "reeds_shepp.h"
#include "state_validator.h"

namespace Planner {
	/**
	 * @brief Tunable parameters of the HybridAStar algorithm.
	 */
	struct HybridAStarParameters {
		// TODO turning radius, resolution, cost multipliers
	};

	/**
	 * @brief Conduct a Hybrid A* search to find the optimal path.
	 * @details The configuration space is defined by Reeds-Shepp path. The 
	 * continuous state-space is discretized to perform a A* search. Two poses
	 * are considered equal if they belong to the same cell in the configuration
	 * space. 
	 * The A* search provides an optimal solution that is then smoothed by a 
	 * nonlinear optimization.
	 */
	class HybridAStar : public PathPlanner<Pose2D<>> {
	public:
		/**
		 * @brief Augmented state used for hybrid A* search
		 */
		struct State {
			Pose2D<> continuous;
			Pose2D<int> discrete;

			bool operator==(const State& rhs) const
			{
				return discrete == rhs.discrete;
			}
		};

		/**
		 * @brief Hash to discretize the 3D configuration space.
		 * @details Two states are considered equal if they correspond to the 
		 * same"cell", i.e. they have their position difference is less than the
		 * spatial resolution and if their heading difference is less than the 
		 * angular resolution.
		 */
		struct HashState {
			std::size_t operator()(const State& state) const
			{
				auto pose = state.discrete;
				std::hash<Pose2D<int>> hasher;
				return hasher(pose);
			}
		};

		/**
		 * @brief Define the state-space used for the A* search.
		 */
		class StateSpace : public AStarStateSpace<State> {
		public:
			StateSpace(const Ref<StateSpaceReedsShepp>& reedsSheppStateSpace, const Ref<StateValidator<Pose2D<>>>& stateValidator,
				double spatialResolution = 1.0, double angularResolution = 0.0872) :
				spatialResolution(spatialResolution),
				angularResolution(angularResolution),
				m_reedsSheppStateSpace(reedsSheppStateSpace), m_validator(stateValidator) { }

			/**
			 * @brief Discretize a state.
			 */
			Pose2D<int> DiscretizePose(const Pose2D<>& pose)
			{
				return {
					static_cast<int>(pose.x / spatialResolution),
					static_cast<int>(pose.y / spatialResolution),
					static_cast<int>(pose.WrapTheta() / angularResolution),
				};
			}

			/**
			 * @brief Create an augmented state from a 2D pose.
			 */
			State CreateState(const Pose2D<>& pose)
			{
				State state;
				state.continuous = pose;
				state.discrete = DiscretizePose(pose);

				return state;
			}

			/**
			 * @brief Update state assuming constant steer angle  and bicycle
			 * kinematic model.
			 */
			State ConstantSteer(const State& state, double delta, bool reverse = false)
			{
				double L = 2.6; // Wheelbase
				double b = L / 2; // Distance to rear axle

				// Compute distance to travel to another cell
				double spatialResolution = 1.0;
				double d = spatialResolution * 1.5;
				if (reverse)
					d = -d;

				// States
				auto cont_pose = state.continuous;
				double& x = cont_pose.x;
				double& y = cont_pose.y;
				double& theta = cont_pose.theta;

				// Update states
				double beta = atan(b * tan(delta) / L);
				x = x + d * cos(theta + beta);
				y = y + d * sin(theta + beta);
				theta = theta + d * cos(beta) * tan(delta) / L;

				// Create new state
				State next = CreateState(cont_pose);

				return next;
			}

			/**
			 * @copydoc Planner::AStarStateSpace::GetNeighborStates()
			 */
			virtual std::vector<std::tuple<State, double>> GetNeighborStates(const State& state) override
			{
				std::vector<double> deltas = { -10, -7.5, -5, -2.5, -0, 2.5, 5, 7.5, 10 };

				std::vector<std::tuple<State, double>> vec;
				vec.reserve(deltas.size());
				for (double delta : deltas) {
					// Find neighbor state
					auto newState = ConstantSteer(state, delta);

					// Validate transition: if the transition is not valid, use
					// last valid state only if it belongs to another cell.
					Pose2D<> lastValid;
					// TODO find efficient way to validate path for a Reeds Shepp path defined by a ReedsShepp::PathSegment
					// FIXME also this is not a good definition of the path, best would be to define different path
					// (e.g. Dublins, Reeds Shepp, SE(2)) and provide a IsPathValid for each implementation
					bool valid = m_validator->IsPathValid(state.continuous, newState.continuous, &lastValid);
					if (!valid) {
						if (newState.discrete != DiscretizePose(lastValid)) {
							newState = CreateState(lastValid);
						}
					}

					// Compute transition cost
					// TODO use real cost, not euclidean distance
					auto deltaState = newState.continuous - state.continuous;
					double cost = sqrtf(powf(deltaState.x, 2) + powf(deltaState.y, 2));

					vec.push_back({ std::move(newState), cost });
				}

				return vec;
			}

		public:
			const double spatialResolution;
			const double angularResolution;

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
				auto delta = to.continuous - from.continuous;
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
		HybridAStar(const Ref<StateValidator<Pose2D<>>>& stateValidator)
		{
			m_reedsSheppStateSpace = makeRef<StateSpaceReedsShepp>();
			m_stateSpace = makeRef<StateSpace>(m_reedsSheppStateSpace, stateValidator);
			auto heuristic = Heuristic(m_reedsSheppStateSpace, stateValidator);
			m_aStarSearch = makeScope<AStar<State, HashState>>(m_stateSpace, heuristic);
		}
		virtual ~HybridAStar() = default;

		HybridAStarParameters& GetParameters() { return m_parameters; }
		const HybridAStarParameters& GetParameters() const { return m_parameters; }
		void SetParameters(const HybridAStarParameters& params) { m_parameters = params; } // TODO need to update parameters in state-space ...

		virtual Status SearchPath() override
		{
			// Run A* on 2D pose
			State init_state = m_stateSpace->CreateState(this->m_init);
			State goal_state = m_stateSpace->CreateState(this->m_goal);
			m_aStarSearch->SetInitState(init_state);
			m_aStarSearch->SetGoalState(goal_state);
			auto status = m_aStarSearch->SearchPath();

			auto solutionStates = m_aStarSearch->GetPath();
			m_path.reserve(solutionStates.size());
			for (auto state : solutionStates) {
				m_path.push_back(state.continuous);
			}

			// Smooth the path
			// TODO

			return status;
		}

		virtual std::vector<Pose2D<>> GetPath() override
		{
			return m_path;
		}

		Ref<StateSpace> GetStateSpace() const { return m_stateSpace; }

	private:
		Ref<StateSpaceReedsShepp> m_reedsSheppStateSpace;
		Ref<StateValidator<Pose2D<>>> m_stateValidator;
		Ref<StateSpace> m_stateSpace;
		Scope<AStar<State, HashState>> m_aStarSearch;
		std::vector<Pose2D<>> m_path;
		HybridAStarParameters m_parameters;
	};
}
