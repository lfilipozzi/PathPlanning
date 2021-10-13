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
		class StateSpace : public AStarStateSpace<State>, public StateSpaceReedsShepp {
			friend HybridAStar;

		public:
			StateSpace(double spatialResolution = 1.0, double angularResolution = 0.0872) :
				spatialResolution(spatialResolution), angularResolution(angularResolution) { }

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
			State CreateStateFromPath(const Ref<Path<Pose2D<>>>& path) const
			{
				State state;
				state.discrete = DiscretizePose(path->GetFinalState());
				state.path = path;
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
			 * @copydoc Planner::AStarStateSpace::GetNeighborStates()
			 */
			virtual std::vector<std::tuple<State, double>> GetNeighborStates(const State& state) override
			{
				// TODO compute delta from minTurning radius and numMotion
				std::vector<double> deltas = { -10, -7.5, -5, -2.5, -0, 2.5, 5, 7.5, 10 };

				std::vector<std::tuple<State, double>> neighbors;
				neighbors.reserve(2 * deltas.size());
				for (double delta : deltas) {
					State newState;
					double cost;

					// Find neighbor state in forward direction
					if (ConstantSteer(state, delta, Direction::Forward, newState, cost))
						neighbors.push_back({ newState, cost });

					// Find neighbor state in forward direction
					if (ConstantSteer(state, delta, Direction::Backward, newState, cost))
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

		private:
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
			 * @brief Compute Reeds-Shepp path from a state to the goal.
			 * @param[in] state The previous state.
			 * @param[out] reedsShepp The goal state containing the Reeds-Shepp 
			 * path from the state to the goal.
			 * @param[out] cost The transition cost.
			 * @return Boolean to indicate if the transition is valid.
			 */
			[[nodiscard]] bool GetReedsSheppChild(const State& state, State& reedsShepp, double& cost) const
			{
				auto path = ComputeOptimalPath(state.GetPose(), m_goalState.GetPose());
				reedsShepp = CreateStateFromPath(path);
				if (!m_validator->IsPathValid(*(reedsShepp.path)))
					return false;

				// Compute transition cost
				cost = GetTransitionCost(reedsShepp.path);
				return true;
			}

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

		class NonHolonomicHeuristic : public AStarHeuristic<State> {
		public:
			// FIXME Should not be public, look at https://stackoverflow.com/questions/8147027/how-do-i-call-stdmake-shared-on-a-class-with-only-protected-or-private-const
			NonHolonomicHeuristic(double spatialResolution, double angularResolution, double minTurningRadius, 
				double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost,
				unsigned int numSpatial) :
				spatialResolution(spatialResolution), angularResolution(angularResolution), minTurningRadius(minTurningRadius), 
				reverseCostMultiplier(reverseCostMultiplier), forwardCostMultiplier(forwardCostMultiplier), directionSwitchingCost(directionSwitchingCost),
				numSpatial(numSpatial), numAngular(ceil(2*M_PI / angularResolution)), offset(floor(numSpatial / 2.0) * spatialResolution)
			{
				m_values = new double**[numSpatial];
				for (unsigned int i = 0; i < numSpatial; ++i) {
					m_values[i] = new double*[numSpatial];
					for(unsigned int j = 0; j < numSpatial; j++) {
						m_values[i][j] = new double[numAngular];
					}
				}
			}
			~NonHolonomicHeuristic()
			{
				for (unsigned int i = 0; i < numSpatial; i++) {
					for (unsigned int j = 0; j < numSpatial; j++) {
						delete[] m_values[i][j];
					}
					delete[] m_values[i];
				}
				delete[] m_values;
			}

			/**
			 * @brief Generate the NonHolonomicHeuristic heuristic.
			 */
			static Ref<NonHolonomicHeuristic> Build(const Ref<StateSpace>& stateSpace, double spatialSize)
			{
				PP_INFO("Generating non-holonomic heuristic without obstacle.");

				const double& spatialResolution = stateSpace->spatialResolution;
				const double& angularResolution = stateSpace->angularResolution;
				const double& minTurningRadius = stateSpace->minTurningRadius;
				const double& reverseCostMultiplier = stateSpace->reverseCostMultiplier;
				const double&forwardCostMultiplier = stateSpace->forwardCostMultiplier;
				const double& directionSwitchingCost = stateSpace->directionSwitchingCost;

				unsigned int numSpatialGuess = ceil(spatialSize / stateSpace->spatialResolution);
				if (numSpatialGuess % 2 == 0)
					numSpatialGuess++;

				auto heuristic = makeRef<NonHolonomicHeuristic>(
					spatialResolution, angularResolution, minTurningRadius, 
					reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost,
					numSpatialGuess);
				double***& values = heuristic->m_values;
				const double& offset = heuristic->offset;
				const unsigned int& numSpatial = heuristic->numSpatial;
				const unsigned int& numAngular = heuristic->numAngular;

				Pose2D<> goal(0.0, 0.0, 0.0);
				for (int i = 0; i < numSpatial; i++) {
					for (int j = 0; j < numSpatial; j++) {
						for (int k = 0; k < numAngular; k++) {
							Pose2D<> pose(i * spatialResolution - offset, j * spatialResolution - offset, k * angularResolution);
							values[i][j][k] = stateSpace->ComputeCost(pose, goal);
						}
					}
				}

				PP_INFO("Non-holonomic heuristic without obstacle generated.");
				return heuristic;
			}

			virtual double GetHeuristicValue(const State& from, const State& to) override
			{
				auto delta = to.GetPose() - from.GetPose();
				delta.theta = delta.WrapTheta();

				int i = (int)round((delta.x + offset) / spatialResolution);
				int j = (int)round((delta.y + offset) / spatialResolution);
				int k = (int)round(delta.theta / angularResolution);
				if (k == numAngular)
					k = 0;

				if (i < 0 || i >= numSpatial || j < 0 || j >= numSpatial) {
					double euclideanDistance = sqrtf(powf(delta.x, 2) + powf(delta.y, 2));
					double distanceMultiplier = std::min(reverseCostMultiplier, forwardCostMultiplier);
					return distanceMultiplier * euclideanDistance;
				}

				return m_values[i][j][k];
			}

		public:
			const double spatialResolution, angularResolution, minTurningRadius;
			const double reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost;
			const unsigned int numSpatial, numAngular;
			const double offset;

		private:
			double*** m_values;
		};

		class ObstaclesHeuristic : public AStarHeuristic<State> {
		public:
			ObstaclesHeuristic() = default;

			virtual double GetHeuristicValue(const State& /*from*/, const State& /*to*/) override
			{
				// TODO use flow-fields algorithm
				return 0.0;
			}

		private:
		};

	public:
		HybridAStar(const Ref<StateSpace>& stateSpace, const Ref<StateValidator<Pose2D<>>>& stateValidator) :
			m_stateSpace(stateSpace), m_stateValidator(stateValidator)
		{
			m_heuristic = makeRef<AStarCombinedHeuristic<State>>();
			m_nonHoloHeuristic = NonHolonomicHeuristic::Build(m_stateSpace, 100.0);
			m_obstacleHeuristic = makeRef<ObstaclesHeuristic>();
			m_heuristic->Add(m_nonHoloHeuristic, m_obstacleHeuristic);
			
			m_aStarSearch = makeScope<AStar<State, State::Hash>>(m_stateSpace, m_heuristic);
		}
		virtual ~HybridAStar() = default;

		virtual Status SearchPath() override
		{
			// Set the validator of the state-space
			if (!m_stateValidator || !m_heuristic)
				return Status::Failure;
			m_stateSpace->m_validator = m_stateValidator.get();
			m_stateSpace->m_heuristic = m_heuristic.get();

			// Run A* on 2D pose
			State initState = m_stateSpace->CreateStateFromPose(this->m_init);
			State goalState = m_stateSpace->SetGoalState(this->m_goal);
			m_aStarSearch->SetInitState(initState);
			m_aStarSearch->SetGoalState(goalState);
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
