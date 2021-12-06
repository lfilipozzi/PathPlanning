#pragma once

#include "algo/a_star.h"
#include <unordered_map>

namespace Planner {

	/// @brief Create a consistent pair of heuristic for bidirectional A*.
	template <typename State>
	class AverageHeuristic : public AStarHeuristic<State> {
		template <typename S, typename A, typename HashState, typename EqualState, bool GraphSearch, typename Algo>
		friend class BidirectionalAStar;

	public:
		AverageHeuristic(const Ref<AStarHeuristic<State>>& toGoalHeuristic, const Ref<AStarHeuristic<State>>& toInitHeuristic) :
			m_toGoalHeuristic(toGoalHeuristic), m_toInitHeuristic(toInitHeuristic) { }

		/// @copydoc Planner::AStar::GetHeuristicValue
		virtual double GetHeuristicValue(const State& state) override
		{
			return m_constant + (m_toGoalHeuristic->GetHeuristicValue(state) - m_toInitHeuristic->GetHeuristicValue(state)) / 2.0;
		}

	protected:
		virtual void SetGoal(const State& /*goal*/) override final { }

	private:
		/// @brief Update the heuristic
		/// @param init The initial state of the search (in the direction of the search).
		/// @param goal The goal state of the search (in the direction of the search).
		void Update(const State& init, const State& goal)
		{
			m_init = init;
			m_goal = goal;
			m_toGoalHeuristic->SetGoal(goal);
			m_toInitHeuristic->SetGoal(init);
			m_constant = m_toInitHeuristic->GetHeuristicValue(goal) / 2.0;
		}

	private:
		Ref<AStarHeuristic<State>> m_toGoalHeuristic, m_toInitHeuristic;
		State m_init, m_goal;
		double m_constant;
	};

	template <
		typename State,
		typename Action,
		typename HashState = std::hash<State>,
		typename EqualState = std::equal_to<State>,
		bool GraphSearch = true,
		typename UnidirectionalAStar = AStar<State, Action, HashState, EqualState, GraphSearch, ExploredMap<State, Action, HashState, EqualState>>>
		// FIXME Maybe use polymorphism here instead of adding a template argument for the unidirectional algorithm
	class BidirectionalAStar : public PathPlanner<State> {
		static_assert(std::is_base_of<AStar<State, Action, HashState, EqualState, GraphSearch, ExploredMap<State, Action, HashState, EqualState>>, UnidirectionalAStar>::value, "The unidirectional algorithm is not an A* search");

	public:
		BidirectionalAStar() = default;
		template <typename... Args>
		BidirectionalAStar(Args... args) :
			m_fSearch(std::forward<Args>(args)...), m_rSearch(std::forward<Args>(args)...) { }
		virtual ~BidirectionalAStar() = default;

		/// @brief Return a consistent pair of heuristic for bidirectional
		/// search.
		static std::tuple<Ref<AStarHeuristic<State>>, Ref<AStarHeuristic<State>>> GetAverageHeuristicPair(const Ref<AStarHeuristic<State>>& fHeuristic, const Ref<AStarHeuristic<State>>& rHeuristic)
		{
			auto fAverageHeuristic = makeRef<AverageHeuristic<State>>(fHeuristic, rHeuristic);
			auto rAverageHeuristic = makeRef<AverageHeuristic<State>>(rHeuristic, fHeuristic);
			return std::make_tuple(fAverageHeuristic, rAverageHeuristic);
		}

		/// @copydoc Planner::PathPlanner::GetPath
		virtual std::vector<State> GetPath() const override
		{
			auto fPath = m_fSearch.GetPath();
			auto rPath = m_rSearch.GetPath();
			fPath.insert(fPath.end(), rPath.rbegin(), rPath.rend());
			return fPath;
		}

		/// @brief Return the actions from the initial state to the goal state.
		std::vector<Action> GetActions() const
		{
			auto fActions = m_fSearch.GetActions();
			auto rActions = m_rSearch.GetActions();
			fActions.insert(fActions.end(), rActions.rbegin(), rActions.rend());
			return fActions;
		}

		/// @brief Return the set of explored states.
		std::tuple<ExploredMap<State, Action, HashState, EqualState>, ExploredMap<State, Action, HashState, EqualState>> GetExploredStates() const
		{
			return std::make_tuple(m_fSearch.GetExploredStates(), m_rSearch.GetExploredStates());
		}

		/// @brief Return the optimal cost
		double GetOptimalCost() const
		{
			return m_fSearch.GetOptimalCost() + m_rSearch.GetOptimalCost();
		}

		/// @Brief Return the state-propagator used by the algorithm.
		std::tuple<const Ref<AStarStatePropagator<State, Action>>&, const Ref<AStarStatePropagator<State, Action>>&> GetStatePropagators() const
		{
			return std::make_tuple(m_fSearch.GetStatePropagator(), m_rSearch.GetStatePropagator());
		}
		/// @brief Return the heuristic used by the algorithm
		std::tuple<const Ref<AStarHeuristic<State>>&, const Ref<AStarHeuristic<State>>&> GetHeuristics() const
		{
			return std::make_tuple(m_fSearch.GetHeuristic(), m_rSearch.GetHeuristic());
		}

		/// @brief Initialize A* search.
		/// @param fPropagator Expand the a given node in the forward search.
		/// @param rPropagator Expand the a given node in the reverse search.
		/// @param fHeuristic Heuristic function to guide the forward search.
		/// @param rHeuristic Heuristic function to guide the reverse search.
		/// @note The following requirements must be met to ensure optimality of
		/// the solution:
		/// - Admissibility: the heuristic under-evaluate the path cost
		/// - Consistency (for graph search only): \f$h(u) - h(v) \leq c(u,v)\f$
		/// for all point \f$u\f$ and \f$v\f$ with \f$h\f$ the heuristic
		/// function and \f$c(u,v)\f$ the path cost from \f$u\f$ to \f$v\f$.
		/// - Consistent pair: for all pair of point \f$u\f$ and \f$v\f$, the
		/// sum of forward and  reverse heuristic must be constant: \f$h_f(u) +
		/// h_r(u) = h_f(v) + h_r(v)\f$ where \f$h_f\f$ and \f$h_r\f$ are the
		/// forward and backward heuristic functions respectively. If a pair of
		/// heuristic does not meet this condition, the pair defined by their
		/// average does and can be constructed from
		/// BidirectionalAStar::GetAverageHeuristicPair.
		bool Initialize(const Ref<AStarStatePropagator<State, Action>>& fPropagator, const Ref<AStarStatePropagator<State, Action>>& rPropagator,
			const Ref<AStarHeuristic<State>>& fHeuristic, const Ref<AStarHeuristic<State>>& rHeuristic)
		{
			if (!m_fSearch.Initialize(fPropagator, fHeuristic))
				return isInitialized = false;
			if (!m_rSearch.Initialize(rPropagator, rHeuristic))
				return isInitialized = false;

			m_fAverageHeuristic = dynamic_cast<AverageHeuristic<State>*>(m_fSearch.GetHeuristic().get());
			m_rAverageHeuristic = dynamic_cast<AverageHeuristic<State>*>(m_rSearch.GetHeuristic().get());

			return isInitialized = true;
		}

		/// @copydoc Planner::PathPlanner::SearchPath
		virtual Status SearchPath() override
		{
			if (!isInitialized) {
				PP_ERROR("The algorithm has not been initialized successfully.");
				return Status::Failure;
			}

			if (m_fAverageHeuristic && m_rAverageHeuristic) {
				m_fAverageHeuristic->Update(this->m_init, this->m_goal);
				m_rAverageHeuristic->Update(this->m_goal, this->m_init);
			}

			m_fSearch.SetInitState(this->m_init);
			m_rSearch.SetInitState(this->m_goal);
			m_fSearch.SetGoalState(this->m_goal);
			m_rSearch.SetGoalState(this->m_init);

			m_fSearch.InitializeSearch();
			m_rSearch.InitializeSearch();

			const double costOffset = m_fSearch.m_heuristic->GetHeuristicValue(this->m_goal) + m_rSearch.m_heuristic->GetHeuristicValue(this->m_goal);
			PP_INFO("Cost offset: {}", costOffset);

			double bestCost = std::numeric_limits<double>::infinity();
			while (!m_fSearch.m_frontier.Empty() && !m_rSearch.m_frontier.Empty()) {
				// TODO Can we avoid expanding node in one direction if they have already been expanded in the other?
				// Forward search
				auto fNode = m_fSearch.m_frontier.Pop();
				m_fSearch.Expand(fNode);
				FindIntersection(fNode, m_fSearch, m_rSearch, bestCost);

				// Reverse search
				auto rNode = m_rSearch.m_frontier.Pop();
				m_rSearch.Expand(rNode);
				FindIntersection(rNode, m_rSearch, m_fSearch, bestCost);

				// Check an intersection has been found
				if (m_fSearch.m_solutionNode && m_rSearch.m_solutionNode) {
					if (m_fSearch.m_frontier.Empty() || m_rSearch.m_frontier.Empty())
						return Status::Success;

					// TODO FIXME need to check what stopping condition to use
// 					double fTopCost = (*m_fSearch.m_frontier.Top())->pathCost;
// 					double rTopCost = (*m_rSearch.m_frontier.Top())->pathCost;
					double fTopCost = (*m_fSearch.m_frontier.Top())->totalCost;
					double rTopCost = (*m_rSearch.m_frontier.Top())->totalCost;
					// TODO try replacing last 4 lines by using fNode and rNode directly

					// Check the stopping criterion
					// TODO allow to change stopping condition more easily
// 					if (fTopCost + rTopCost >= bestCost + costOffset)
// 						return Status::Success;
					if (fTopCost + rTopCost >= bestCost + costOffset)
						return Status::Success;
// 					if (std::max(fTopCost, rTopCost) >= bestCost)
// 						return Status::Success;
				}
			}
			return Status::Failure;
		}

	protected:
		inline virtual void FindIntersection(AStarNode<State, Action>* nodeA, UnidirectionalAStar& searchA, UnidirectionalAStar& searchB, double& bestPathCost)
		{
			auto it = searchB.m_explored.find((*nodeA)->state);
			if (it != searchB.m_explored.end()) {
				auto nodeB = it->second;

				// Check the path of the new candidate path
				double candidatePathCost = (*nodeA)->pathCost + (*nodeB)->pathCost;
				if (candidatePathCost < bestPathCost) {
					// This is the new best path
					bestPathCost = candidatePathCost;
					searchA.m_solutionNode = nodeA;
					searchB.m_solutionNode = nodeB;
				}
			}
		}

	protected:
		UnidirectionalAStar m_fSearch, m_rSearch;
		AverageHeuristic<State>*m_fAverageHeuristic = nullptr, *m_rAverageHeuristic = nullptr;

	private:
		bool isInitialized = false;
	};

}
