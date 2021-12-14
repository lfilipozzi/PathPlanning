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
			m_toGoalHeuristic(toGoalHeuristic), m_toInitHeuristic(toInitHeuristic)
		{
			// Edge case: if the two heuristic given refer to the same object, the goal will not be updated correctly
			if (m_toGoalHeuristic.get() == m_toInitHeuristic.get())
				throw std::invalid_argument("The forward and reverse heuristic cannot refer to the same object.");
		}

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

	/// @brief Bidirectional A* search
	template <
		typename State,
		typename Action,
		typename HashState = std::hash<State>,
		typename EqualState = std::equal_to<State>,
		bool GraphSearch = true,
		typename UnidirectionalAStar = AStar<State, Action, HashState, EqualState, GraphSearch, ExploredMap<State, Action, HashState, EqualState>>>
	class BidirectionalAStar : public BidirectionalAStarBase<State, Action, HashState, EqualState, GraphSearch, UnidirectionalAStar> {
		static_assert(std::is_same<typename UnidirectionalAStar::Explored, ExploredMap<State, Action, HashState, EqualState>>::value,
			"Explored container of the unidirectional search must be an ExploredMap.");

	public:
		BidirectionalAStar() = default;
		template <typename... Args>
		BidirectionalAStar(Args... args) : 
			BidirectionalAStarBase<State, Action, HashState, EqualState, GraphSearch, UnidirectionalAStar>(std::forward<Args>(args)...) { }
		virtual ~BidirectionalAStar() = default;

		/// @brief Return a consistent pair of heuristic for bidirectional
		/// search.
		static std::tuple<Ref<AStarHeuristic<State>>, Ref<AStarHeuristic<State>>> GetAverageHeuristicPair(const Ref<AStarHeuristic<State>>& fHeuristic, const Ref<AStarHeuristic<State>>& rHeuristic)
		{
			auto fAverageHeuristic = makeRef<AverageHeuristic<State>>(fHeuristic, rHeuristic);
			auto rAverageHeuristic = makeRef<AverageHeuristic<State>>(rHeuristic, fHeuristic);
			return std::make_tuple(fAverageHeuristic, rAverageHeuristic);
		}

		/// @copydoc Planner::BidirectionalAStarBase::Initialize
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
			bool isInitialized = BidirectionalAStarBase<State, Action, HashState, EqualState, GraphSearch, UnidirectionalAStar>::Initialize(fPropagator, rPropagator, fHeuristic, rHeuristic);

			m_fAverageHeuristic = dynamic_cast<AverageHeuristic<State>*>(this->m_fSearch.GetHeuristic().get());
			m_rAverageHeuristic = dynamic_cast<AverageHeuristic<State>*>(this->m_rSearch.GetHeuristic().get());

			return isInitialized;
		}

		/// @copydoc Planner::PathPlanner::SearchPath
		virtual Status SearchPath() override
		{
			if (!this->InitializeSearch()) {
				return Status::Failure;
			}

			if (m_fAverageHeuristic && m_rAverageHeuristic) {
				m_fAverageHeuristic->Update(this->m_init, this->m_goal);
				m_rAverageHeuristic->Update(this->m_goal, this->m_init);
			}

			auto [fExplored, rExplored] = this->GetExploredSets();
			auto [fFrontier, rFrontier] = this->GetFrontiers();
			auto [fSolutionNode, rSolutionNode] = this->GetSolutionNodes();
			auto [fHeuristic, rHeuristic] = this->GetHeuristics();

			const double costOffset = fHeuristic->GetHeuristicValue(this->m_goal) + rHeuristic->GetHeuristicValue(this->m_goal);
			PP_INFO("Cost offset: {}", costOffset);

			double optimalCost = std::numeric_limits<double>::infinity();
			while (!fFrontier.Empty() && !rFrontier.Empty()) {
				// TODO Can we avoid expanding node in one direction if they have already been expanded in the other?
				// Forward search
				auto fNode = fFrontier.Pop();
				this->ExpandForward(fNode);
				FindIntersection(fNode, fSolutionNode, rSolutionNode, rExplored, optimalCost);

				// Reverse search
				auto rNode = rFrontier.Pop();
				this->ExpandReverse(rNode);
				FindIntersection(rNode, rSolutionNode, fSolutionNode, fExplored, optimalCost);

				// Check an intersection has been found
				if (fSolutionNode && rSolutionNode) {
					if (fFrontier.Empty() || rFrontier.Empty())
						return Status::Success;

					// TODO FIXME need to check what stopping condition to use
// 					double fTopCost = (*fFrontier.Top())->pathCost;
// 					double rTopCost = (*rFrontier.Top())->pathCost;
					double fTopCost = (*fFrontier.Top())->totalCost;
					double rTopCost = (*rFrontier.Top())->totalCost;

					// Check the stopping criterion
					// TODO allow to change stopping condition more easily
// 					if (fTopCost + rTopCost >= optimalCost + costOffset)
// 						return Status::Success;
					if (fTopCost + rTopCost >= optimalCost + costOffset)
						return Status::Success;
// 					if (std::max(fTopCost, rTopCost) >= bestCost)
// 						return Status::Success;
				}
			}
			return Status::Failure;
		}

	protected:
		inline virtual void FindIntersection(AStarNode<State, Action>* nodeA, typename UnidirectionalAStar::Node*& solutionNodeA, 
			typename UnidirectionalAStar::Node*& solutionNodeB, typename UnidirectionalAStar::Explored& exploredB, double& optimalPathCost)
		{
			// TODO should the intersection be in explored or in frontier
			auto it = exploredB.find((*nodeA)->state);
			if (it != exploredB.end()) {
				auto nodeB = it->second;

				// Check the path of the new candidate path
				double candidatePathCost = (*nodeA)->pathCost + (*nodeB)->pathCost;
				if (candidatePathCost < optimalPathCost) {
					// This is the new best path
					optimalPathCost = candidatePathCost;
					solutionNodeA = nodeA;
					solutionNodeB = nodeB;
				}
			}
		}

	protected:
		AverageHeuristic<State>*m_fAverageHeuristic = nullptr, *m_rAverageHeuristic = nullptr;
	};

}
