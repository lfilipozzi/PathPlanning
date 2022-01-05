#pragma once

#include "core/base.h"
#include "algo/path_planner.h"
#include "algo/a_star.h"
#include <unordered_map>

namespace Planner {

	/// @brief Implement a priority list which allows unique membership. It
	/// combines the capabilities of a priority queue and a hash table.
	/// Elements are stored in two sorted lists: waiting storing element with
	/// total cost greater than lbCost ans in descending total cost order, and 
	/// ready storing other elements in descending path cost order.
	/// @details The list is sorted by increasing order. A user-provided Compare
	/// can be supplied to change the ordering, e.g. using std::greater would
	/// cause the smallest element to appear as the top.
	template <
		typename T,
		typename CompareGCost,
		typename CompareFCost,
		typename Hash,
		typename KeyEqual,
		typename Container = std::vector<T>>
	class NBSFrontierImpl {
	public:
		NBSFrontierImpl() = default;
		NBSFrontierImpl(const CompareGCost& compareReady, const CompareFCost& compareWaiting) :
			m_compareReady(compareReady), m_compareWaiting(compareWaiting) { }
		NBSFrontierImpl(const CompareGCost& compareReady, const CompareFCost& compareWaiting, size_t bucketCount, const Hash& hash, const KeyEqual& equal) :
			m_compareReady(compareReady), m_compareWaiting(compareWaiting), m_set(bucketCount, hash, equal) { }

		/// @brief Return the number of elements in the container.
		size_t Size() const { return m_waiting.size() + m_ready.size(); }
		size_t SizeWaiting() const { return m_waiting.size(); }
		size_t SizeReady() const { return m_ready.size(); }

		/// @brief Check whether the container is empty.
		bool Empty() const { return m_waiting.empty() && m_ready.empty(); }
		bool EmptyWaiting() const { return m_waiting.empty(); }
		bool EmptyReady() const { return m_ready.empty(); }

		/// @brief Empty the container.
		void Clear() noexcept
		{
			m_waiting.clear();
			m_ready.clear();
			m_set.clear();
		}

		/// @brief Insert an element in the container.
		/// @return Returns a pair consisting of a pointer to the inserted
		/// element (or to the element that prevented the insertion) and a bool
		/// value set to true if the insertion took place.
		std::pair<const T*, bool> Push(const T& elmt) { return FrontierImpl::Push(elmt, m_set, m_waiting, m_compareWaiting); }
		std::pair<const T*, bool> PushReady(const T& elmt) { return FrontierImpl::Push(elmt, m_set, m_ready, m_compareReady); }

		/// @brief Remove an element equal to @elmt from the container (if it exists).
		/// @return Number of elements removed (0 or 1).
		size_t Remove(const T& elmt)
		{
			// Find the elmt in the set
			auto setIt = m_set.find(elmt);
			if (setIt != m_set.end()) {
				bool erased = false;
				// Erase elmt in waiting
				// TODO use m_lbCost to find if in waiting or ready
				{
					erased = FrontierImpl::Remove(*setIt, m_waiting, m_compareWaiting, m_set.key_eq());
				}
				// Erase elmt in ready
				if (!erased) {
					erased = FrontierImpl::Remove(*setIt, m_ready, m_compareReady, m_set.key_eq());
				}
				PP_ASSERT(erased, "Element has not been erased");
				// Erase elmt in set
				m_set.erase(setIt);

				return 1;
			} else
				return 0;
		}

		/// @brief Access the last element of the container.
		const T& Top() const { return !m_ready.empty() ? m_ready.back() : m_waiting.back(); }
		const T& TopWaiting() const { return m_waiting.back(); }
		const T& TopReady() const { return m_ready.back(); }

		/// @brief Removes the last element of the container.
		/// @return The removed element.
		T Pop() { return !m_ready.empty() ? FrontierImpl::Pop<T>(m_set, m_ready) : FrontierImpl::Pop<T>(m_set, m_waiting); }
		T PopWaiting() { return FrontierImpl::Pop<T>(m_set, m_waiting); }
		T PopReady() { return FrontierImpl::Pop<T>(m_set, m_ready); }

		/// @brief Find the element in the container that is equal to @elmt.
		/// @return A pointer to the element, nullptr if the element does not exist.
		const T* Find(const T& elmt) const
		{
			auto search = m_set.find(elmt);
			if (search != m_set.end())
				return &(*search);
			return nullptr;
		}

	private:
		CompareFCost m_compareWaiting;
		CompareGCost m_compareReady;
		Container m_waiting;
		Container m_ready;
		std::unordered_set<T, Hash, KeyEqual> m_set;
	};

	/// @brief Comparator to sort A* node in descending g-cost order.
	template <typename State, typename Action>
	struct CompareAStarNodeGCost {
		bool operator()(AStarNode<State, Action>* lhs, AStarNode<State, Action>* rhs) const
		{
			return (*lhs)->totalCost > (*rhs)->totalCost;
		}
	};

	/// @brief Frontier for NBS.
	template <typename State, typename Action, typename HashState, typename EqualState>
	using NBSFrontier = NBSFrontierImpl<
		AStarNode<State, Action>*,
		CompareAStarNodeGCost<State, Action>,
		CompareAStarNodeFCost<State, Action>,
		HashAStarNode<State, Action, HashState>,
		EqualAStarNode<State, Action, EqualState>>;

	/// @brief Near-optimal Bidirectional Search.
	template <
		typename State,
		typename Action = NullAction,
		typename HashState = std::hash<State>,
		typename EqualState = std::equal_to<State>,
		bool GraphSearch = true,
		typename UnidirectionalSearch = AStar<State, Action, HashState, EqualState, GraphSearch, ExploredMap<State, Action, HashState, EqualState>, NBSFrontier<State, Action, HashState, EqualState>>>
	class NBS : public BidirectionalAStarBase<State, Action, HashState, EqualState, GraphSearch, UnidirectionalSearch> {
		static_assert(std::is_same<typename UnidirectionalSearch::Frontier, NBSFrontier<State, Action, HashState, EqualState>>::value,
			"Frontier container of the unidirectional search must be an NBSFrontier.");
		static_assert(std::is_same<typename UnidirectionalSearch::Explored, ExploredMap<State, Action, HashState, EqualState>>::value,
			"Explored container of the unidirectional search must be an ExploredMap.");

	public:
		NBS() = default;
		template <typename... Args>
		NBS(Args... args) :
			BidirectionalAStarBase<State, Action, HashState, EqualState, GraphSearch, UnidirectionalSearch>(std::forward<Args>(args)...) { }
		virtual ~NBS() = default;

		/// @copydoc Planner::BidirectionalAStarBase::Initialize
		/// @note The following requirements must be met to ensure optimality of
		/// the solution:
		/// - Admissibility: the heuristic under-evaluate the path cost
		/// - Consistency (for graph search only): \f$h(u) - h(v) \leq c(u,v)\f$
		/// for all point \f$u\f$ and \f$v\f$ with \f$h\f$ the heuristic
		/// function and \f$c(u,v)\f$ the path cost from \f$u\f$ to \f$v\f$.
		bool Initialize(const Ref<AStarStatePropagator<State, Action>>& fPropagator, const Ref<AStarStatePropagator<State, Action>>& rPropagator,
			const Ref<AStarHeuristic<State>>& fHeuristic, const Ref<AStarHeuristic<State>>& rHeuristic)
		{
			return BidirectionalAStarBase<State, Action, HashState, EqualState, GraphSearch, UnidirectionalSearch>::Initialize(fPropagator, rPropagator, fHeuristic, rHeuristic);
		}

		/// @copydoc Planner::PathPlanner::SearchPath
		virtual Status SearchPath() override
		{
			PP_PROFILE_FUNCTION();

			if (!this->InitializeSearch()) {
				return Status::Failure;
			}

			m_lbCost = 0;

			auto [fExplored, rExplored] = this->GetExploredSets();
			auto [fFrontier, rFrontier] = this->GetFrontiers();
			auto [fSolutionNode, rSolutionNode] = this->GetSolutionNodes();

			double optimalCost = std::numeric_limits<double>::infinity();
			while (!fFrontier.Empty() && !rFrontier.Empty()) {
				// Sort frontier and raise m_lbCost
				while (!fFrontier.EmptyWaiting() && (*fFrontier.TopWaiting())->totalCost < m_lbCost) {
					fFrontier.PushReady(fFrontier.PopWaiting());
				}
				while (!rFrontier.EmptyWaiting() && (*rFrontier.TopWaiting())->totalCost < m_lbCost) {
					rFrontier.PushReady(rFrontier.PopWaiting());
				}
				while (!fFrontier.EmptyReady() && !rFrontier.EmptyReady() && (*fFrontier.TopReady())->pathCost + (*rFrontier.TopReady())->pathCost > m_lbCost) {
					if ((*fFrontier.TopWaiting())->totalCost <= m_lbCost) {
						fFrontier.PushReady(fFrontier.PopWaiting());
						continue;
					}
					if ((*rFrontier.TopWaiting())->totalCost <= m_lbCost) {
						rFrontier.PushReady(rFrontier.PopWaiting());
						continue;
					}
					m_lbCost = std::min<double>({
						(*fFrontier.TopWaiting())->totalCost,
						(*rFrontier.TopWaiting())->totalCost,
						(*fFrontier.TopReady())->pathCost + (*rFrontier.TopReady())->pathCost,
					});
				}

				// Forward search
				auto fNode = fFrontier.Pop();
				this->ExpandForward(fNode);
				FindIntersection(fNode, fSolutionNode, rSolutionNode, rExplored, optimalCost);

				// Reverse search
				auto rNode = rFrontier.Pop();
				this->ExpandReverse(rNode);
				FindIntersection(rNode, rSolutionNode, fSolutionNode, fExplored, optimalCost);

				if (fSolutionNode && rSolutionNode) {// && (*fSolutionNode)->pathCost + (*rSolutionNode)->pathCost <= m_lbCost) {
				// if ((*fFrontier.Top())->pathCost + (*rFrontier.Top())->pathCost <= m_lbCost) {
					return Status::Success;
				}
			}
			return Status::Failure;
		}

	protected:
		inline virtual void FindIntersection(AStarNode<State, Action>* nodeA, typename UnidirectionalSearch::Node*& solutionNodeA, 
			typename UnidirectionalSearch::Node*& solutionNodeB, typename UnidirectionalSearch::Explored& exploredB, double& optimalPathCost)
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
		// Lower bound on lbmin = min{lb(u, v), u in U and v in V} where lb(u,v) = max(f(u), f(v), g(u) + g(v)).
		double m_lbCost;
	};

}
