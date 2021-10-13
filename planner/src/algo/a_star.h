#pragma once

#include "algo/path_planner.h"
#include "state_space/state_space.h"
#include "utils/tree.h"
#include "core/hash.h"

#include <queue>
#include <unordered_set>
#include <unordered_map>

namespace Planner {

	/**
	 * @brief Interface to sample the configuration space as required by A* 
	 * algorithms.
	 */
	template <typename State>
	class AStarStateSpace {
	public:
		AStarStateSpace() = default;
		virtual ~AStarStateSpace() = default;

		/**
		* @brief Returns a list of all neighbor positions and the transition 
		* cost. The transition from the current state to the neighbor state must
		* be valid.
		* @return A list of tuple all neighboring positions and their associated
		* transition cost.
		*/
		virtual std::vector<std::tuple<State, double>> GetNeighborStates(const State& state) = 0;
	};

	/**
	 * @brief Interface for an A* heuristic function.
	 */
	template <typename State>
	class AStarHeuristic {
	public:
		AStarHeuristic() = default;
		virtual ~AStarHeuristic() = default;

		/**
		 * @brief Heuristic function.
		 */
		virtual double GetHeuristicValue(const State& from, const State& to) = 0;
	};

	/**
	 * @brief Combine several admissible heuristics into an improved admissible 
	 * one.
	 */
	template <typename State>
	class AStarCombinedHeuristic : public AStarHeuristic<State> {
	public:
		AStarCombinedHeuristic() = default;

		void Add(const Ref<AStarHeuristic<State>>& heuristic)
		{
			m_heuristics.push_back(heuristic);
		}

		template <typename... Args>
		void Add(const Ref<AStarHeuristic<State>>& heuristic, Args&&... args)
		{
			m_heuristics.push_back(heuristic);
			Add(std::forward<Args>(args)...);
		}

		virtual double GetHeuristicValue(const State& from, const State& to) override
		{
			double value = -std::numeric_limits<double>::infinity();
			for (auto& h : m_heuristics) {
				value = std::max(value, h->GetHeuristicValue(from, to));
			}
			return value;
		}

	private:
		std::vector<Ref<AStarHeuristic<State>>> m_heuristics;
	};

	/**
	 * @brief Implementation of the A* algorithm.
	 */
	template <typename State, typename Hash = std::hash<State>>
	class AStar : public PathPlanner<State> {
		static_assert(std::is_copy_constructible<State>::value);

	private:
		/**
		 * @brief Node metadata used by A* star.
		 */
		struct NodeMetadata {
			double pathCost = 0;
			double totalCost = 0;
		};
		using Node = GenericNode<State, NodeMetadata>;

		/**
		 * @brief Implementation of a priority queue storing Node* ranked 
		 * according to their totalPath cost. Two nodes are considered equal if 
		 * they refer to the same state.
		 * @details The implementation combines a std::vector (for cache-
		 * friendliness) that is continuously sorted and a 
		 * std::unordered_set for efficient access. The std::vector is sorted in
		 * descending order of path-cost.
		 */
		class PriorityQueue {
			using T = Node*;

		public:
			PriorityQueue() = default;

			size_t Size() const { return m_vec.size(); }

			bool Empty() const { return m_vec.empty(); }

			void Push(const T& node)
			{
				State& state = node->GetState();
				auto [it, success] = m_map.insert({ state, node });
				if (success) {
					auto it = FindFirstElementStrictlySmallerThan(node);
					m_vec.insert(it, node);
				}
			}

			void Remove(const T& node)
			{
				State& state = node->GetState();
				size_t removed = m_map.erase(state);
				if (removed > 0) {
					auto begin = FindLastElementBiggerThan(node);
					auto end = m_vec.end();

					bool erased = false;
					for (auto it = begin; it != end; it++) {
						if (*it == node) {
							m_vec.erase(it);
							erased = true;
							break;
						}
					}
					PP_ASSERT(erased, "Element has not been erased");
				}
			}

			T Poll()
			{
				// Access last element
				T& node = m_vec.back();
				State& state = node->GetState();
				// Remove it
				m_vec.pop_back();
				m_map.erase(state);
				return node;
			}

			const T* Find(const State& state) const
			{
				auto search = m_map.find(state);
				if (search != m_map.end())
					return &(search->second);
				return nullptr;
			}

		private:
			/**
			 * @brief Return an iterator to the first element in the vector that
			 * is strictly smaller than @value.
			 * @details Implemented using a binary search.
			 * @return An iterator to the element.
			 */
			typename std::vector<T>::iterator FindFirstElementStrictlySmallerThan(const T& value)
			{
				unsigned int index = 0;
				{
					int begin = 0;
					int end = m_vec.size() - 1;
					while (begin <= end) {
						int mid = begin + (end - begin) / 2;

						bool isLeftBigger = m_vec[mid]->meta.totalCost >= value->meta.totalCost;
						bool isRightSmaller = mid + 1 < m_vec.size() ? value->meta.totalCost > m_vec[mid + 1]->meta.totalCost : true;

						if (isLeftBigger && isRightSmaller) {
							index = mid + 1;
							break;
						}
						if (!isLeftBigger)
							end = mid - 1;
						if (!isRightSmaller)
							begin = mid + 1;
					}
				}

				auto it = m_vec.begin();
				std::advance(it, index);
				return it;
			}

			/**
			 * @brief Return an iterator to the first element in the vector that
			 * is bigger than or equal to @value.
			 * @details Implemented using a binary search.
			 * @return An iterator to the element.
			 */
			typename std::vector<T>::iterator FindLastElementBiggerThan(const T& value)
			{
				unsigned int index = 0;
				{
					int begin = 0;
					int end = m_vec.size() - 1;
					while (begin <= end) {
						int mid = begin + (end - begin) / 2;

						bool isLeftBigger = m_vec[mid]->meta.totalCost > value->meta.totalCost;
						bool isRightSmaller = mid + 1 < m_vec.size() ? value->meta.totalCost >= m_vec[mid + 1]->meta.totalCost : true;

						if (isLeftBigger && isRightSmaller) {
							index = mid;
							break;
						}
						if (!isLeftBigger)
							end = mid - 1;
						if (!isRightSmaller)
							begin = mid + 1;
					}
				}

				auto it = m_vec.begin();
				std::advance(it, index);
				return it;
			}

		private:
			std::vector<T> m_vec;
			std::unordered_map<State, T, Hash> m_map;
		};

	public:
		/**
		 * @brief Constructor.
		 * @param stateSpace The configuration space.
		 */
		AStar(const Ref<AStarStateSpace<State>>& stateSpace, const Ref<AStarHeuristic<State>>& heuristic) :
			m_stateSpace(stateSpace), m_heuristic(heuristic) {};
		virtual ~AStar() = default;

		virtual Status SearchPath() override
		{
			m_rootNode = makeScope<Node>(this->m_init);

			PriorityQueue frontier;
			frontier.Push(m_rootNode.get());

			std::unordered_set<State, Hash> explored;
			explored.insert(m_rootNode->GetState());

			while (true) {
				if (frontier.Empty()) {
					return Status::Failure;
				}
				PP_INFO("Open: {0}, close: {1}", frontier.Size(), explored.size());

				// Pop lowest-cost node of the frontier
				auto node = frontier.Poll();

				// Check if node is a solution
				if (node->GetState() == this->m_goal) {
					m_solutionNode = node;
					return Status::Success;
				}

				explored.insert(node->GetState());

				for (auto [childState, transitionCost] : m_stateSpace->GetNeighborStates(node->GetState())) {
					// Create the child node
					Scope<Node> childScope = makeScope<Node>(childState);
					Node* child = childScope.get();
					child->meta.pathCost = node->meta.pathCost + transitionCost;
					child->meta.totalCost = child->meta.pathCost + m_heuristic->GetHeuristicValue(child->GetState(), this->m_goal);

					// Check if the child node is in the frontier or explored set
					Node* const* inFrontier = frontier.Find(child->GetState());
					bool inExplored = explored.find(child->GetState()) != explored.end();
					if (!inFrontier && !inExplored) {
						// Add child to frontier and to the tree
						frontier.Push(child);
						node->AddChild(std::move(childScope));
					} else if (inFrontier) {
						// Check if the node in frontier has a higher cost than the
						// current path, and if so replace it by child
						if ((*inFrontier)->meta.totalCost > child->meta.totalCost) {
							// Replace the node *inFrontier in the tree by the newly find better node child.
							// If a node is in the frontier, it does not have a child yet, so no need to
							// adopt grandchildren
							// Remark: no need to check whether the pointer (*inFrontier)->GetParent() is
							// nullptr, since we should never enter this condition for the root node of the
							// tree
							auto previousChildScope = (*inFrontier)->GetParent()->ReplaceChild(*inFrontier, std::move(childScope), false);
							// Replace the node in the frontier by child
							frontier.Remove(previousChildScope.get());
							frontier.Push(child);
						}
					}
				}
			}
		}

		virtual std::vector<State> GetPath() override
		{
			std::vector<State> path;

			auto node = m_solutionNode;
			if (!node)
				return path;

			unsigned int depth = m_solutionNode->GetDepth();
			path.resize(depth + 1);
			for (auto it = path.rbegin(); it != path.rend(); it++) {
				*it = node->GetState();
				node = node->GetParent();
			}
			return path;
		}

		Ref<AStarStateSpace<State>> GetStateSpace() const { return m_stateSpace; }

	private:
		Ref<AStarStateSpace<State>> m_stateSpace;
		Ref<AStarHeuristic<State>> m_heuristic;

		Scope<Node> m_rootNode;
		Node* m_solutionNode = nullptr;
	};
}
