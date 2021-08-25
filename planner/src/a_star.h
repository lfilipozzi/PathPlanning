#pragma once

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include "path_planner.h"
#include "state_space.h"
#include "tree.h"

namespace Planner {

	template <typename Vertex, class Hash = std::hash<Vertex>, typename VertexType = double>
	class AStar : public PathPlanner<Vertex> {
	public:
		/**
		* @brief Tunable parameters of the A* algorithm.
		*/
		struct Parameters {
			double optimalSolutionTolerance = 0.05;
		};

	private:
		/**
		 * @brief Node metadata used by A* star.
		 */
		struct NodeMetadata {
			double pathCost = 0;
			double totalCost = 0;
		};
		using Node = GenericNode<Vertex, NodeMetadata>;

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
		public:
			using T = Node*;
			using V = Vertex;

			bool Empty() const { return m_vec.empty(); }

			void Push(const T& value)
			{
				auto [it, success] = m_map.insert({ value->GetState(), value });
				if (success) {
					auto it = FindFirstElementStrictlySmallerThan(value);
					m_vec.insert(it, value);
				}
			}

			void Remove(const T& value)
			{
				size_t removed = m_map.erase(value->GetState());
				if (removed > 0) {
					auto begin = FindLastElementBiggerThan(value);
					auto end = m_vec.end();

					bool erased = false;
					for (auto it = begin; it != end; it++) {
						if (*it == value) {
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
				T elmt = m_vec.back();
				// Remove it
				m_vec.pop_back();
				m_map.erase(elmt->GetState());
				return elmt;
			}

			const T* Find(const V& value) const
			{
				auto search = m_map.find(value);
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
			std::unordered_map<V, T, Hash> m_map;
		};

	public:
		/**
		 * @brief Constructor.
		 * @param stateSpace The configuration space.
		 * @param heuristic The heuristic used by the algorithm. Its prototype 
		 * is:
		 * <code>double Heuristic (const Vertex& from, const Vertex& to)</code>
		 * where `from' is the origin state and `to' is the destination state. 
		 */
		AStar(const Ref<AStarStateSpace<Vertex>>& stateSpace, std::function<double(const Vertex&, const Vertex&)> heuristicFcn) :
			m_stateSpace(stateSpace), m_heuristicFcn(heuristicFcn) {};
		virtual ~AStar() = default;

		Parameters& GetParameters() { return m_parameters; }
		const Parameters& GetParameters() const { return m_parameters; }
		void SetParameters(const Parameters& params) { m_parameters = params; }

		virtual Status SearchPath() override
		{
			m_rootNode = makeScope<Node>(this->m_init);

			PriorityQueue frontier;
			frontier.Push(m_rootNode.get());

			std::unordered_set<Vertex> explored;
			explored.insert(m_rootNode->GetState());

			while (true) {
				if (frontier.Empty()) {
					return Status::Failure;
				}

				// Pop lowest-cost node of the frontier
				auto node = frontier.Poll();

				// Check if node is a solution
				if (m_stateSpace->ComputeDistance(node->GetState(), this->m_goal) < m_parameters.optimalSolutionTolerance) {
					m_solutionNode = node;
					return Status::Success;
				}

				explored.insert(node->GetState());

				for (Vertex childVertex : m_stateSpace->GetNeighborStates(node->GetState())) {
					// Create the child node
					Scope<Node> childScope = makeScope<Node>(childVertex);
					Node* child = childScope.get();
					auto [transitionCost, transitionCollisionFree] = m_stateSpace->SteerExactly(node->GetState(), child->GetState());
					child->meta.pathCost = node->meta.pathCost + transitionCost;
					child->meta.totalCost = child->meta.pathCost + m_heuristicFcn(child->GetState(), this->m_goal);

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

		virtual std::vector<Vertex> GetPath() override
		{
			std::vector<Vertex> path;

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

	private:
		Parameters m_parameters;
		Ref<AStarStateSpace<Vertex>> m_stateSpace;
		std::function<double(const Vertex&, const Vertex&)> m_heuristicFcn;

		Scope<Node> m_rootNode;
		Node* m_solutionNode = nullptr;
	};
}
