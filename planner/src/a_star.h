#pragma once

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include "path_planner.h"
#include "tree.h"

namespace Planner {

	template <typename Vertex, class Hash = std::hash<Vertex>, typename VertexType = double>
	class AStar : public PathPlanner<Vertex> {
	public:

		/**
		* @brief Tunable parameters of the RRT algorithm.
		*/
		struct Parameters {
			unsigned int maxIteration = 100;
			double optimalSolutionTolerance = 1;
		};

		/**
		 * @brief Node metadata used by RRT star.
		 */
		struct NodeMetadata {
			double pathCost = 0;
			double totalCost = 0;
		};
		using Node = GenericNode<Vertex, NodeMetadata>;

		// TODO implement this function
		std::vector<Vertex> GetNeighbors(Vertex state)
		{
			return std::vector<Vertex>();
		}
		double EvalHeuristic(Vertex from, Vertex to) { return 0.0; }
		// TODO implement this function

		/**
		 * @brief Implementation of a priority queue storing Node* ranked 
		 * according to their totalPath cost. Two nodes are considered equal if 
		 * they refer to the same state.
		 * @details The implementation combines a std::vector (for cache-
		 * friendliness) that is continuously sorted and a 
		 * std::unordered_set for efficient access.
		 */
		class PriorityQueue {
		public:
			using T = Node*;
			using V = Vertex;

			bool Empty() const { return m_vec.empty(); }

			void Push(const T& value)
			{
				auto[it, success] = m_map.insert({ value->GetState(), value });
				if (success) {
					auto it = FindFirstBiggerElement(value);
					m_vec.insert(it, value);
				}
			}

			void Remove(const T& value)
			{
				size_t removed = m_map.erase(value->GetState());
				if (removed > 0) {
					 auto it = FindFirstBiggerElement(value);
					 m_vec.erase(std::prev(it));
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
			static bool Compare(const T& a, const T& b)
			{
				if (!a || !b)
					return false;
				return a->meta.totalCost < b->meta.totalCost;
			}

			typename std::vector<T>::iterator FindFirstBiggerElement(const T& value)
			{
				for (auto it = m_vec.begin(); it != m_vec.end(); it++) {
					// TODO replace by dichotomic search
					if (Compare(*it, value))
						return it;
				}
				return m_vec.end();
			}

		private:
			std::vector<T> m_vec;
			std::unordered_map<V, T, Hash> m_map;
		};

	public:
		/**
		 * @brief Constructor.
		 * @param stateSpace
		 */
		AStar(Scope<StateSpace<Vertex>>&& stateSpace) :
			PathPlanner<Vertex>(std::move(stateSpace)) {};
		virtual ~AStar() = default;

		Parameters& GetParameters() { return m_parameters; }
		const Parameters& GetParameters() const { return m_parameters; }
		void SetParameters(const Parameters& params) { m_parameters = params; }

		virtual void SearchPath() override
		{
			m_rootNode = makeScope<Node>(this->m_init);

			PriorityQueue frontier;
			frontier.Push(m_rootNode.get());

			std::unordered_set<Vertex> explored;
			explored.insert(m_rootNode->GetState());

			while (true) {
				if (frontier.Empty()) {
					// TODO Add error (either throw error or return enum)
					break;
				}
				
				// Pop lowest-cost node of the frontier
				auto node = frontier.Poll();
				
				// Check if node is a solution
				if (this->m_stateSpace->ComputeDistance(node->GetState(), this->m_goal) < m_parameters.optimalSolutionTolerance) {
					m_solutionNode = node;
					break;
				}
				
				explored.insert(node->GetState());
				
				for (Vertex childVertex : GetNeighbors(node->GetState())) {
					// Create the child node
					Scope<Node> childScope = makeScope<Node>(childVertex);
					Node* child = childScope.get();
					auto [transitionCost, transitionCollisionFree] = this->m_stateSpace->SteerExactly(node->GetState(), child->GetState());
					child->meta.pathCost = node->meta.pathCost + transitionCost;
					child->meta.totalCost = child->meta.pathCost + EvalHeuristic(child->GetState(), this->m_goal);

					// Check if the child node is in the frontier or explored set
					Node*const* inFrontier = frontier.Find(child->GetState());
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
							(*inFrontier)->GetParent()->ReplaceChild(*inFrontier, std::move(childScope), false);
							// Replace the node in the frontier by child
							frontier.Remove(*inFrontier);
							frontier.Push(child);
						}
					}
				}
			}
		};

		virtual std::vector<Vertex> GetPath() override
		{
			std::vector<Vertex> path;

			auto node = m_solutionNode;
			if (!node)
				return path;

			unsigned int depth = m_solutionNode->GetDepth();
			path.resize(depth);
			for (auto it = path.rbegin(); it != path.rend(); it++) {
				*it = node->GetState();
				node = node->GetParent();
			}
			return path;
		}

	private:
		Parameters m_parameters;

		Scope<Node> m_rootNode;
		Node* m_solutionNode = nullptr;
	};
}
