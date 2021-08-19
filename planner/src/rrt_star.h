#pragma once

#include "path_planner.h"
#include "tree.h"

namespace Planner {

	template <typename Vertex, unsigned int Dimensions, class Hash = std::hash<Vertex>, typename VertexType = double>
	class RRTStar : public PathPlanner<Vertex> {
	public:
		/**
		* @brief Tunable parameters of the RRT algorithm.
		*/
		struct Parameters {
			unsigned int maxIteration = 100;
			double optimalSolutionTolerance = 1;
		};

	public:
		/**
		 * @brief Constructor.
		 * @param stateSpace
		 * @param hash
		 */
		RRTStar(Scope<StateSpace<Vertex>>&& stateSpace) :
			PathPlanner<Vertex>(std::move(stateSpace)) {};
		virtual ~RRTStar() = default;

		Parameters& GetParameters() { return m_parameters; }
		const Parameters& GetParameters() const { return m_parameters; }
		void SetParameters(const Parameters& params) { m_parameters = params; }

		virtual void SearchPath() override
		{
			m_tree.CreateRootNode(this->m_init);

			for (unsigned int k = 0; k < m_parameters.maxIteration; k++) {
				// Create a random configuration
				Vertex randomState = this->m_stateSpace->Sample();
				// Find the nearest node in the tree
				auto nearestNode = m_tree.GetNearestNode(randomState);
				if (!nearestNode)
					continue;
				// Find a new vertex from nearestNode towards randomState and check if the transition is collision-free
				auto newState = this->m_stateSpace->SteerTowards(nearestNode->GetState(), randomState);
				if (!this->m_stateSpace->IsTransitionCollisionFree(nearestNode->GetState(), newState))
					continue;

				// Find k nearest neighbor of newState where k is logarithmic in the size of the tree
				unsigned int nn = log(m_tree.GetSize());
				auto nearNodes = m_tree.GetNearestNodes(newState, nn);
				// Find the best parent for newNode
				auto bestParentNode = nearestNode;
				[[maybe_unused]] auto [transitionCost, transitionCollisionFree] = this->m_stateSpace->SteerExactly(nearestNode->GetState(), newState);
				double bestCost = nearestNode->GetCostFromGoal() + transitionCost;
				for (auto node : nearNodes) {
					auto [transitionCost, transitionCollisionFree] = this->m_stateSpace->SteerExactly(node->GetState(), newState);
					double cost = node->GetCostFromGoal() + transitionCost;
					if (cost < bestCost && transitionCollisionFree) {
						bestParentNode = node;
						bestCost = cost;
					}
				}
				auto newNode = m_tree.Extend(newState, bestParentNode);
				newNode->SetCostFromGoal(bestCost);

				// Reparent the nearest neighbor if necessary
				for (auto node : nearNodes) {
					auto [transitionCost, transitionCollisionFree] = this->m_stateSpace->SteerExactly(newNode->GetState(), node->GetState());
					double cost = newNode->GetCostFromGoal() + transitionCost;
					if (cost < node->GetCostFromGoal() && transitionCollisionFree) {
						m_tree.Reparent(node, newNode);
						node->SetCostFromGoal(cost);
					}
				}

				// Check solution
				if (this->m_stateSpace->ComputeDistance(newState, this->m_goal) < m_parameters.optimalSolutionTolerance) {
					m_solutionNode = newNode;
					break;
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
			path.resize(depth);
			for (auto it = path.rbegin(); it != path.rend(); it++) {
				*it = node->GetState();
				node = node->GetParent();
			}
			return path;
		}

		/**
		 * @brief Clear the tree.
		 */
		void Clear()
		{
			m_tree.Clear();
			m_solutionNode = nullptr;
		}

	private:
		Parameters m_parameters;

		Tree<Vertex, Dimensions, Hash, VertexType> m_tree;
		typename Tree<Vertex, Dimensions, Hash, VertexType>::Node* m_solutionNode = nullptr;
	};
}
