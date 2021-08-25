#pragma once

#include "path_planner.h"
#include "tree.h"

namespace Planner {

	/**
	* @brief Implementation of the Rapidly-exploring Random Trees (RRT).
	*/
	template <typename Vertex, unsigned int Dimensions, class Hash = std::hash<Vertex>, typename VertexType = double>
	class RRT : public PathPlanner<Vertex> {
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
		 */
		RRT(Scope<RRTStateSpace<Vertex>>&& stateSpace) :
			m_stateSpace(std::move(stateSpace)) {};
		virtual ~RRT() = default;

		Parameters& GetParameters() { return m_parameters; }
		const Parameters& GetParameters() const { return m_parameters; }
		void SetParameters(const Parameters& params) { m_parameters = params; }

		virtual void SearchPath() override
		{
			m_tree.CreateRootNode(this->m_init);

			for (unsigned int k = 0; k < m_parameters.maxIteration; k++) {
				// Create a random configuration
				Vertex randomState = m_stateSpace->Sample();
				// Find the nearest node in the tree
				auto nearestNode = m_tree.GetNearestNode(randomState);
				if (!nearestNode)
					continue;
				// Extend the tree toward randomState by creating a valid new node with obstacle-free path
				auto newState = m_stateSpace->SteerTowards(nearestNode->GetState(), randomState);
				if (!m_stateSpace->IsTransitionCollisionFree(nearestNode->GetState(), newState))
					continue;
				auto newNode = m_tree.Extend(newState, nearestNode);

				// Check solution
				if (m_stateSpace->ComputeDistance(newState, this->m_goal) < m_parameters.optimalSolutionTolerance) {
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
			path.resize(depth + 1);
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
		Scope<RRTStateSpace<Vertex>> m_stateSpace;

		Tree<Vertex, Dimensions, VoidClass, Hash, VertexType> m_tree;
		typename Tree<Vertex, Dimensions, VoidClass, Hash, VertexType>::Node* m_solutionNode = nullptr;
	};
}
