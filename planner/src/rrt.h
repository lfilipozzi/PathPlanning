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
			double stepSize = 0.1;
			double optimalSolutionTolerance = 1;
		};

	public:
		/**
		 * @brief Constructor.
		 * @param stateSpace
		 * @param validator
		 * @param hash
		 */
		RRT(Scope<StateSpace<Vertex>>&& stateSpace, Scope<StateValidator<Vertex>>&& validator) :
			PathPlanner<Vertex>(std::move(stateSpace), std::move(validator)) {};
		virtual ~RRT() = default;

		Parameters& GetParameters() { return m_parameters; }
		const Parameters& GetParameters() const { return m_parameters; }
		void SetParameters(const Parameters& params) { m_parameters = params; }

		virtual void SearchPath() override
		{
			m_tree.CreateRootNode(this->m_init);

			for (unsigned int k = 0; k < m_parameters.maxIteration; k++) {
				// Create a random configuration
				Vertex randomState = this->m_stateSpace->CreateRandomState();
				// Find the nearest node in the tree
				auto nearNode = m_tree.GetNearestNode(randomState);
				if (!nearNode)
					continue;
				// Extend the tree toward randomState by creating a valid new node with obstacle-free path
				auto newState = this->m_stateSpace->CreateIncrementalState(nearNode->GetState(), randomState, m_parameters.stepSize);
				if (!this->m_stateValidator->ValidateState(newState))
					continue;
				if (!this->m_stateValidator->ValidateTransition(nearNode->GetState(), newState))
					continue;
				auto newNode = m_tree.Extend(newState, nearNode);

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
