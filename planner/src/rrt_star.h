#pragma once

#include "path_planner.h"

#include "tree.h"

namespace Planner {

	template <typename T, unsigned int Dimensions, class Hash = std::hash<T>>
	class RRTStar : public PathPlanner<T> {
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
		RRTStar(Scope<StateSpace<T>>&& stateSpace, Scope<StateValidator<T>>&& validator) :
			PathPlanner<T>(std::move(stateSpace), std::move(validator)) {};
		virtual ~RRTStar() = default;

		Parameters& GetParameters() { return m_parameters; }
		const Parameters& GetParameters() const { return m_parameters; }
		void SetParameters(const Parameters& params) { m_parameters = params; }

		virtual void SearchPath() override
		{
			m_tree.CreateRootNode(this->m_init);

			for (unsigned int k = 0; k < m_parameters.maxIteration; k++) {
				// Create a random configuration
				T randomState = this->m_stateSpace->CreateRandomState();
				// Check if a collision-free path between the state and the tree exists
				auto [newState, nearNode] = GetCollisionFreePath(randomState);
				if (!newState)
					continue;
			}
		}
		
		virtual std::vector<T> GetPath()
		{
			std::vector<T> path;

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

		Tree<T, Dimensions, Hash> m_tree;
		typename Tree<T, Dimensions, Hash>::Node* m_solutionNode = nullptr;
	};
}
