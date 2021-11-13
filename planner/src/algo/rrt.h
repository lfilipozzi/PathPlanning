#pragma once

#include "algo/path_planner.h"
#include "state_space/state_space.h"
#include "utils/tree.h"

namespace Planner {

	/// @brief Interface to sample the configuration space as required by RRT
	/// algorithms.
	template <typename Vertex>
	class RRTStateSpace {
	public:
		virtual ~RRTStateSpace() = default;

		// TODO FIXME Use StateSpace and StateValidator to implement those functions in RRT and delete RRTStateSpace
		/// @brief Calculate the distance between two states
		/// @param from The start state.
		/// @param to The end state.
		/// @return The distance between the states
		virtual double ComputeDistance(const Vertex& from, const Vertex& to) const = 0;

		/// @brief Check if there exist a valid transition between two states.
		/// @param from The initial state.
		/// @param to The destination state.
		/// @return True if the transition is valid, false otherwise.
		virtual bool IsTransitionCollisionFree(const Vertex& from, const Vertex& to) = 0;

		/// @brief Generate a random state within the configuration space.
		/// @details The sample must not be inside an obstacle.
		/// @return A random state.
		virtual Vertex Sample() = 0;

		/// @brief Construct a new state by moving an incremental distance
		/// from @source towards @target.
		/// @details The path is not assumed to bring exactly to target but it
		/// must evolve towards it.
		/// @return The new state.
		virtual Vertex SteerTowards(const Vertex& source, const Vertex& target) = 0;
	};

	/// @brief Tunable parameters of the RRT algorithm.
	struct RRTParameters {
		unsigned int maxIteration = 100;
		double optimalSolutionTolerance = 1;
	};

	/// @brief Implementation of the Rapidly-exploring Random Trees (RRT).
	template <typename Vertex, unsigned int Dimensions, class Hash = std::hash<Vertex>, typename VertexType = double>
	class RRT : public PathPlanner<Vertex> {
	public:
		/// @brief Constructor.
		/// @param stateSpace
		RRT(const Ref<RRTStateSpace<Vertex>>& stateSpace) :
			m_stateSpace(stateSpace) {};
		virtual ~RRT() = default;

		RRTParameters GetParameters() { return m_parameters; }
		const RRTParameters& GetParameters() const { return m_parameters; }
		void SetParameters(const RRTParameters& params) { m_parameters = params; }

		virtual Status SearchPath() override
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
					return Status::Success;
				}
			}

			return Status::Failure;
		}

		virtual std::vector<Vertex> GetPath() const override
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

		void Clear()
		{
			m_tree.Clear();
			m_solutionNode = nullptr;
		}

	private:
		RRTParameters m_parameters;
		Ref<RRTStateSpace<Vertex>> m_stateSpace;

		Tree<Vertex, Dimensions, VoidClass, Hash, VertexType> m_tree;
		typename Tree<Vertex, Dimensions, VoidClass, Hash, VertexType>::Node* m_solutionNode = nullptr;
	};
}
