#pragma once

#include "algo/path_planner.h"
#include "state_space/state_space.h"
#include "utils/tree.h"

namespace Planner {

	/// @brief Interface to sample the configuration space as required by RRT
	/// algorithms.
	template <typename Vertex>
	class RRTStarStateSpace {
	public:
		virtual ~RRTStarStateSpace() = default;

		// TODO FIXME Use StateSpace and StateValidator to implement those functions in RRTStar and delete RRTStarStateSpace
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

		/// @brief Construct a path that connects the node @source to the node
		/// @target.
		/// @return A tuple containing the cost to go from the source to the
		/// target and a boolean indicating if the path is collision-free.
		virtual std::tuple<double, bool> SteerExactly(const Vertex& source, const Vertex& target) = 0;
	};

	/// @brief Tunable parameters of the RRT algorithm.
	struct RRTStarParameters {
		unsigned int maxIteration = 100;
		double optimalSolutionTolerance = 1;
	};

	template <typename Vertex, unsigned int Dimensions, class Hash = std::hash<Vertex>, typename VertexType = double>
	class RRTStar : public PathPlanner<Vertex> {
	private:
		/// @brief Node metadata used by RRT star.
		struct NodeMetadata {
			double costFromGoal = 0;
		};

	public:
		/// @brief Constructor.
		/// @param stateSpace
		RRTStar(const Ref<RRTStarStateSpace<Vertex>>& stateSpace) :
			m_stateSpace(stateSpace) {};
		virtual ~RRTStar() = default;

		RRTStarParameters GetParameters() { return m_parameters; }
		const RRTStarParameters& GetParameters() const { return m_parameters; }
		void SetParameters(const RRTStarParameters& params) { m_parameters = params; }

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
				// Find a new vertex from nearestNode towards randomState and check if the transition is collision-free
				auto newState = m_stateSpace->SteerTowards(nearestNode->GetState(), randomState);
				if (!m_stateSpace->IsTransitionCollisionFree(nearestNode->GetState(), newState))
					continue;

				// Find k nearest neighbor of newState where k is logarithmic in the size of the tree
				unsigned int nn = log(m_tree.GetSize());
				auto nearNodes = m_tree.GetNearestNodes(newState, nn);
				// Find the best parent for newNode
				auto bestParentNode = nearestNode;
				[[maybe_unused]] auto [transitionCost, transitionCollisionFree] = m_stateSpace->SteerExactly(nearestNode->GetState(), newState);
				double bestCost = nearestNode->meta.costFromGoal + transitionCost;
				for (auto node : nearNodes) {
					auto [transitionCost, transitionCollisionFree] = m_stateSpace->SteerExactly(node->GetState(), newState);
					double cost = node->meta.costFromGoal + transitionCost;
					if (cost < bestCost && transitionCollisionFree) {
						bestParentNode = node;
						bestCost = cost;
					}
				}
				auto newNode = m_tree.Extend(newState, bestParentNode);
				newNode->meta.costFromGoal = bestCost;

				// Reparent the nearest neighbor if necessary
				for (auto node : nearNodes) {
					auto [transitionCost, transitionCollisionFree] = m_stateSpace->SteerExactly(newNode->GetState(), node->GetState());
					double cost = newNode->meta.costFromGoal + transitionCost;
					if (cost < node->meta.costFromGoal && transitionCollisionFree) {
						m_tree.Reparent(node, newNode);
						node->meta.costFromGoal = cost;
					}
				}

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

		/// @brief Clear the tree.
		void Clear()
		{
			m_tree.Clear();
			m_solutionNode = nullptr;
		}

	private:
		RRTStarParameters m_parameters;
		Ref<RRTStarStateSpace<Vertex>> m_stateSpace;

		Tree<Vertex, Dimensions, NodeMetadata, Hash, VertexType> m_tree;
		typename Tree<Vertex, Dimensions, NodeMetadata, Hash, VertexType>::Node* m_solutionNode = nullptr;
	};
}
