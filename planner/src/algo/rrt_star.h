#pragma once

#include "algo/path_planner.h"
#include "paths/path.h"
#include "state_space/state_space.h"
#include "state_validator/state_validator.h"
#include "utils/tree.h"

namespace Planner {

	/// @brief Tunable parameters of the RRT algorithm.
	struct RRTStarParameters {
		/// @brief Maximum number of iterations
		unsigned int maxIteration = 1e4;
		/// @brief Maximum number of nodes in the search tree
		unsigned int maxNumberTreeNode = 1e4;
		/// @brief Maximum length of a motion allowed in the tree
		double maxConnectionDistance = 0.1;
		/// @brief Probability of choosing goal state during state sampling
		double goalBias = 0.05;
	};

	template <
		typename Vertex,
		unsigned int Dimensions,
		typename Hash = std::hash<Vertex>,
		typename Equal = std::equal_to<Vertex>,
		typename DataType = double>
	class RRTStar : public PathPlanner<Vertex> {
	private:
		/// @brief Node metadata used by RRT star.
		struct NodeInfo {
			/// @brief Path connecting the previous state to the current state.
			Ref<Path<Vertex>> path;
			/// @brief Cost from goal
			double costFromGoal = 0;
		};

	public:
		/// @brief Constructor.
		RRTStar(const Ref<StateSpace<Vertex, Dimensions, DataType>>& stateSpace,
			const Ref<StateValidator<Vertex, Dimensions, DataType>>& stateValidator,
			const Ref<PathConnection<Vertex>>& pathConnection) :
			m_stateSpace(stateSpace),
			m_stateValidator(stateValidator),
			m_pathConnection(pathConnection) {};
		virtual ~RRTStar() = default;

		RRTStarParameters GetParameters() { return m_parameters; }
		const RRTStarParameters& GetParameters() const { return m_parameters; }
		void SetParameters(const RRTStarParameters& params) { m_parameters = params; }

		virtual Status SearchPath() override
		{
			Clear();

			m_tree.CreateRootNode(this->m_init);

			int count = -1;
			while (true) {
				count++;
				if (count > m_parameters.maxIteration) {
					return Status::Failure;
				}
				if (m_tree.Size() > m_parameters.maxNumberTreeNode) {
					return Status::Failure;
				}

				// Create a random configuration
				Vertex randomState = Random<double>::SampleUniform(0, 1) < m_parameters.goalBias ? this->m_goal : m_stateSpace->SampleUniform();
				// Find the nearest node in the tree
				TreeNode* const nearestNode = m_tree.GetNearestNode(randomState);
				if (!nearestNode)
					continue;
				// Create a new state towards the direction of randomState
				auto pathNearToNew = SteerTowards((*nearestNode)->state, randomState, m_parameters.maxConnectionDistance);
				if (!m_stateValidator->IsPathValid(*pathNearToNew))
					continue;
				const Vertex& newState = pathNearToNew->GetFinalState();

				// Find the best parent for newNode amongst k nearest neighbors of
				// the new state where k is logarithmic in the size of the tree
				// FIXME need to use radius search to make sure that the connection length is always less than maxConnectionDistance
				unsigned int nn = std::max<unsigned int>(1, log(m_tree.Size()));
				std::vector<TreeNode*> nearNodes = m_tree.GetNearestNodes(newState, nn);
				TreeNode* bestParentNode = nullptr;
				double bestCost = std::numeric_limits<double>::infinity();
				Ref<Path<Vertex>> bestPath;
				for (const auto node : nearNodes) {
					auto [pathParentToNew, transitionCost] = SteerExactly((*node)->state, newState);
					double cost = (*node)->info.costFromGoal + transitionCost;
					if (cost < bestCost && m_stateValidator->IsPathValid(*pathParentToNew)) {
						bestParentNode = node;
						bestCost = cost;
						bestPath = std::move(pathParentToNew);
					}
				}

				// Add the node to the tree
				auto newNode = m_tree.Extend(newState, bestParentNode);
				(*newNode)->info.costFromGoal = bestCost;
				(*newNode)->info.path = std::move(bestPath);

				// Check solution
				if (IsSolution(newState)) {
					m_solutionNode = newNode;
					return Status::Success;
				}
			}

			return Status::Failure;
		}

		virtual std::vector<Vertex> GetPath() const override
		{
			std::vector<Vertex> path;

			// TODO interpolate path

			auto node = m_solutionNode;
			if (!node)
				return path;

			unsigned int depth = m_solutionNode->GetDepth();
			path.resize(depth + 1);
			for (auto it = path.rbegin(); it != path.rend(); it++) {
				*it = (*node)->state;
				node = node->GetParent();
			}
			return path;
		}

	protected:
		/// @brief Check if the node is a solution.
		/// @return true is the node is a solution.
		inline virtual bool IsSolution(const Vertex& state)
		{
			return Equal()(state, this->m_goal);
		}

		/// @brief Construct a new state by moving an incremental distance
		/// from @from towards @to.
		inline virtual Ref<Path<Vertex>> SteerTowards(const Vertex& from, const Vertex& to, double distance)
		{
			auto path = m_pathConnection->Connect(from, to);
			if (path->GetLength() > 0) {
				double ratio = std::clamp(distance / path->GetLength(), 0.0, 1.0);
				path->Truncate(ratio);
			}
			return path;
		}

		/// @brief Construct a path that connects @from to @to.
		/// @return A tuple containing the path connecting the states and the
		/// cost to go from @from to @to.
		virtual std::tuple<Ref<Path<Vertex>>, double> SteerExactly(const Vertex& from, const Vertex& to)
		{
			auto path = m_pathConnection->Connect(from, to);
			return { path, path->GetLength() };
		}

		void Clear()
		{
			m_tree.Clear();
			m_solutionNode = nullptr;
		}

	protected:
		RRTStarParameters m_parameters;
		Ref<StateSpace<Vertex, Dimensions, DataType>> m_stateSpace;
		Ref<StateValidator<Vertex, Dimensions, DataType>> m_stateValidator;
		Ref<PathConnection<Vertex>> m_pathConnection;

		using TreeDeclType = Tree<Vertex, Dimensions, NodeInfo, Hash, Equal, DataType>;
		TreeDeclType m_tree;
		using TreeNode = typename Tree<Vertex, Dimensions, NodeInfo, Hash, Equal, DataType>::Node;
		TreeNode* m_solutionNode = nullptr;
	};

	using RRTStarR2 = RRTStar<Point2d, 2>;
	// using RRTStarSE2 = RRTStar<Pose2d, 3>;
}
