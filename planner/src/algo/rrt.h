#pragma once

#include "algo/path_planner.h"
#include "paths/path.h"
#include "state_space/state_space.h"
#include "state_validator/state_validator.h"
#include "utils/tree.h"

namespace Planner {

	/// @brief Tunable parameters of the RRT algorithm.
	struct RRTParameters {
		/// @brief Maximum number of iterations
		unsigned int maxIteration = 100;
		/// @brief Maximum number of nodes in the search tree
		unsigned int maxNumberTreeNode = 1e4;
		/// @brief Maximum length of a motion allowed in the tree
		double maxConnectionDistance = 0.1;
		/// @brief Probability of choosing goal state during state sampling
		double goalBias = 0.05;
	};

	/// @brief Implementation of the Rapidly-exploring Random Trees (RRT).
	template <
		typename Vertex,
		unsigned int Dimensions,
		typename Hash = std::hash<Vertex>,
		typename Equal = std::equal_to<Vertex>,
		typename DataType = double>
	class RRT : public PathPlanner<Vertex> {
	protected:
		struct NodeMetadata {
			/// @brief Path connecting the previous state to the current state.
			Ref<Path<Vertex>> path;
		};

	public:
		/// @brief Constructor.
		RRT(const Ref<StateSpace<Vertex, Dimensions, DataType>>& stateSpace,
			const Ref<StateValidator<Vertex, Dimensions, DataType>>& stateValidator,
			const Ref<PathConnection<Vertex>>& pathConnection) :
			m_stateSpace(stateSpace), m_stateValidator(stateValidator),
			m_pathConnection(pathConnection)
		{
			Random<DataType>::Init();
			Random<double>::Init();
		};
		virtual ~RRT() = default;

		RRTParameters GetParameters() { return m_parameters; }
		const RRTParameters& GetParameters() const { return m_parameters; }
		void SetParameters(const RRTParameters& params) { m_parameters = params; }

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
				Vertex randomState = Random<double>::SampleUniform(0, 1) < m_parameters.goalBias ?
					this->m_goal : m_stateSpace->SampleUniform();
				// Find the nearest node in the tree
				TreeNode* const nearestNode = m_tree.GetNearestNode(randomState);
				if (!nearestNode)
					continue;
				// Create a new state towards the direction of randomState
				auto pathNearToNew = SteerTowards(nearestNode->GetState(), randomState, m_parameters.maxConnectionDistance);
				if (!m_stateValidator->IsPathValid(*pathNearToNew))
					continue;
				const Vertex& newState = pathNearToNew->GetFinalState();

				// Add the node to the tree
				TreeNode* newNode = m_tree.Extend(newState, nearestNode);
				newNode->meta.path = std::move(pathNearToNew);

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
				*it = node->GetState();
				node = node->GetParent();
			}
			return path;
		}

	protected:
		void Clear()
		{
			m_tree.Clear();
			m_solutionNode = nullptr;
		}

		/// @brief Check if the node is a solution.
		/// @return true is the node is a solution.
		inline virtual bool IsSolution(const Vertex& state)
		{
			return (state - this->m_goal).norm() < 1;
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

	protected:
		RRTParameters m_parameters;
		Ref<StateSpace<Vertex, Dimensions, DataType>> m_stateSpace;
		Ref<StateValidator<Vertex, Dimensions, DataType>> m_stateValidator;
		Ref<PathConnection<Vertex>> m_pathConnection;

		using TreeDeclType = Tree<Vertex, Dimensions, NodeMetadata, Hash, Equal, DataType>;
		TreeDeclType m_tree;
		using TreeNode = typename Tree<Vertex, Dimensions, NodeMetadata, Hash, Equal, DataType>::Node;
		TreeNode* m_solutionNode = nullptr;
	};

	using RRTR2 = RRT<Point2d, 2>;
// 	using RRTSE2 = RRT<Pose2d, 3>;
}
