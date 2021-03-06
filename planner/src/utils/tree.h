#pragma once

#include "core/base.h"
#include "utils/node.h"

#include <unordered_map>
#include <flann/flann.hpp>

namespace Planner {

	template <typename State, typename NodeInfo>
	struct TreeNodeData {
		State state;
		NodeInfo info;

		TreeNodeData(const State& state) :
			state(state), info(NodeInfo()) { }
		TreeNodeData(const State& state, const NodeInfo& info) :
			state(state), info(info) { }
	};
	template <typename State, typename NodeInfo>
	using TreeNode = Node<TreeNodeData<State, NodeInfo>>;

	/// @brief Tree of T node.
	/// @details This tree allows operation such as finding a nearest neighbor
	/// to a node in the tree, rewiring the tree.
	template <
		typename State,
		unsigned int Dimensions,
		typename NodeInfo = NullClass,
		typename Hash = std::hash<State>,
		typename Equal = std::equal_to<State>,
		typename DataType = double>
	class Tree {
	public:
		using Node = TreeNode<State, NodeInfo>;

	public:
		Tree(const Tree&) = delete;
		Tree& operator=(const Tree&) = delete;

		///@brief Constructor.
		Tree(size_t mapBucketCound = 20) :
			m_exploredNodeMap(mapBucketCound), m_kdTree(flann::KDTreeSingleIndexParams()) {};
		~Tree() = default;

		///@brief Return the number of node in the tree.
		/// @return The number of nodes.
		std::size_t Size() const { return m_exploredNodeMap.size(); }

		/// @brief Check if the node belongs to the tree.
		/// @return A boolean.
		bool BelongToTree(const Node* node) const
		{
			return (node->IsDescendantOf(m_rootNode.get()) || (node == m_rootNode.get()));
		}

		/// @brief Create the root node of the tree.
		Node* CreateRootNode(const State& start)
		{
			m_rootNode = makeScope<Node>(start);
			m_exploredNodeMap.insert({ start, m_rootNode.get() });
			m_kdTree.buildIndex(flann::Matrix<DataType>((DataType*)&((*m_rootNode)->state), 1, m_dimensions));
			return m_rootNode.get();
		}

		/// @brief Find the k nearest node in the tree to @state.
		/// @details This method searches a k-d tree of the points to determine
		/// the nearest neighbor.
		/// @param state The state whose closest neighbors must be find.
		/// @param nn The number of neighbors to find.
		/// @return The nearest nodes to @state.
		std::vector<Node*> GetNearestNodes(const State& state, const unsigned int nn = 1)
		{
			std::vector<Node*> nodes;
			if (nn == 0)
				return nodes;

			flann::Matrix<DataType> query;
			query = flann::Matrix<DataType>((DataType*)&state, 1, sizeof(state) / sizeof(DataType));

			flann::Matrix<int> indices(new int[query.rows * nn], query.rows, nn);
			flann::Matrix<DataType> dists(new DataType[query.rows * nn], query.rows, nn);
			int n = m_kdTree.knnSearch(query, indices, dists, nn, flann::SearchParams());

			nodes.reserve(n);
			for (unsigned int i = 0; i < n; i++) {
				State point = (State)m_kdTree.getPoint(indices[0][i]);
				nodes.push_back(m_exploredNodeMap[point]);
			}

			delete[] indices.ptr();
			delete[] dists.ptr();
			return nodes;
		}

		/// @brief Find the node in the tree closest to @state.
		/// @details This method searches a k-d tree of the points to determine
		/// the nearest neighbor.
		/// @param state The state whose closest neighbor must be find.
		/// @return The nearest node to @state.
		Node* GetNearestNode(const State& state)
		{
			flann::Matrix<DataType> query;
			query = flann::Matrix<DataType>((DataType*)&state, 1, sizeof(state) / sizeof(DataType));

			flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
			flann::Matrix<DataType> dists(new DataType[query.rows], query.rows, 1);
			m_kdTree.knnSearch(query, indices, dists, 1, flann::SearchParams());

			State point = (State)m_kdTree.getPoint(indices[0][0]);

			delete[] indices.ptr();
			delete[] dists.ptr();
			return m_exploredNodeMap[point];
		}

		/// @brief Extend the tree by adding a new node target whose parent is
		/// start.
		/// @param target The target point to extend the tree to.
		/// @param start The node to connect from. If none is provided, the
		/// closest node in the tree is used instead.
		/// @return The new node in the tree.
		Node* Extend(const State& target, Node* source = nullptr)
		{
			// Check if the target is already in the tree
			auto search = m_exploredNodeMap.find(target);
			if (search != m_exploredNodeMap.end())
				return search->second;

			// Get source node (use nearest node if necessary, return if there is no nearest node)
			source = source ? source : GetNearestNode(target);
			if (!source) {
				return nullptr;
			}
			PP_ASSERT(BelongToTree(source), "The node to extend does not belong to the tree.");

			// Create the new node and connect it to its parent
			Node* newNode = source->AddChild(makeScope<Node>(target));

			// Add the node to the map and k-d tree
			m_kdTree.addPoints(flann::Matrix<DataType>((DataType*)&((*newNode)->state), 1, m_dimensions));
			m_exploredNodeMap.insert({ (*newNode)->state, newNode });

			return newNode;
		}

		/// @brief Change the parent of @child to @newParent.
		/// @details If child is not a parent of newParent, the method does not
		/// change the hierarchy of any other nodes, including the relationship
		/// between @child and its children and @newParent and its
		/// children.
		/// Both @child and @newParent must not be nullptr.
		/// @param child The child node.
		/// @param newParent The new parent of child.
		void Reparent(Node* child, Node* newParent)
		{
			Node::Reparent(child, newParent, m_rootNode);
		}

		/// @brief Clear the tree and deletes all nodes.
		void Clear()
		{
			m_exploredNodeMap.clear();
			// TODO allow chaning metric used by KD_tree
			m_kdTree = flann::Index<flann::L2_Simple<DataType>>(flann::KDTreeSingleIndexParams());
			m_rootNode.reset();
		}

	private:
		const unsigned int m_dimensions = Dimensions;

		std::unordered_map<State, Node*, Hash, Equal> m_exploredNodeMap;
		flann::Index<flann::L2_Simple<DataType>> m_kdTree;

		Scope<Node> m_rootNode;
	};
}
