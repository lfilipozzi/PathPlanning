#pragma once

#include <vector>
#include <unordered_map>
#include <flann/flann.hpp>

#include "base.h"

namespace Planner {

	/**
	* @brief Tree of T node.
	* @details This tree allows operation such as finding a nearest neighbor 
	* to a node in the tree, rewiring the tree.
	*/
	template <typename Vertex, unsigned int Dimensions, class Hash = std::hash<Vertex>, typename VertexType = double>
	class Tree {
	public:
		class Node {
		public:
			friend Tree<Vertex, Dimensions>;

			Node(const Vertex& state) :
				m_state(state) { }
			~Node() = default;

			/**
			* @brief Compute the depth of the node in the tree.
			* @return The depth.
			*/
			unsigned int GetDepth() const
			{
				unsigned int depth = 0;
				for (Node* parent = m_parent; parent != nullptr; parent = parent->m_parent) {
					depth++;
				}
				return depth;
			}

			/**
			* @brief Return the parent of the node.
			* @return The parent of the node.
			*/
			const Node* GetParent() const { return m_parent; }
			Node* GetParent() { return m_parent; }

			/**
			* @brief Return the state of the node.
			* @return The state of the node.
			*/
			const Vertex& GetState() const { return m_state; }
			Vertex& GetState() { return m_state; }

		private:
			/**
			* @brief Set @node as a child of the current node.
			* @details Change the ownership of the node.
			* @return The child node.
			*/
			Node* AddChild(Scope<Node>&& node)
			{
				node->m_parent = this;
				m_children.push_back(std::move(node));
				return m_children.back().get();
			}

		private:
			std::vector<Scope<Node>> m_children;
			Vertex m_state;
			Node* m_parent;
		};

	public:
		Tree(const Tree&) = delete;
		Tree& operator=(const Tree&) = delete;

		/**
		 * @brief Constructor.
		 */
		Tree() :
			m_exploredNodeMap(20), m_kdTree(flann::KDTreeSingleIndexParams()) {};
		~Tree() = default;

		/**
		 * @brief Create the root node of the tree.
		 */
		Node* CreateRootNode(const Vertex& start)
		{
			m_rootNode = makeScope<Node>(start);
			m_exploredNodeMap.insert({ start, m_rootNode.get() });
			m_kdTree.buildIndex(flann::Matrix<VertexType>((VertexType*)&(m_rootNode->GetState()), 1, m_dimensions));
			return m_rootNode.get();
		}

		/**
		* @brief Find the k nearest node in the tree to @state. 
		* @details This method searches a k-d tree of the points to determine 
		* the nearest neighbor.
		* @param state The state whose closest neighbors must be find.
		* @param nn The number of neighbors to find.
		* @return The nearest nodes to @state.
		*/
		std::vector<Node*> GetNearestNodes(const Vertex& state, const unsigned int nn = 1)
		{
			flann::Matrix<VertexType> query;
			query = flann::Matrix<VertexType>((VertexType*)&state, 1, sizeof(state) / sizeof(VertexType));

			flann::Matrix<int> indices(new int[query.rows * nn], query.rows, 1);
			flann::Matrix<VertexType> dists(new VertexType[query.rows * nn], query.rows, 1);
			int n = m_kdTree.knnSearch(query, indices, dists, nn, flann::SearchParams());

			std::vector<Node*> nodes;
			nodes.reserve(n);
			for (unsigned int i = 0; i < n; i++) {
				Vertex point = (Vertex)m_kdTree.getPoint(indices[i][0]);
				nodes.push_back(m_exploredNodeMap[point]);
			}

			delete[] indices.ptr();
			delete[] dists.ptr();

			return nodes;
		}

		/**
		* @brief Find the node in the tree closest to @state. 
		* @details This method searches a k-d tree of the points to determine 
		* the nearest neighbor.
		* @param state The state whose closest neighbor must be find.
		* @return The nearest node to @state.
		*/
		Node* GetNearestNode(const Vertex& state)
		{
			flann::Matrix<VertexType> query;
			query = flann::Matrix<VertexType>((VertexType*)&state, 1, sizeof(state) / sizeof(VertexType));

			flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
			flann::Matrix<VertexType> dists(new VertexType[query.rows], query.rows, 1);
			m_kdTree.knnSearch(query, indices, dists, 1, flann::SearchParams());

			Vertex point = (Vertex)m_kdTree.getPoint(indices[0][0]);

			delete[] indices.ptr();
			delete[] dists.ptr();

			return m_exploredNodeMap[point];
		}

		/**
		* @brief Extend the tree by adding a new node target whose parent is 
		* start.
		* @param target The target point to extend the tree to.
		* @param start The node to connect from. If none is provided, the 
		* closest node in the tree is used instead.
		* @return The new node in the tree.
		*/
		Node* Extend(const Vertex& target, Node* source = nullptr)
		{
			// TODO: check that source belongs to tree in debug

			// Get source node (use nearest node if necessary, return if there is no nearest node)
			source = source ? source : GetNearestNode(target);
			if (!source) {
				return nullptr;
			}

			// Check if the target is already in the tree
			auto search = m_exploredNodeMap.find(target);
			if (search != m_exploredNodeMap.end())
				return search->second;

			// Create the new node and connect it to its parent
			Node* newNode = source->AddChild(makeScope<Node>(target));

			// Add the node to the map and k-d tree
			m_kdTree.addPoints(flann::Matrix<VertexType>((VertexType*)&(newNode->GetState()), 1, m_dimensions));
			m_exploredNodeMap.insert({ newNode->GetState(), newNode });

			return newNode;
		}

		/**
		 * @brief Clear the tree and deletes all nodes.
		 */
		void Clear()
		{
			m_exploredNodeMap.clear();
			m_kdTree = flann::Index<flann::L2_Simple<VertexType>>(flann::KDTreeSingleIndexParams());
			m_rootNode.reset();
		}

	private:
		const unsigned int m_dimensions = Dimensions;

		std::unordered_map<Vertex, Node*, Hash> m_exploredNodeMap;
		flann::Index<flann::L2_Simple<VertexType>> m_kdTree;

		Scope<Node> m_rootNode;
	};
}
