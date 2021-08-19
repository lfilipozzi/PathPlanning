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
			[[nodiscard]] unsigned int GetDepth() const
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

			/**
			 * @brief Return the total cost from the goal to the state.
			 * @return The cost.
			 */
			[[nodiscard]] double GetCostFromGoal() const { return m_costFromGoal; }

			/**
			 * @brief Modify the total cost from the goal to the state.
			 */
			void SetCostFromGoal(double cost) { m_costFromGoal = cost; }

			/**
			 * @brief Check if the current node if a child of @node.
			 * @return True if the @this is a child of @node, false otherwise.
			 */
			[[nodiscard]] bool IsChildOf(const Node* const node) const
			{
				Node* parent = m_parent;
				while (parent != nullptr) {
					if (parent == node)
						return true;
					parent = parent->m_parent;
				}
				return false;
			}

			/**
			 * @brief Check if the current node is a parent of @node.
			 * @return True if @this is a parent of @node, false otherwise.
			 */
			[[nodiscard]] bool IsParentOf(const Node* const node) const
			{
				return node->IsChildOf(this);
			}

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

			/**
			 * @brief Remove the node @node. @node must be a child of the 
			 * current node.
			 * @details This method does not delete the node @node and its 
			 * children, it only remove the ownership of @node by its parent.
			 * @param node The child to remove.
			 * @return A smart pointer owning the removed child and its possible
			 * children.
			 */
			Scope<Node> RemoveChild(Node* node)
			{
				Scope<Node> childScope;
				for (auto it = m_children.begin(); it != m_children.end(); it++) {
					if (it->get() == node) {
						childScope = std::move(*it);
						m_children.erase(it);
						return childScope;
					}
				}
				// TODO add false assertion here if node is not a child of *this
				return childScope;
			}

		private:
			std::vector<Scope<Node>> m_children;
			Vertex m_state;
			Node* m_parent = nullptr;
			double m_costFromGoal = 0;
		};

	public:
		Tree(const Tree&) = delete;
		Tree& operator=(const Tree&) = delete;

		/**
		 * @brief Constructor.
		 */
		Tree(size_t mapBucketCound = 20) :
			m_exploredNodeMap(mapBucketCound), m_kdTree(flann::KDTreeSingleIndexParams()) {};
		~Tree() = default;

		/**
		 * @brief Return the number of node in the tree.
		 * @return The number of nodes.
		 */
		std::size_t GetSize() const { return m_exploredNodeMap.size(); }

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
			std::vector<Node*> nodes;
			if (nn == 0)
				return nodes;

			flann::Matrix<VertexType> query;
			query = flann::Matrix<VertexType>((VertexType*)&state, 1, sizeof(state) / sizeof(VertexType));

			std::vector<int> indicesBuffer(query.rows);
			std::vector<VertexType> distsBuffer(query.rows);
			flann::Matrix<int> indices(indicesBuffer.data(), query.rows, nn);
			flann::Matrix<VertexType> dists(distsBuffer.data(), query.rows, nn);
			int n = m_kdTree.knnSearch(query, indices, dists, nn, flann::SearchParams());

			nodes.reserve(n);
			for (unsigned int i = 0; i < n; i++) {
				Vertex point = (Vertex)m_kdTree.getPoint(indices[0][i]);
				nodes.push_back(m_exploredNodeMap[point]);
			}

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

			std::vector<int> indicesBuffer(query.rows);
			std::vector<VertexType> distsBuffer(query.rows);
			flann::Matrix<int> indices(indicesBuffer.data(), query.rows, 1);
			flann::Matrix<VertexType> dists(distsBuffer.data(), query.rows, 1);
			m_kdTree.knnSearch(query, indices, dists, 1, flann::SearchParams());

			Vertex point = (Vertex)m_kdTree.getPoint(indices[0][0]);

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
			// TODO: check that source belongs to tree in debug using IsChildOf or IsParentOf

			// Check if the target is already in the tree
			auto search = m_exploredNodeMap.find(target);
			if (search != m_exploredNodeMap.end())
				return search->second;

			// Get source node (use nearest node if necessary, return if there is no nearest node)
			source = source ? source : GetNearestNode(target);
			if (!source) {
				return nullptr;
			}

			// Create the new node and connect it to its parent
			Node* newNode = source->AddChild(makeScope<Node>(target));

			// Add the node to the map and k-d tree
			m_kdTree.addPoints(flann::Matrix<VertexType>((VertexType*)&(newNode->GetState()), 1, m_dimensions));
			m_exploredNodeMap.insert({ newNode->GetState(), newNode });

			return newNode;
		}

		/**
		 * @brief Change the parent of @child to @newParent. 
		 * @details If child is not a parent of newParent, the method does not 
		 * change the hierarchy of any other nodes, including the relationship 
		 * between @child and its children and @newParent and its 
		 * children.
		 * Both @child and @newParent must not be nullptr.
		 * @param child The child node.
		 * @param newParent The new parent of child.
		 */
		void Reparent(Node* child, Node* newParent)
		{
			// TODO assert that child and newParent are not empty in debug

			// Delete parenthood between child and its parent or de-anchor this node as the root node
			Node* oldParent = child->GetParent();
			Scope<Node> childScope;
			if (oldParent)
				childScope = oldParent->RemoveChild(child);
			else
				childScope = std::move(m_rootNode);

			// The hierarchy must be modified if newParent is a child of child
			if (newParent->IsChildOf(child)) {
				Scope<Node> newParentScope = newParent->GetParent()->RemoveChild(newParent);
				if (oldParent)
					oldParent->AddChild(std::move(newParentScope));
				else
					m_rootNode = std::move(newParentScope);
			}

			// Add the child to parent
			newParent->AddChild(std::move(childScope));
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
