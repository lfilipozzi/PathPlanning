#pragma once

#include "core/base.h"

#include <vector>

namespace Planner {

	struct NullClass {
	};

	/// @brief Node
	template <typename T>
	class Node {
	public:
		Node() = default;
		Node(const T& data) :
			m_data(data) { }
		template <typename... Args>
		Node(Args&&... args) :
			m_data(T(std::forward<Args>(args)...)) { }
		~Node() = default;

		/// @brief Perform a pre-order traversal of the tree whose root node
		/// is *this.
		/// @param func Function to apply.
		template <typename Func>
		void PreOrderTraversal(Func func) const
		{
			func(*this);
			for (auto& child : m_children) {
				child->PreOrderTraversal(func);
			}
		}

		/// @brief Perform a post-order traversal of the tree whose root node
		/// is *this.
		/// @param func Function to apply.
		template <typename Func>
		void PostOrderTraversal(Func func) const
		{
			for (auto& child : m_children) {
				child->PostOrderTraversal(func);
			}
			func(*this);
		}

		/// @brief Compute the depth of the node in the tree.
		/// @return The depth.
		[[nodiscard]] unsigned int GetDepth() const
		{
			unsigned int depth = 0;
			for (Node* parent = m_parent; parent != nullptr; parent = parent->m_parent) {
				depth++;
			}
			return depth;
		}

		/// @brief Return the parent of the node.
		/// @return The parent of the node.
		const Node* GetParent() const { return m_parent; }
		Node* GetParent() { return m_parent; }

		/// @brief Return the children of the node.
		/// @return The children of the node.
		const std::vector<Scope<Node>>& GetChildren() const { return m_children; }

		/// @brief Return the previous sibling of the node.
		/// @return The previous sibling of the node.
		const Node* GetPrevious() const
		{
			if (m_parent == nullptr)
				return nullptr;
			auto& siblings = m_parent->m_children;
			auto it = std::find_if(siblings.begin(), siblings.end(), [this](const Scope<Node>& object) { return object.get() == this; });
			if (it == siblings.begin())
				return nullptr;
			return (--it)->get();
		}

		/// @brief Return the next sibling of the node.
		/// @return The next sibling of the node.
		const Node* GetNext() const
		{
			if (m_parent == nullptr)
				return nullptr;
			auto& siblings = m_parent->m_children;
			auto it = std::find_if(siblings.rbegin(), siblings.rend(), [this](const Scope<Node>& object) { return object.get() == this; });
			if (it == siblings.rbegin())
				return nullptr;
			return (--it)->get();
		}

		/// @brief Return the first children of the node.
		/// @return The first children of the node.
		const Node* GetFirst() const
		{
			if (m_children.empty())
				return nullptr;
			return m_children[0].get();
		}

		T& operator*() { return m_data; }
		const T& operator*() const { return m_data; }
		T* operator->() { return &m_data; }
		const T* operator->() const { return &m_data; }

		/// @brief Check if the current node if a child of @node.
		/// @return True if the @this is a child of @node, false otherwise.
		[[nodiscard]] bool IsDescendantOf(const Node* const node) const
		{
			Node* parent = m_parent;
			while (parent != nullptr) {
				if (parent == node)
					return true;
				parent = parent->m_parent;
			}
			return false;
		}

		/// @brief Check if the current node is a parent of @node.
		/// @return True if @this is a parent of @node, false otherwise.
		[[nodiscard]] bool IsAncestorOf(const Node* const node) const
		{
			return node->IsDescendantOf(this);
		}

		/// @brief Set @node as a child of the current node.
		/// @details Change the ownership of the node.
		/// @return The child node.
		Node* AddChild(Scope<Node>&& node)
		{
			m_children.push_back(std::move(node));
			Node* nodePtr = m_children.back().get();
			nodePtr->m_parent = this;
			return nodePtr;
		}

		/// @brief Remove the node @node. @node must be a child of the
		/// current node.
		/// @details This method does not delete the node @node and its
		/// children, it only remove the ownership of @node by its parent.
		/// @param node The child to remove.
		/// @return A smart pointer owning the removed child and its possible
		/// children.
		Scope<Node> RemoveChild(Node* node)
		{
			Scope<Node> childScope;
			for (auto it = m_children.begin(); it != m_children.end(); it++) {
				if (it->get() == node) {
					childScope = std::move(*it);
					childScope->m_parent = nullptr;
					m_children.erase(it);
					return childScope;
				}
			}

			PP_ASSERT(false, "The node to remove is not a direct child.");
			return childScope;
		}

		/// @brief Remove the node @brat and replace it by @changeling. If @brat
		/// has children, they become the children of @changeling if
		/// @adoptBratsChildren is set to true so that these nodes stay in the
		/// tree.
		/// @param brat The child to remove.
		/// @param changeling The new child.
		/// @param adoptGrandchildren Boolean, if set to true, the children of @brat
		/// are moved to children of @changeling.
		/// @return A Scope owning @brat.
		Scope<Node> ReplaceChild(Node* brat, Scope<Node>&& changeling, bool adoptGrandchildren = true)
		{
			Scope<Node> bratScope;
			for (auto it = m_children.begin(); it != m_children.end(); it++) {
				if (it->get() == brat) {
					bratScope = std::move(*it);
					bratScope->m_parent = nullptr;
					if (adoptGrandchildren) {
						for (auto& child : bratScope->m_children) {
							child->m_parent = changeling.get();
						}
						changeling->m_children = std::move(bratScope->m_children);
					}
					changeling->m_parent = this;
					*it = std::move(changeling);
					return bratScope;
				}
			}

			PP_ASSERT(false, "The node to remove is not a direct child.");
			return bratScope;
		}

		/// @brief Change the parent of @child to @newParent.
		/// @details If child is not a parent of newParent, the method does not
		/// change the hierarchy of any other nodes, including the relationship
		/// between @child and its children and @newParent and its
		/// children.
		/// Both @child and @newParent must not be nullptr.
		/// @param child The child node.
		/// @param newParent The new parent of child.
		/// @param rootNode The root node of the tree.
		static void Reparent(Node* child, Node* newParent, Scope<Node>& rootNode)
		{
			// Delete parenthood between child and its parent or de-anchor this node as the root node
			Node* const oldParent = child->GetParent();
			Scope<Node> childScope;
			if (oldParent)
				childScope = oldParent->RemoveChild(child);
			else
				childScope = std::move(rootNode);

			// The hierarchy must be modified if newParent is a child of child.
			// Add it as a child of oldParent or anchor it as the root node
			if (newParent->IsDescendantOf(child)) {
				Scope<Node> newParentScope = newParent->GetParent()->RemoveChild(newParent);
				if (oldParent)
					oldParent->AddChild(std::move(newParentScope));
				else
					rootNode = std::move(newParentScope);
			}

			// Add the child to parent
			newParent->AddChild(std::move(childScope));
		}

	protected:
		T m_data;

	private:
		std::vector<Scope<Node>> m_children;
		Node* m_parent = nullptr;
	};
}
