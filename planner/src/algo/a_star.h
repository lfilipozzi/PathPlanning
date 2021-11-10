#pragma once

#include "algo/path_planner.h"
#include "state_space/state_space.h"
#include "utils/node.h"
#include "core/hash.h"
#include "utils/frontier.h"

namespace Planner {

	/// @brief Interface to sample the configuration space as required by A*
	/// algorithms.
	template <typename State>
	class AStarStatePropagator {
	public:
		AStarStatePropagator() = default;
		virtual ~AStarStatePropagator() = default;

		/// @brief Returns a list of all neighbor positions and the transition
		/// cost. The transition from the current state to the neighbor state must
		/// be valid.
		/// @return A list of tuple all neighboring positions and their associated
		/// transition cost.
		virtual std::vector<std::tuple<State, double>> GetNeighborStates(const State& state) = 0;
	};

	/// @brief Interface for an A* heuristic function.
	template <typename State>
	class AStarHeuristic {
	public:
		AStarHeuristic() = default;
		virtual ~AStarHeuristic() = default;

		/// @brief Heuristic function.
		virtual double GetHeuristicValue(const State& from, const State& to) = 0;
	};

	/// @brief Combine several admissible heuristics into an improved admissible
	/// one.
	template <typename State>
	class AStarCombinedHeuristic : public AStarHeuristic<State> {
	public:
		AStarCombinedHeuristic() = default;

		void Add(const Ref<AStarHeuristic<State>>& heuristic)
		{
			m_heuristics.push_back(heuristic);
		}

		template <typename... Args>
		void Add(const Ref<AStarHeuristic<State>>& heuristic, Args&&... args)
		{
			m_heuristics.push_back(heuristic);
			Add(std::forward<Args>(args)...);
		}

		virtual double GetHeuristicValue(const State& from, const State& to) override
		{
			double value = -std::numeric_limits<double>::infinity();
			for (auto& h : m_heuristics) {
				value = std::max(value, h->GetHeuristicValue(from, to));
			}
			return value;
		}

	private:
		std::vector<Ref<AStarHeuristic<State>>> m_heuristics;
	};

	/// @brief Implementation of the A* algorithm.
	template <typename State, typename HashState = std::hash<State>, typename EqualState = std::equal_to<State>, bool GraphSearch = true>
	class AStar : public PathPlanner<State> {
		static_assert(std::is_copy_constructible<State>::value);

	public:
		/// @brief Node metadata used by A* star.
		struct NodeMetadata {
			double pathCost = 0;
			double totalCost = 0;
		};
		using Node = GenericNode<State, NodeMetadata>;

	protected:
		struct CompareNode {
			bool operator()(Node* lhs, Node* rhs) const
			{
				return lhs->meta.totalCost > rhs->meta.totalCost;
			}
		};

		struct HashNode {
			std::size_t operator()(Node* node) const
			{
				return HashState()(node->GetState());
			}
		};

		struct EqualNode {
			bool operator()(Node* lhs, Node* rhs) const
			{
				return EqualState()(lhs->GetState(), rhs->GetState());
			}
		};

	public:
		/// @brief Constructor.
		/// @param Propagator The state-space propagator.
		AStar() = default;
		virtual ~AStar() = default;

		/// @copydoc Planner::PathPlanner::GetPath
		virtual std::vector<State> GetPath() const override
		{
			std::vector<State> path;

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

		/// @brief Return the set of explored states.
		const std::unordered_set<State, HashState, EqualState>& GetExploredStates() const { return m_explored; }

		/// @Brief Return the state-propagator used by the algorithm.
		const Ref<AStarStatePropagator<State>>& GetStatePropagator() const { return m_propagator; }
		/// @brief Return the heuristic used by the algorithm
		const Ref<AStarHeuristic<State>>& GetHeuristic() const { return m_heuristic; }

		/// @brief Initialize A* search.
		bool Initialize(const Ref<AStarStatePropagator<State>>& propagator, const Ref<AStarHeuristic<State>>& heuristic)
		{
			if (!propagator || !heuristic) {
				isInitialized = false;
				return false;
			}

			m_propagator = propagator;
			m_heuristic = heuristic;
			isInitialized = true;
			return true;
		}

		/// @copydoc Planner::PathPlanner::SearchPath
		virtual Status SearchPath() override
		{
			if (!isInitialized) {
				PP_ERROR("The algorithm has not been initialized successfully.");
				return Status::Failure;
			}

			InitializeSearch();

			while (!m_frontier.Empty()) {
				PP_INFO("Open: {0}, close: {1}", m_frontier.Size(), m_explored.size());

				// Pop lowest-cost node of the frontier
				auto node = m_frontier.Pop();

				// Check if node is a solution
				if (EqualState()(node->GetState(), this->m_goal)) {
					m_solutionNode = node;
					return Status::Success;
				}

				m_explored.insert(node->GetState());

				AddChildren(node);
			}
			return Status::Failure;
		}

	protected:
		inline virtual void InitializeSearch()
		{
			m_frontier.Clear();
			m_explored.clear();
			m_solutionNode = nullptr;
			m_rootNode.reset();

			m_rootNode = makeScope<Node>(this->m_init);
			m_frontier.Push(m_rootNode.get());
			m_explored.insert(m_rootNode->GetState());
		}

		/// @brief Add the child of @node to the frontier and to @node
		inline virtual void AddChildren(Node* node)
		{
			for (auto& [childState, transitionCost] : m_propagator->GetNeighborStates(node->GetState())) {
				// Create the child node
				Scope<Node> childScope = makeScope<Node>(childState);
				Node* child = childScope.get();
				child->meta.pathCost = node->meta.pathCost + transitionCost;
				child->meta.totalCost = child->meta.pathCost + m_heuristic->GetHeuristicValue(child->GetState(), this->m_goal);

				// Check if the child node is in the frontier or explored set
				if constexpr (GraphSearch) {
					Node* const* inFrontier = m_frontier.Find(child);
					bool inExplored = m_explored.find(child->GetState()) != m_explored.end();
					if (!inFrontier && !inExplored) {
						// Add child to frontier and to the tree
						m_frontier.Push(child);
						node->AddChild(std::move(childScope));
					} else if (inFrontier) {
						// Check if the node in frontier has a higher cost than the
						// current path, and if so replace it by child
						if ((*inFrontier)->meta.totalCost > child->meta.totalCost) {
							// Replace the node *inFrontier by the newly find better node.
							auto previousChildScope = (*inFrontier)->GetParent()->ReplaceChild(*inFrontier, std::move(childScope), false);
							// Replace the node in the frontier by child
							m_frontier.Remove(previousChildScope.get());
							m_frontier.Push(child);
						}
					}
				} else {
					// Add child to frontier and to the tree
					m_frontier.Push(child);
					node->AddChild(std::move(childScope));
				}
			}
		}

	protected:
		Ref<AStarStatePropagator<State>> m_propagator;
		Ref<AStarHeuristic<State>> m_heuristic;

		Frontier<Node*, CompareNode, HashNode, EqualNode> m_frontier;
		std::unordered_set<State, HashState, EqualState> m_explored;

		Scope<Node> m_rootNode;
		Node* m_solutionNode = nullptr;

	private:
		bool isInitialized = false;
	};
}
