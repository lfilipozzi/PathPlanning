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
		template <typename S>
		friend class AStarCombinedHeuristic;
		template <typename S1, typename S2, typename Func>
		friend class AStarHeuristicAdapter;
		template <typename S, typename HashState, typename EqualState, bool GraphSearch, typename ExploredContainer>
		friend class AStar;

	public:
		AStarHeuristic() = default;
		virtual ~AStarHeuristic() = default;

		/// @brief Heuristic function.
		virtual double GetHeuristicValue(const State& state) = 0;

	protected:
		/// @brief Set the goal of the heuristic.
		virtual void SetGoal(const State& goal) = 0;
	};

	/// @brief Base class for concrete implementation of an A* heuristic
	/// function.
	template <typename State>
	class AStarConcreteHeuristic : public AStarHeuristic<State> {
	public:
		AStarConcreteHeuristic() = default;
		virtual ~AStarConcreteHeuristic() = default;

	protected:
		virtual void SetGoal(const State& goal) override { m_goal = goal; }

	protected:
		State m_goal;
	};

	/// @brief A* heuristic from function.
	template <typename State, typename Func>
	class AStarConcreteHeuristicFcn : public AStarConcreteHeuristic<State> {
	public:
		AStarConcreteHeuristicFcn(Func func) :
			m_func(func) { }
		virtual ~AStarConcreteHeuristicFcn() = default;

		virtual double GetHeuristicValue(const State& state) override
		{
			return m_func(state, this->m_goal);
		}

	protected:
		Func m_func;
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

		virtual double GetHeuristicValue(const State& state) override
		{
			double value = -std::numeric_limits<double>::infinity();
			for (auto& h : m_heuristics) {
				value = std::max(value, h->GetHeuristicValue(state));
			}
			return value;
		}

	protected:
		virtual void SetGoal(const State& goal) override
		{
			for (auto& h : m_heuristics)
				h->SetGoal(goal);
		}

	private:
		std::vector<Ref<AStarHeuristic<State>>> m_heuristics;
	};

	/// @brief Adapt a heuristic for an A* search of type S1 to an A* search of
	/// type S2.
	template <typename S1, typename S2, typename Func>
	class AStarHeuristicAdapter : public AStarHeuristic<S1> {
	public:
		AStarHeuristicAdapter(const Ref<AStarHeuristic<S2>>& heuristic, Func func) :
			m_heuristic(heuristic), m_funcAdapter(func) { }

		virtual double GetHeuristicValue(const S1& state) override
		{
			return m_heuristic->GetHeuristicValue(m_funcAdapter(state));
		}

	protected:
		virtual void SetGoal(const S1& goal) override
		{
			m_heuristic->SetGoal(m_funcAdapter(goal));
		}

	private:
		Ref<AStarHeuristic<S2>> m_heuristic;
		Func m_funcAdapter;
	};

	/// @brief Node metadata used by A* star.
	struct AStarNodeMetadata {
		double pathCost = 0;
		double totalCost = 0;
	};
	template <typename State>
	using AStarNode = GenericNode<State, AStarNodeMetadata>;

	/// @brief Implementation of an explored set for A* search using a
	/// std::unordered_set.
	template <typename State, typename HashState, typename EqualState>
	class Explored : public std::unordered_set<State, HashState, EqualState> {
	public:
		Explored() = default;

		/// @brief Empty the container.
		inline void Clear() { this->clear(); }
		/// @brief Insert a node in the container.
		inline void Insert(AStarNode<State>* const node) { this->insert(node->GetState()); }
		/// @brief Check if a node is in the container.
		inline bool Contains(AStarNode<State>* const node) { return this->find(node->GetState()) != this->end(); }
	};

	/// @brief Implementation of an explored set for A* search using a
	/// std::unordered_map.
	template <typename State, typename HashState, typename EqualState>
	class ExploredMap : public std::unordered_map<State, AStarNode<State>*, HashState, EqualState> {
	public:
		ExploredMap() = default;

		/// @brief Empty the container.
		inline void Clear() { this->clear(); }
		/// @brief Insert a node in the container.
		inline void Insert(AStarNode<State>* const node) { this->insert({ node->GetState(), node }); }
		/// @brief Check if a node is in the container.
		inline bool Contains(AStarNode<State>* const node) { return this->find(node->GetState()) != this->end(); }
	};

	/// @brief Implementation of a void explored set for A* search.
	/// @note For use with A* tree search only.
	template <typename State, typename HashState, typename EqualState>
	class ExploredVoid {
	public:
		ExploredVoid() = default;

		/// @brief Empty the container.
		inline void Clear() { }
		/// @brief Insert a node in the container.
		inline void Insert(AStarNode<State>* const /*node*/) { }
		/// @brief Check if a node is in the container.
		inline bool Contains(AStarNode<State>* const /*node*/) { return false; }
	};

	/// @brief Implementation of the A* algorithm.
	template <
		typename State,
		typename HashState = std::hash<State>,
		typename EqualState = std::equal_to<State>,
		bool GraphSearch = true,
		typename ExploredContainer = Explored<State, HashState, EqualState>>
	class AStar : public PathPlanner<State> {
		static_assert(std::is_copy_constructible<State>::value);

	protected:
		using Node = AStarNode<State>;

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
		const ExploredContainer& GetExploredStates() const { return m_explored; }

		/// @brief Return the optimal cost
		double GetOptimalCost() const
		{
			if (!m_solutionNode)
				return std::numeric_limits<double>::infinity();
			return m_solutionNode->meta.pathCost;
		}

		/// @Brief Return the state-propagator used by the algorithm.
		const Ref<AStarStatePropagator<State>>& GetStatePropagator() const { return m_propagator; }
		/// @brief Return the heuristic used by the algorithm
		const Ref<AStarHeuristic<State>>& GetHeuristic() const { return m_heuristic; }

		/// @brief Initialize A* search.
		/// @param propagator Expand a given node.
		/// @param heuristic Heuristic function to guide the search.
		/// @note The following requirements must be met to ensure optimality of
		/// the solution:
		/// - Admissibility: the heuristic under-evaluate the path cost
		/// - Consistency (for graph search only): \f$h(u) - h(v) \leq c(u,v)\f$
		/// for all point \f$u\f$ and \f$v\f$ with \f$h\f$ the heuristic
		/// function and \f$c(u,v)\f$ the path cost from \f$u\f$ to \f$v\f$.
		bool Initialize(const Ref<AStarStatePropagator<State>>& propagator, const Ref<AStarHeuristic<State>>& heuristic)
		{
			if (!propagator || !heuristic)
				return isInitialized = false;

			m_propagator = propagator;
			m_heuristic = heuristic;
			return isInitialized = true;
		}

		/// @copydoc Planner::PathPlanner::SearchPath
		virtual Status SearchPath() override
		{
			PP_PROFILE_FUNCTION();

			if (!isInitialized) {
				PP_ERROR("The algorithm has not been initialized successfully.");
				return Status::Failure;
			}

			InitializeSearch();

			while (!m_frontier.Empty()) {
				auto node = m_frontier.Pop();
				if (IsSolution(node)) {
					m_solutionNode = node;
					return Status::Success;
				}
				Expand(node);
			}
			return Status::Failure;
		}

	protected:
		/// @brief Initialize the search with the start pose.
		inline virtual void InitializeSearch()
		{
			PP_PROFILE_FUNCTION();

			m_frontier.Clear();
			m_explored.Clear();
			m_solutionNode = nullptr;
			m_rootNode.reset();

			m_rootNode = makeScope<Node>(this->m_init);
			m_frontier.Push(m_rootNode.get());
			m_explored.Insert(m_rootNode.get());

			m_heuristic->SetGoal(this->m_goal);
		}

		/// @brief Check if the node is a solution.
		/// @return true is the node is a solution.
		inline virtual bool IsSolution(Node* node)
		{
			PP_PROFILE_FUNCTION();

			return EqualState()(node->GetState(), this->m_goal);
		}

		/// @brief Add a node to the explored set, expand it, and push its
		/// children to the frontier.
		inline virtual void Expand(Node* node)
		{
			PP_PROFILE_FUNCTION();

			m_explored.Insert(node);

			for (auto& [childState, transitionCost] : m_propagator->GetNeighborStates(node->GetState())) {
				// Create the child node
				Scope<Node> childScope = makeScope<Node>(childState);
				Node* child = childScope.get();
				child->meta.pathCost = node->meta.pathCost + transitionCost;
				child->meta.totalCost = child->meta.pathCost + m_heuristic->GetHeuristicValue(child->GetState());

				// Check if the child node is in the frontier or explored set
				if constexpr (GraphSearch) {
					Node* const* inFrontier = m_frontier.Find(child);
					bool inExplored = m_explored.Contains(child);
					if (!inFrontier && !inExplored) {
						// Add child to frontier and to the tree
						m_frontier.Push(child);
						node->AddChild(std::move(childScope));
					} else if (inFrontier) {
						// Check if the node in frontier has a higher cost than the
						// current path, and if so replace it by child
						ProcessPossibleShortcut(*inFrontier, std::move(childScope), node);
					}
				} else {
					// Add child to frontier and to the tree
					m_frontier.Push(child);
					node->AddChild(std::move(childScope));
				}
			}
		}

		/// @brief Update the frontier if a shorter path is found while
		/// expanding a node.
		/// @param frontierNode The node in the frontier with the same state as
		/// @childScope.
		/// @param childScope The node being added to the frontier.
		/// @param node The node being expanded.
		inline virtual void ProcessPossibleShortcut(Node* frontierNode, Scope<Node> childScope, Node* node)
		{
			PP_PROFILE_FUNCTION();

			auto child = childScope.get();
			if (frontierNode->meta.totalCost > child->meta.totalCost) {
				// Replace the node in the frontier by child
				node->AddChild(std::move(childScope));
				m_frontier.Remove(frontierNode);
				m_frontier.Push(child);
			}
		}

	protected:
		Ref<AStarStatePropagator<State>> m_propagator;
		Ref<AStarHeuristic<State>> m_heuristic;

		Frontier<Node*, CompareNode, HashNode, EqualNode> m_frontier;
		ExploredContainer m_explored;

		Scope<Node> m_rootNode;
		Node* m_solutionNode = nullptr;

	private:
		bool isInitialized = false;
	};
}
