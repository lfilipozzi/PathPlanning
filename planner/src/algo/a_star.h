#pragma once

#include "algo/path_planner.h"
#include "state_space/state_space.h"
#include "utils/node.h"
#include "core/hash.h"
#include "utils/frontier.h"

namespace Planner {

	using NullAction = NullClass;

	/// @brief Interface to sample the configuration space as required by A*
	/// algorithms.
	template <typename State, typename Action = NullAction>
	class AStarStatePropagator {
	public:
		AStarStatePropagator() = default;
		virtual ~AStarStatePropagator() = default;

		/// @brief Returns a list of all neighbor positions and the transition
		/// cost. The transition from the current state to the neighbor state must
		/// be valid.
		/// @return A list of tuple all neighboring positions and their associated
		/// transition cost.
		virtual std::vector<std::tuple<State, Action, double>> GetNeighborStates(const State& state) = 0;
	};

	/// @brief Interface for an A* heuristic function.
	template <typename State>
	class AStarHeuristic {
		template <typename S>
		friend class AStarCombinedHeuristic;
		template <typename S1, typename S2, typename Func>
		friend class AStarHeuristicAdapter;
		template <typename S>
		friend class AverageHeuristic;
		template <typename S, typename A, typename HashState, typename EqualState, bool GraphSearch, typename ExploredContainer, typename FrontierContainer>
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
		virtual void SetGoal(const State& goal) override final { m_goal = goal; }

	protected:
		State m_goal;
	};

	/// @brief A* heuristic returning zero.
	template <typename State>
	class AStarNullHeuristic : public AStarConcreteHeuristic<State> {
	public:
		AStarNullHeuristic() = default;
		~AStarNullHeuristic() = default;

		virtual double GetHeuristicValue(const State& /*state*/) { return 0.0; }
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
		AStarHeuristicAdapter(Func func) :
			m_funcAdapter(func) { }

		void SetHeuristic(const Ref<AStarHeuristic<S2>>& heuristic) { m_heuristic = heuristic; }

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

	/// Node data for A* search
	template <typename State, typename Action = NullAction>
	struct AStarNodeData {
		const State state;
		const Action action;
		double pathCost = 0;
		double totalCost = 0;

		AStarNodeData(const State& state) :
			state(state), action(Action()) { }
		AStarNodeData(const State& state, const Action& action) :
			state(state), action(action) { }
	};
	/// @brief Node for A* search
	template <typename State, typename Action = NullAction>
	using AStarNode = Node<AStarNodeData<State, Action>>;

	/// @brief Comparator to sort A* node in descending f-cost order, where f=g+h.
	template <typename State, typename Action>
	struct CompareAStarNodeFCost {
		bool operator()(AStarNode<State, Action>* lhs, AStarNode<State, Action>* rhs) const
		{
			return (*lhs)->totalCost > (*rhs)->totalCost;
		}
	};

	/// @brief Hasher to hash A* node
	template <typename State, typename Action, typename HashState>
	struct HashAStarNode {
		std::size_t operator()(AStarNode<State, Action>* node) const
		{
			return hasher((*node)->state);
		}

		HashState hasher;
	};

	/// @brief Comparator so that A* node are equal if they refer to the same state.
	template <typename State, typename Action, typename EqualState>
	struct EqualAStarNode {
		bool operator()(AStarNode<State, Action>* lhs, AStarNode<State, Action>* rhs) const
		{
			return equal((*lhs)->state, (*rhs)->state);
		}

		EqualState equal;
	};

	/// @brief Frontier for A* search
	template <typename State, typename Action, typename HashState, typename EqualState>
	using AStarFrontier = Frontier<AStarNode<State, Action>*, CompareAStarNodeFCost<State, Action>, HashAStarNode<State, Action, HashState>, EqualAStarNode<State, Action, EqualState>>;

	/// @brief Implementation of an explored set for A* search using a
	/// std::unordered_set.
	template <typename State, typename HashState, typename EqualState>
	class Explored : public std::unordered_set<State, HashState, EqualState> {
	public:
		Explored() = default;

		/// @brief Empty the container.
		inline void Clear() { this->clear(); }
		/// @brief Insert a node in the container.
		template <typename Action>
		inline void Insert(AStarNode<State, Action>* const node) { this->insert((*node)->state); }
		/// @brief Check if a node is in the container.
		template <typename Action>
		inline bool Contains(AStarNode<State, Action>* const node) { return this->find((*node)->state) != this->end(); }
	};

	/// @brief Implementation of an explored set for A* search using a
	/// std::unordered_map.
	template <typename State, typename Action, typename HashState, typename EqualState>
	class ExploredMap : public std::unordered_map<State, AStarNode<State, Action>*, HashState, EqualState> {
	public:
		ExploredMap() = default;

		/// @brief Empty the container.
		inline void Clear() { this->clear(); }
		/// @brief Insert a node in the container.
		inline void Insert(AStarNode<State, Action>* const node) { this->insert({ (*node)->state, node }); }
		/// @brief Check if a node is in the container.
		inline bool Contains(AStarNode<State, Action>* const node) { return this->find((*node)->state) != this->end(); }
	};

	/// @brief Implementation of a void explored set for A* search.
	/// @note For use with A* tree search only.
	template <typename State>
	class ExploredVoid {
	public:
		ExploredVoid() = default;

		/// @brief Empty the container.
		inline void Clear() { }
		/// @brief Insert a node in the container.
		template <typename Action>
		inline void Insert(AStarNode<State, Action>* const /*node*/) { }
		/// @brief Check if a node is in the container.
		template <typename Action>
		inline bool Contains(AStarNode<State, Action>* const /*node*/) { return false; }
	};

	/// @brief Interface for A* algorithm
	/// @details This class is only inehrited by AStar and is used so that we 
	/// can infer if an algorithm is an A* search independently of the explored
	/// and frontier containers.
	template <typename State, typename Action, typename HashState, typename EqualState, bool GraphSearch>
	class AStarBase : public PathPlanner<State>{
	public:
		virtual ~AStarBase() = default;

	protected:
		AStarBase() = default;
		template <typename S, typename A, typename HashS, typename EqualS, bool, typename ExploredContainer, typename FrontierContainer>
		friend class AStar;
	};

	/// @brief Trait for A* search algorithm.
	template <typename Search, typename State, typename Action, typename HashState, typename EqualState, bool GraphSearch>
	struct IsAStar {
		static const bool value = std::is_base_of<AStarBase<State, Action, HashState, EqualState, GraphSearch>, Search>::value;
	};

	/// @brief Implementation of the A* algorithm.
	template <
		typename State,
		typename Action = NullAction,
		typename HashState = std::hash<State>,
		typename EqualState = std::equal_to<State>,
		bool GraphSearch = true,
		typename ExploredContainer = Explored<State, HashState, EqualState>,
		typename FrontierContainer = AStarFrontier<State, Action, HashState, EqualState>>
	class AStar : public AStarBase<State, Action, HashState, EqualState, GraphSearch> {
		static_assert(std::is_copy_constructible<State>::value);

		template <typename S, typename A, typename HashS, typename EqualS, bool, typename Algo>
		friend class BidirectionalAStarBase;

	public:
		using Explored = ExploredContainer;
		using Frontier = FrontierContainer;
		using Node = AStarNode<State, Action>;

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
				*it = (*node)->state;
				node = node->GetParent();
			}
			return path;
		}

		/// @brief Return the actions from the initial state to the goal state.
		std::vector<Action> GetActions() const
		{
			std::vector<Action> actions;

			auto node = m_solutionNode;
			if (!node)
				return actions;

			unsigned int depth = m_solutionNode->GetDepth();
			actions.resize(depth);
			for (auto it = actions.rbegin(); it != actions.rend(); it++) {
				*it = (*node)->action;
				node = node->GetParent();
			}

			return actions;
		}

		/// @brief Return the set of explored states.
		const ExploredContainer& GetExploredStates() const { return m_explored; }

		/// @brief Return the optimal cost
		double GetOptimalCost() const
		{
			if (!m_solutionNode)
				return std::numeric_limits<double>::infinity();
			return (*m_solutionNode)->pathCost;
		}

		/// @Brief Return the state-propagator used by the algorithm.
		const Ref<AStarStatePropagator<State, Action>>& GetStatePropagator() const { return m_propagator; }
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
		bool Initialize(const Ref<AStarStatePropagator<State, Action>>& propagator, const Ref<AStarHeuristic<State>>& heuristic)
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

			return EqualState()((*node)->state, this->m_goal);
		}

		/// @brief Add a node to the explored set, expand it, and push its
		/// children to the frontier.
		inline virtual void Expand(Node* node)
		{
			PP_PROFILE_FUNCTION();

			m_explored.Insert(node);

			for (auto& [childState, action, transitionCost] : m_propagator->GetNeighborStates((*node)->state)) {
				// Create the child node
				Scope<Node> childScope = makeScope<Node>(childState, action);
				Node* child = childScope.get();
				(*child)->pathCost = (*node)->pathCost + transitionCost;
				(*child)->totalCost = (*child)->pathCost + m_heuristic->GetHeuristicValue((*child)->state);

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
			if ((*frontierNode)->totalCost > (*child)->totalCost) {
				m_frontier.Remove(frontierNode);
				m_frontier.Push(child);
				node->AddChild(std::move(childScope));
			}
		}

	protected:
		Ref<AStarStatePropagator<State, Action>> m_propagator;
		Ref<AStarHeuristic<State>> m_heuristic;

		FrontierContainer m_frontier;
		ExploredContainer m_explored;

		Scope<Node> m_rootNode;
		Node* m_solutionNode = nullptr;

	private:
		bool isInitialized = false;
	};

	/// @brief Interface for bidirectional search
	template <
		typename State,
		typename Action,
		typename HashState = std::hash<State>,
		typename EqualState = std::equal_to<State>,
		bool GraphSearch = true,
		typename UnidirectionalAStar = AStar<State, Action, HashState, EqualState, GraphSearch>>
	class BidirectionalAStarBase : public PathPlanner<State> {
		static_assert(IsAStar<UnidirectionalAStar, State, Action, HashState, EqualState, GraphSearch>::value, "The unidirectional search algorithm is not an A* algorithm.");

	public:
		using Explored = typename UnidirectionalAStar::Explored;
		using Frontier = typename UnidirectionalAStar::Frontier;
		using Node = typename UnidirectionalAStar::Node;

	public:
		BidirectionalAStarBase() = default;
		template <typename... Args>
		BidirectionalAStarBase(Args... args) :
			m_fSearch(std::forward<Args>(args)...), m_rSearch(std::forward<Args>(args)...) { }
		virtual ~BidirectionalAStarBase() = default;

		/// @copydoc Planner::PathPlanner::GetPath
		virtual std::vector<State> GetPath() const override
		{
			auto fPath = m_fSearch.GetPath();
			auto rPath = m_rSearch.GetPath();
			fPath.insert(fPath.end(), rPath.rbegin(), rPath.rend());
			return fPath;
		}

		/// @brief Return the actions from the initial state to the goal state.
		std::vector<Action> GetActions() const
		{
			auto fActions = m_fSearch.GetActions();
			auto rActions = m_rSearch.GetActions();
			fActions.insert(fActions.end(), rActions.rbegin(), rActions.rend());
			return fActions;
		}

		/// @brief Return the set of explored states.
		std::tuple<const Explored&, const Explored&> GetExploredStates() const
		{
			return std::tie(m_fSearch.GetExploredStates(), m_rSearch.GetExploredStates());
		}

		/// @brief Return the optimal cost
		double GetOptimalCost() const
		{
			return m_fSearch.GetOptimalCost() + m_rSearch.GetOptimalCost();
		}

		/// @Brief Return the state-propagator used by the algorithm.
		std::tuple<const Ref<AStarStatePropagator<State, Action>>&, const Ref<AStarStatePropagator<State, Action>>&> GetStatePropagators() const
		{
			return std::tie(m_fSearch.GetStatePropagator(), m_rSearch.GetStatePropagator());
		}

		/// @brief Return the heuristic used by the algorithm
		std::tuple<const Ref<AStarHeuristic<State>>&, const Ref<AStarHeuristic<State>>&> GetHeuristics() const
		{
			return std::tie(m_fSearch.GetHeuristic(), m_rSearch.GetHeuristic());
		}

		/// @brief Initialize A* search.
		/// @param fPropagator Expand the a given node in the forward search.
		/// @param rPropagator Expand the a given node in the reverse search.
		/// @param fHeuristic Heuristic function to guide the forward search.
		/// @param rHeuristic Heuristic function to guide the reverse search.
		bool Initialize(const Ref<AStarStatePropagator<State, Action>>& fPropagator, const Ref<AStarStatePropagator<State, Action>>& rPropagator,
			const Ref<AStarHeuristic<State>>& fHeuristic, const Ref<AStarHeuristic<State>>& rHeuristic)
		{
			// Edge case: if the two heuristic given refer to the same object, the goal will not be updated correctly
			if (fHeuristic.get() == rHeuristic.get())
				throw std::invalid_argument("The forward and reverse heuristic cannot refer to the same object.");

			if (!m_fSearch.Initialize(fPropagator, fHeuristic))
				return m_isInitialized = false;
			if (!m_rSearch.Initialize(rPropagator, rHeuristic))
				return m_isInitialized = false;

			return m_isInitialized = true;
		}

	protected:
		bool InitializeSearch()
		{
			if (!m_isInitialized) {
				PP_ERROR("The algorithm has not been initialized successfully.");
				return false;
			}

			m_fSearch.SetInitState(this->m_init);
			m_rSearch.SetInitState(this->m_goal);
			m_fSearch.SetGoalState(this->m_goal);
			m_rSearch.SetGoalState(this->m_init);

			m_fSearch.InitializeSearch();
			m_rSearch.InitializeSearch();

			return true;
		}

		inline void ExpandForward(Node* node) { m_fSearch.Expand(node); }
		inline void ExpandReverse(Node* node) { m_rSearch.Expand(node); }

		inline std::tuple<Explored&, Explored&> GetExploredSets() { return std::tie(m_fSearch.m_explored, m_rSearch.m_explored); }
		inline std::tuple<Frontier&, Frontier&> GetFrontiers() { return std::tie(m_fSearch.m_frontier, m_rSearch.m_frontier); }
		inline std::tuple<Node*&, Node*&> GetSolutionNodes() { return std::tie(m_fSearch.m_solutionNode, m_rSearch.m_solutionNode); }

	protected:
		UnidirectionalAStar m_fSearch, m_rSearch;

	private:
		bool m_isInitialized = false;
	};
}
