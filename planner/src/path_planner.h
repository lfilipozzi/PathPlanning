#pragma once

#include <vector>

#include "state_space.h"

namespace Planner {

	/**
	 * @brief Solve path planning problems.
	 * @details Templated class where Vertex is the type of the vertices. Its 
	 * data must be arranged contiguously in memory. Vertex must be copy 
	 * assignable and copy constructible. A specialized template of std::hash 
	 * for Vertex must be defined.
	 */
	template <typename Vertex>
	class PathPlanner {
	public:
		PathPlanner() {}
		virtual ~PathPlanner() = default;

		/**
		 * @brief Search for the optimal collision-free path.
		 */
		virtual void SearchPath() = 0;

		/**
		 * @brief Return the optimal collision-free path.
		 */
		virtual std::vector<Vertex> GetPath() = 0;

		void SetInitState(const Vertex& init) { m_init = init; }
		void SetGoalState(const Vertex& goal) { m_goal = goal; }
		const Vertex& GetInitState() const { return m_init; }
		const Vertex& GetGoalState() const { return m_goal; }
		Vertex& GetInitState() { return m_init; }
		Vertex& GetGoalState() { return m_goal; }

	protected:
		Vertex m_init;
		Vertex m_goal;
	};
}
