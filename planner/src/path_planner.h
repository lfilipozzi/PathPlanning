#pragma once

#include <vector>

#include "state_space.h"
#include "state_validator.h"

namespace Planner {

	/**
	 * @brief Solve path planning problems.
	 * @details Templated class where T is the type of the vertices. The data 
	 * must be doubles arranged contiguously in memory. T must be copy 
	 * assignable and copy constructible. A specialized template of std::hash 
	 * for T must be defined.
	 */
	template <typename T>
	class PathPlanner {
	public:
		PathPlanner(Scope<StateSpace<T>>&& stateSpace, Scope<StateValidator<T>>&& validator) :
			m_stateSpace(std::move(stateSpace)), m_stateValidator(std::move(validator)) { }
		virtual ~PathPlanner() = default;

		/**
		 * @brief Search for the optimal collision-free path.
		 */
		virtual void SearchPath() = 0;

		/**
		 * @brief Return the optimal collision-free path.
		 */
		virtual std::vector<T> GetPath() = 0;

		void SetInitState(const T& init) { m_init = init; }
		void SetGoalState(const T& goal) { m_goal = goal; }
		const T& GetInitState() const { return m_init; }
		const T& GetGoalState() const { return m_goal; }
		T& GetInitState() { return m_init; }
		T& GetGoalState() { return m_goal; }

	protected:
		Scope<StateSpace<T>> m_stateSpace;
		Scope<StateValidator<T>> m_stateValidator;

		T m_init;
		T m_goal;
	};
}
