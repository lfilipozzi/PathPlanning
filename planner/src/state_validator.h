#pragma once

namespace Planner {

	/**
	 * @brief Interface to validate states.
	 */
	template <typename Vertex>
	class StateValidator {
	public:
		StateValidator() = default;
		virtual ~StateValidator() = default;

		/**
		 * @brief Check if the state intersects with an obstacle.
		 * @param state The state to check.
		 * @return True if the state is valid, false otherwise.
		 */
		virtual bool ValidateState(const Vertex& state) = 0;

		/**
		 * @brief Check if there exist a valid transition between two states.
		 * @param from The initial state.
		 * @param to The destination state.
		 * @return True if the transition is valid, false otherwise.
		 */
		virtual bool ValidateTransition(const Vertex& from, const Vertex& to) = 0;
	};
}
