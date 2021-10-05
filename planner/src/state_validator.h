#pragma once

namespace Planner {

	/**
	 * @brief Validate states and paths.
	 */
	template <typename State>
	class StateValidator {
	public:
		StateValidator() = default;
		~StateValidator() = default;

		/// @brief Check if a state is valid.
		virtual bool IsStateValid(const State& state) = 0;

		/// @brief Check if a path is valid
		/// @param[in] from The initial state
		/// @param[in] to The terminal state
		/// @param[out] last A pointer to the last valid state along the path.
		/// @return A boolean indicating if the path is valid.
		virtual bool IsPathValid(const State& from, const State& to, State* last = nullptr) = 0;
	};
}
