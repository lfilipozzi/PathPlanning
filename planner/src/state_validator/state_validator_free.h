#pragma once

#include "state_validator/state_validator.h"

namespace Planner {

	/// @brief State validator approving any state and path.
	template <typename State, int Dimension, typename T>
	class StateValidatorFree : public StateValidator<State, Dimension, T> {
	public:
		StateValidatorFree(const Ref<StateSpace<State, Dimension, T>>& stateSpace) :
			StateValidator<State, Dimension, T>(stateSpace) { }
		~StateValidatorFree() = default;

		/// @copydoc Planner::StateValidator::IsStateValid
		virtual bool IsStateValid(const State& state) override
		{
			// Validate bounds
			if (!this->stateSpace->ValidateBounds(state))
				return false;
			return true;
		}

		/// @copydoc Planner::StateValidator::IsPathValid
		virtual bool IsPathValid(const Path<State>& /*path*/, float* last = nullptr) override
		{
			if (last)
				*last = 1.0;
			return true;
		}
	};
}
