#pragma once

#include "state_validator/state_validator.h"

namespace Planner {

	/// @brief State validator approving any state and path.
	template <typename State>
	class StateValidatorFree : public StateValidator<State> {
	public:
		StateValidatorFree() = default;
		~StateValidatorFree() = default;

		/// @copydoc Planner::StateValidator::IsStateValid
		virtual bool IsStateValid(const State& /*state*/) override
		{
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
