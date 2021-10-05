#pragma once

#include "state_validator.h"

namespace Planner {

	/**
	 * @brief State validator approving any state and path.
	 */
	template <typename State>
	class StateValidatorFree : public StateValidator<State> {
	public:
		StateValidatorFree() = default;
		~StateValidatorFree() = default;

		virtual bool IsStateValid(const State& /*state*/) override
		{
			return true;
		}

		virtual bool IsPathValid(const State& /*from*/, const State& to, State* last = nullptr) override
		{
			if (last)
				*last = to;
			return true;
		}
	};
}
