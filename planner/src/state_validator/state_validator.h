#pragma once

#include "paths/path.h"

namespace Planner {

	/// @brief Validate states and paths.
	template <typename State>
	class StateValidator {
	public:
		StateValidator() = default;
		virtual ~StateValidator() = default;

		/// @brief Check if a state is valid.
		virtual bool IsStateValid(const State& state) = 0;

		/// @brief Check if a path is valid
		/// @param[in] path The path to validate
		/// @param[out] last The ratio to interpolate to the last valid state along the path.
		/// @return A boolean indicating if the path is valid.
		virtual bool IsPathValid(const Path<State>& path, float* last = nullptr) = 0;

		/// @overload
		/// @param[in] path The path to validate
		/// @param[out] last A pointer to the last valid state along the path.
		/// @return A boolean indicating if the path is valid.
		bool IsPathValid(const Path<State>& path, State* last)
		{
			float ratio;
			bool validated = IsPathValid(path, ratio);
			if (last)
				*last = path.Interpolate(ratio);
			return validated;
		}
	};
}
