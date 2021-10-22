#pragma once

namespace Planner {
	/// @details Struct to allow constructing a shared pointer of a class whose
	/// constructor is private or protected.
	template <typename T>
	struct MakeRefEnabler : public T {
		template <typename... Args>
		MakeRefEnabler(Args&&... args) :
			T(std::forward<Args>(args)...) { }
	};
};
