#pragma once

#include <memory>

namespace Planner {

	/// @brief Unique pointer
	template <typename T>
	using Scope = std::unique_ptr<T>;
	template <typename T, typename... Args>
	constexpr Scope<T> makeScope(Args&&... args)
	{
		return std::make_unique<T>(std::forward<Args>(args)...);
	}

	/// @brief Shared pointer
	template <typename T>
	using Ref = std::shared_ptr<T>;
	template <typename T, typename... Args>
	constexpr Ref<T> makeRef(Args&&... args)
	{
		return std::make_shared<T>(std::forward<Args>(args)...);
	}

}

#define PP_CONCATENATE(arg1, arg2) PP_INTERNAL_CONCATENATE1(arg1, arg2)
#define PP_INTERNAL_CONCATENATE1(arg1, arg2) PP_INTERNAL_CONCATENATE2(arg1, arg2) // Defer concatenation
#define PP_INTERNAL_CONCATENATE2(arg1, arg2) arg1##arg2

// Iterate over variadic macro
// clang-format off
#define PP_INTERNAL_FOR_EACH_1(WHAT, x, ...) WHAT(x)
#define PP_INTERNAL_FOR_EACH_2(WHAT, x, ...) WHAT(x)PP_INTERNAL_FOR_EACH_1(WHAT, __VA_ARGS__)
#define PP_INTERNAL_FOR_EACH_3(WHAT, x, ...) WHAT(x)PP_INTERNAL_FOR_EACH_2(WHAT, __VA_ARGS__)
#define PP_INTERNAL_FOR_EACH_4(WHAT, x, ...) WHAT(x)PP_INTERNAL_FOR_EACH_3(WHAT, __VA_ARGS__)
#define PP_INTERNAL_FOR_EACH_5(WHAT, x, ...) WHAT(x)PP_INTERNAL_FOR_EACH_4(WHAT, __VA_ARGS__)
#define PP_INTERNAL_FOR_EACH_6(WHAT, x, ...) WHAT(x)PP_INTERNAL_FOR_EACH_5(WHAT, __VA_ARGS__)
#define PP_INTERNAL_FOR_EACH_7(WHAT, x, ...) WHAT(x)PP_INTERNAL_FOR_EACH_6(WHAT, __VA_ARGS__)
#define PP_INTERNAL_FOR_EACH_8(WHAT, x, ...) WHAT(x)PP_INTERNAL_FOR_EACH_7(WHAT, __VA_ARGS__)
#define PP_INTERNAL_FOR_EACH_NARG(...) PP_INTERNAL_FOR_EACH_NARG_(__VA_ARGS__, PP_INTERNAL_FOR_EACH_RSEQ_N())
#define PP_INTERNAL_FOR_EACH_NARG_(...) PP_INTERNAL_FOR_EACH_ARG_N(__VA_ARGS__)
#define PP_INTERNAL_FOR_EACH_ARG_N(_1, _2, _3, _4, _5, _6, _7, _8, N, ...) N
#define PP_INTERNAL_FOR_EACH_RSEQ_N() 8, 7, 6, 5, 4, 3, 2, 1, 0
// clang-format on
#define PP_INTERNAL_FOR_EACH_(N, WHAT, ...) \
	PP_CONCATENATE(PP_INTERNAL_FOR_EACH_, N)   \
	(WHAT, __VA_ARGS__)
#define PP_FOR_EACH(WHAT, ...) PP_INTERNAL_FOR_EACH_(PP_INTERNAL_FOR_EACH_NARG(__VA_ARGS__), WHAT, __VA_ARGS__)

// Platform detection
#include "core/platform_detection.h"

// Platform specific macro
#if defined(PP_PLATFORM_WINDOWS)
	// Microsoft
	#define PP_DEBUG_BREAK __debugbreak()
#elif defined(PP_PLATFORM_APPLE)
	// Apple
	#define PP_DEBUG_BREAK
#elif defined(PP_PLATFORM_LINUX)
	// Linux
	#include <signal.h>
	#define PP_DEBUG_BREAK raise(SIGTRAP)
#else
	#define PP_DEBUG_BREAK
	#warning "Platform doesn't support debugbreak yet!"
#endif // End of platform specific macro

#include "core/log.h"
#include "core/assert.h"
