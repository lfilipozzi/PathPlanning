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


