#pragma once

#include "core/base.h"
#include "core/log.h"

#include <filesystem>

#define PP_EXPAND_MACRO(x) x
#define PP_STRINGIFY_MACRO(x) #x

#ifdef PP_ENABLE_ASSERT
	#define PP_INTERNAL_ASSERT_IMPL(type, check, msg, ...) \
		{                                                  \
			if (!(check)) {                                \
				SR##type##ERROR(msg, __VA_ARGS__);         \
				PP_DEBUG_BREAK;                            \
			}                                              \
		}
	#define PP_INTERNAL_ASSERT_WITH_MSG(type, check, ...) PP_INTERNAL_ASSERT_IMPL(type, check, "Assertion failed: {0}", __VA_ARGS__)
	#define PP_INTERNAL_ASSERT_NO_MSG(type, check) PP_INTERNAL_ASSERT_IMPL(type, check, "Assertion '{0}' failed at {1}:{2}", PP_STRINGIFY_MACRO(check), std::filesystem::path(__FILE__).filename().string(), __LINE__)

	#define PP_INTERNAL_ASSERT_GET_MACRO_NAME(arg1, arg2, macro, ...) macro
	#define PP_INTERNAL_ASSERT_GET_MACRO(...) PP_EXPAND_MACRO(PP_INTERNAL_ASSERT_GET_MACRO_NAME(__VA_ARGS__, PP_INTERNAL_ASSERT_WITH_MSG, PP_INTERNAL_ASSERT_NO_MSG))

	#define PP_ASSERT(...) PP_EXPAND_MACRO(PP_INTERNAL_ASSERT_GET_MACRO(__VA_ARGS__)(_, __VA_ARGS__))
#else
	#define PP_ASSERT(...)
#endif
