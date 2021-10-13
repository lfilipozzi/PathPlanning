#pragma once

#ifdef PP_ENABLE_LOG

	#include "core/base.h"
	#include <spdlog/spdlog.h>

namespace Planner {

	///@brief Logging system
	class Log {
	public:
		static void Init();

		inline static Ref<spdlog::logger>& GetLogger() { return s_logger; }

	private:
		static bool s_initialized;
		static Ref<spdlog::logger> s_logger;
	};

}

	#define PP_INIT_LOGGER ::Planner::Log::Init()
	#define PP_TRACE(...) ::Planner::Log::GetLogger()->trace(__VA_ARGS__)
	#define PP_INFO(...) ::Planner::Log::GetLogger()->info(__VA_ARGS__)
	#define PP_WARN(...) ::Planner::Log::GetLogger()->warn(__VA_ARGS__)
	#define PP_ERROR(...) ::Planner::Log::GetLogger()->error(__VA_ARGS__)
	#define PP_CRITICAL(...) ::Planner::Log::GetLogger()->critical(__VA_ARGS__)
#else
	#define PP_INIT_LOGGER
	#define PP_TRACE(...)
	#define PP_INFO(...)
	#define PP_WARN(...)
	#define PP_ERROR(...)
	#define PP_CRITICAL(...)
#endif
