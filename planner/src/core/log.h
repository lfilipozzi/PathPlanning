#pragma once

#ifdef PP_ENABLE_LOG

	#include "core/base.h"
	#include "utils/singleton.h"
	#include <spdlog/spdlog.h>
	#include <memory>

namespace Planner {

	/// @brief Logging system
	class Log : public Singleton<Log>, public spdlog::logger {
		friend class Singleton<Log>;

	public:
		~Log() = default;

	private:
		Log() :
			spdlog::logger(CreateLogger()) { }

		/// @brief Define the logger and create the list of sink associated to it.
		static spdlog::logger&& CreateLogger();
	};

}

	#define PP_INIT_LOGGER ::Planner::Log::Init()
	#define PP_TRACE(...) ::Planner::Log::Get().trace(__VA_ARGS__)
	#define PP_INFO(...) ::Planner::Log::Get().info(__VA_ARGS__)
	#define PP_WARN(...) ::Planner::Log::Get().warn(__VA_ARGS__)
	#define PP_ERROR(...) ::Planner::Log::Get().error(__VA_ARGS__)
	#define PP_CRITICAL(...) ::Planner::Log::Get().critical(__VA_ARGS__)
#else
	#define PP_INIT_LOGGER
	#define PP_TRACE(...)
	#define PP_INFO(...)
	#define PP_WARN(...)
	#define PP_ERROR(...)
	#define PP_CRITICAL(...)
#endif
