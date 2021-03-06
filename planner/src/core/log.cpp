#include "core/log.h"
#include "core/base.h"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

namespace Planner {

	spdlog::logger&& Log::CreateLogger()
	{
		std::vector<spdlog::sink_ptr> logSinks;
		logSinks.emplace_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
		logSinks.emplace_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("path_planner.log", true));

		logSinks[0]->set_pattern("%^[%T] %n: %v%$");
		logSinks[1]->set_pattern("[%T] [%l] %n: %v");

		auto logger = std::make_shared<spdlog::logger>("Path-Planner", begin(logSinks), end(logSinks));
		spdlog::register_logger(logger);
		logger->set_level(spdlog::level::trace);
		logger->flush_on(spdlog::level::trace);

		return std::move(*logger);
	}

}
