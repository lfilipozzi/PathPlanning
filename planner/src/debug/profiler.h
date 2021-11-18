#pragma once

#ifdef PP_ENABLE_PROFILING

	#include <string>
	#include <chrono>
	#include <algorithm>
	#include <fstream>
	#include <thread>
	#include <mutex>

	#include "core/base.h"
	#include "utils/singleton.h"

namespace Planner {

	/// @brief Store profiling information
	struct ProfilingResult {
		std::string name;
		long long start, end;
		std::thread::id threadID;
	};

	/// @brief Store profiling session information
	struct ProfilingSession {
		std::string name;
	};

	/// @brief Profiler
	/// @details Create a JSON file that keep trace information. The file can be
	/// read using chrome://tracing.
	class Profiler : public Singleton<Profiler> {
		friend class Singleton<Profiler>;

	public:
		~Profiler()
		{
			EndSession();
		}

		Profiler(const Profiler& other) = delete;
		Profiler(Profiler&& other) = delete;

		void WriteProfile(const ProfilingResult& result);

	private:
		Profiler()
		{
			BeginSession("Runtime");
		}

		void BeginSession(const std::string& name, const std::string& filepath = "profiling.json");

		void EndSession();

		void WriteHeader();
		void WriteFooter();
		void InternalEndSession();

	private:
		std::mutex m_mutex;
		ProfilingSession* m_currentSession = nullptr;
		std::ofstream m_outputStream;
	};

	/// @brief Timer used for profiling
	class ProfilingTimer {
	public:
		ProfilingTimer(const char* name) :
			m_name(name), m_stopped(false)
		{
			m_startTimepoint = std::chrono::high_resolution_clock::now();
		}

		~ProfilingTimer()
		{
			if (!m_stopped)
				Stop();
		}

		void Stop()
		{
			auto endTimepoint = std::chrono::high_resolution_clock::now();

			long long start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTimepoint).time_since_epoch().count();
			long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();

			Profiler::Get().WriteProfile({ m_name, start, end, std::this_thread::get_id() });

			m_stopped = true;
		}

	private:
		const char* m_name;
		std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimepoint;
		bool m_stopped;
	};

	namespace ProfilerUtilities {

		template <size_t N>
		struct ChangeResult {
			char Data[N];
		};

		template <size_t N, size_t K>
		constexpr auto CleanupOutputString(const char (&expr)[N], const char (&remove)[K])
		{
			ChangeResult<N> result = {};

			size_t srcIndex = 0;
			size_t dstIndex = 0;
			while (srcIndex < N) {
				size_t matchIndex = 0;
				while (matchIndex < K - 1 && srcIndex + matchIndex < N - 1 && expr[srcIndex + matchIndex] == remove[matchIndex])
					matchIndex++;
				if (matchIndex == K - 1)
					srcIndex += matchIndex;
				result.Data[dstIndex++] = expr[srcIndex] == '"' ? '\'' : expr[srcIndex];
				srcIndex++;
			}
			return result;
		}
	}
}

	// Resolve which function signature macro will be used.
	#if defined(__GNUC__) || (defined(__MWERKS__) && (__MWERKS__ >= 0x3000)) || (defined(__ICC) && (__ICC >= 600)) || defined(__ghs__)
		#define PP_FUNC_SIG __PRETTY_FUNCTION__
	#elif defined(__DMC__) && (__DMC__ >= 0x810)
		#define PP_FUNC_SIG __PRETTY_FUNCTION__
	#elif (defined(__FUNCSIG__) || (_MSC_VER))
		#define PP_FUNC_SIG __FUNCSIG__
	#elif (defined(__INTEL_COMPILER) && (__INTEL_COMPILER >= 600)) || (defined(__IBMCPP__) && (__IBMCPP__ >= 500))
		#define PP_FUNC_SIG __FUNCTION__
	#elif defined(__BORLANDC__) && (__BORLANDC__ >= 0x550)
		#define PP_FUNC_SIG __FUNC__
	#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901)
		#define PP_FUNC_SIG __func__
	#elif defined(__cplusplus) && (__cplusplus >= 201103)
		#define PP_FUNC_SIG __func__
	#else
		#define PP_FUNC_SIG "PP_FUNC_SIG unknown!"
	#endif

	#define PP_INIT_PROFILER ::Planner::Profiler::Init()
	#define PP_PROFILE_INTERNAL_SCOPE_LINE2(name, line)                                                       \
		constexpr auto fixedName##line = ::Planner::ProfilerUtilities::CleanupOutputString(name, "__cdecl "); \
		::Planner::ProfilingTimer timer##line(fixedName##line.Data)
	#define PP_PROFILE_INTERNAL_SCOPE_LINE(name, line) PP_PROFILE_INTERNAL_SCOPE_LINE2(name, line)
	#define PP_PROFILE_SCOPE(name) PP_PROFILE_INTERNAL_SCOPE_LINE(name, __LINE__)
	#define PP_PROFILE_FUNCTION() PP_PROFILE_SCOPE(PP_FUNC_SIG)
#else
	#define PP_INIT_PROFILER
	#define PP_PROFILE_SCOPE(name)
	#define PP_PROFILE_FUNCTION()
#endif
