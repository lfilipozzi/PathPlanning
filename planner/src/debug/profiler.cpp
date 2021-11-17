#include "debug/profiler.h"
// #include "core/base.h"
#include "core/log.h"
// #include "core/assert.h"

namespace Planner {

	void Profiler::BeginSession(const std::string& name, const std::string& filepath)
	{
		std::lock_guard lock(m_mutex);
		if (m_currentSession) {
			// If there is already a current session, then close it before
			// beginning new one. Subsequent profiling output meant for the
			// original session will end up in the newly opened session
			// instead.  That's better than having badly formatted
			// profiling output.
			if (Log::GetLogger()) {
				PP_ERROR(
					"Profiler::BeginSession('{0}') when session '{1}' already open.",
					name, m_currentSession->name);
			}
			InternalEndSession();
		}
		m_outputStream.open(filepath);
		if (m_outputStream.is_open()) {
			m_currentSession = new ProfilingSession({ name });
			WriteHeader();
		} else {
			if (Log::GetLogger()) {
				// Edge case: BeginSession() might be before Log::Init()
				PP_ERROR("Profiler could not open results file '{0}'.", filepath);
			}
		}
	}

	void Profiler::EndSession()
	{
		std::lock_guard lock(m_mutex);
		InternalEndSession();
	}

	void Profiler::WriteProfile(const ProfilingResult& result)
	{
		std::stringstream json;

		json << ",\n{";
		json << "\"cat\":\"function\",";
		json << "\"dur\":" << (result.end - result.start) << ',';
		json << "\"name\":\"" << result.name << "\",";
		json << "\"ph\":\"X\",";
		json << "\"pid\":0,";
		json << "\"tid\":" << result.threadID << ",";
		json << "\"ts\":" << result.start;
		json << "}";

		std::lock_guard lock(m_mutex);
		if (m_currentSession) {
			m_outputStream << json.str();
			m_outputStream.flush();
		}
	}

	void Profiler::WriteHeader()
	{
		m_outputStream << "{\"otherData\": {},\"traceEvents\":[{}";
		m_outputStream.flush();
	}

	void Profiler::WriteFooter()
	{
		m_outputStream << "]}";
		m_outputStream.flush();
	}

	void Profiler::InternalEndSession()
	{
		if (m_currentSession) {
			WriteFooter();
			m_outputStream.close();
			delete m_currentSession;
			m_currentSession = nullptr;
		}
	}

}
