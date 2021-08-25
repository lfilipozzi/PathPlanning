cmake_minimum_required(VERSION 3.0)
project(pathPlanning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LOG_WITH_SPDLOG "Log error with spdlog" ON)
if(LOG_WITH_SPDLOG)
	message("Enable logging")
	add_compile_definitions(PP_ENABLE_LOG)
	include_directories(${CMAKE_SOURCE_DIR}/external/spdlog/include)
	add_library(logger ${CMAKE_SOURCE_DIR}/planner/src/core/log.cpp)
	set(logger_LIB logger)
else()
	message("Disable logging")
	set(logger_LIB)
endif()

option(ENABLE_ASSERT "Enable assertion" ON)
if(ENABLE_ASSERT)
	message("Enable assertion")
	add_compile_definitions(PP_ENABLE_ASSERT)
endif()

include_directories(
	${CMAKE_SOURCE_DIR}/planner/src
)

find_path(flannIncludeDir "flann/flann.hpp")
find_library(flannLibrary "flann" "/usr/lib" "/usr/local/lib")
if(flannInclude AND flannLibrary)
	add_library(flann INTERFACE)
	#target_include_directories(flann INTERFACE ${flannIncludeDir})
	#target_link_libraries(foo INTERFACE ${flannLibrary})
else()
	#... # read on
endif()

add_executable(pathPlanning
	main.cpp
)
target_link_libraries(${PROJECT_NAME} flann ${logger_LIB})

enable_testing()
add_subdirectory(planner/tests)
