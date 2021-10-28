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

add_library(pathPlanning
	${CMAKE_SOURCE_DIR}/planner/src/algo/hybrid_a_star.cpp
	${CMAKE_SOURCE_DIR}/planner/src/algo/heuristics.cpp
	${CMAKE_SOURCE_DIR}/planner/src/geometry/2dplane.cpp
	${CMAKE_SOURCE_DIR}/planner/src/geometry/reeds_shepp.cpp
	${CMAKE_SOURCE_DIR}/planner/src/models/kinematic_bicycle_model.cpp
	${CMAKE_SOURCE_DIR}/planner/src/paths/path_constant_steer.cpp
	${CMAKE_SOURCE_DIR}/planner/src/paths/path_reeds_shepp.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_space/state_space.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/gvd.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/obstacle.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/binary_occupancy_map.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/occupancy_map.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/state_validator_occupancy_map.cpp
	${CMAKE_SOURCE_DIR}/planner/src/utils/grid.cpp
)
target_link_libraries(pathPlanning flann ${logger_LIB})

add_subdirectory(interfaces/python/src)

add_executable(mainPathPlanning main.cpp)
target_link_libraries(mainPathPlanning pathPlanning)

enable_testing()
add_subdirectory(planner/tests)
