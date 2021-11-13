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
	set_target_properties(logger PROPERTIES POSITION_INDEPENDENT_CODE ON)
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

# Define build directory
set(CMAKE_BUILD_DIR ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BUILD_DIR}/lib)
message(STATUS "EXECUTABLE_OUTPUT_PATH: " ${EXECUTABLE_OUTPUT_PATH})
message(STATUS "LIBRARY_OUTPUT_PATH: " ${LIBRARY_OUTPUT_PATH})
message(STATUS "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})
file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})

include_directories(
	${CMAKE_SOURCE_DIR}/planner/src
)

message(STATUS "Finding flann libraries")
find_library(LZ4_LIBRARIES NAMES liblz4.so HINTS "lz4/lib")
if(LZ4_LIBRARIES)
	message("LZ4 libraries: ${LZ4_LIBRARIES}")
else()
	message("LZ4 libraries: ${LZ4_LIBRARIES}")
	message(FATAL_ERROR "Cannot find required LZ4 libraries")
endif()
find_path(FLANN_INCLUDE_DIR "flann/flann.hpp")
find_library(FLANN_LIBRARIES NAMES "flann" PATHS "/usr/lib" "/usr/lib/x86_64-linux-gnu" "/usr/local/lib")
if(FLANN_INCLUDE_DIR AND FLANN_LIBRARIES)
	message("Flann include directory: " ${FLANN_INCLUDE_DIR})
	message("Flann libraries: " ${FLANN_LIBRARIES})
	add_library(flann INTERFACE)
	target_include_directories(flann INTERFACE ${FLANN_INCLUDE_DIR})
	target_link_libraries(flann INTERFACE ${FLANN_LIBRARIES} ${LZ4_LIBRARIES})
else()
	message("Flann include directory: " ${FLANN_INCLUDE_DIR})
	message("Flann libraries: " ${FLANN_LIBRARIES})
	message(FATAL_ERROR "Cannot find required flann libraries")
endif()

add_library(pathPlanning
	${CMAKE_SOURCE_DIR}/planner/src/algo/hybrid_a_star.cpp
	${CMAKE_SOURCE_DIR}/planner/src/algo/heuristics.cpp
	${CMAKE_SOURCE_DIR}/planner/src/algo/planar_a_star_grid.cpp
	${CMAKE_SOURCE_DIR}/planner/src/algo/smoother.cpp
	${CMAKE_SOURCE_DIR}/planner/src/geometry/2dplane.cpp
	${CMAKE_SOURCE_DIR}/planner/src/geometry/reeds_shepp.cpp
	${CMAKE_SOURCE_DIR}/planner/src/models/kinematic_bicycle_model.cpp
	${CMAKE_SOURCE_DIR}/planner/src/paths/path_constant_steer.cpp
	${CMAKE_SOURCE_DIR}/planner/src/paths/path_reeds_shepp.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_space/state_space.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/gvd.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/obstacle.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/obstacle_list_occupancy_map.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/occupancy_map.cpp
	${CMAKE_SOURCE_DIR}/planner/src/state_validator/state_validator_occupancy_map.cpp
	${CMAKE_SOURCE_DIR}/planner/src/utils/grid.cpp
)
set_target_properties(pathPlanning PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(pathPlanning flann ${logger_LIB})

add_subdirectory(interfaces/python/src)

add_executable(mainPathPlanning main.cpp)
target_link_libraries(mainPathPlanning pathPlanning)

enable_testing()
add_subdirectory(planner/tests)
