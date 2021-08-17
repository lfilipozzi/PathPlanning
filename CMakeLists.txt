cmake_minimum_required(VERSION 3.0)

project(pathPlanning)

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
target_link_libraries(${PROJECT_NAME} flann)
