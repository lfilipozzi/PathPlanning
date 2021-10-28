cmake_minimum_required(VERSION 3.5)
project(pypathPlanning)

find_package(PythonLibs 3.7 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

set(CMAKE_SHARED_MODULE_PREFIX "")

add_library(pypathPlanning MODULE
	paths.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(pypathPlanning ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pypathPlanning PRIVATE ${PYTHON_INCLUDE_DIRS})
