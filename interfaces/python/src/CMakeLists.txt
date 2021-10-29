cmake_minimum_required(VERSION 3.5)
project(pyplanning)

find_package(PythonLibs 3.7 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

set(CMAKE_SHARED_MODULE_PREFIX "")

add_library(pyplanning MODULE pyplanning.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(pyplanning pathPlanning ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pyplanning PRIVATE ${PYTHON_INCLUDE_DIRS})
