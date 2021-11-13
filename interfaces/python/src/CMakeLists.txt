cmake_minimum_required(VERSION 3.5)
project(pyplanning)

find_package(pybind11 REQUIRED)
pybind11_add_module(pyplanning pyplanning.cpp)
target_link_libraries(pyplanning PRIVATE pathPlanning)

# Rename macro to support versions of pybind11 older than v2.5.0
if (pybind11_VERSION_MAJOR VERSION_LESS "2.5.0")
	target_compile_definitions(pyplanning PRIVATE PYBIND11_OVERRIDE_PURE=PYBIND11_OVERLOAD_PURE)
	target_compile_definitions(pyplanning PRIVATE PYBIND11_OVERRIDE=PYBIND11_OVERLOAD)
endif()
