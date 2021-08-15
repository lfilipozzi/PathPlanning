cmake_minimum_required(VERSION 3.0)

project(pathplanning)

add_executable(pathplanning main.cpp)

install(TARGETS pathplanning RUNTIME DESTINATION bin)
