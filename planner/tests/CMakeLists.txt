cmake_minimum_required(VERSION 3.0)

include_directories(
	${CMAKE_SOURCE_DIR}/planner/src
	${CMAKE_SOURCE_DIR}/planner/tests
)

# Set up unit testing
add_executable(testTree test_tree.cpp)
target_link_libraries(testTree flann)
add_test(testTree testTree)

add_executable(testRRT test_rrt)
target_link_libraries(testRRT flann)
add_test(testRRT testRRT)
