cmake_minimum_required(VERSION 3.0)

include_directories(
	${CMAKE_SOURCE_DIR}/planner/src
	${CMAKE_SOURCE_DIR}/planner/tests
)

# Set up unit testing
add_executable(testTree test_tree.cpp)
target_link_libraries(testTree flann)
add_test(testTree testTree)

add_executable(testRRT test_rrt.cpp)
target_link_libraries(testRRT flann)
add_test(testRRT testRRT)

add_executable(testRRTStar test_rrt_star.cpp)
target_link_libraries(testRRTStar flann)
add_test(testRRTStar testRRTStar)

add_executable(testAStar test_a_star.cpp)
target_link_libraries(testAStar flann)
add_test(testAStar testAStar)
