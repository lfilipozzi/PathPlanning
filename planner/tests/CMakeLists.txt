cmake_minimum_required(VERSION 3.0)

include_directories(
	${CMAKE_SOURCE_DIR}/planner/src
	${CMAKE_SOURCE_DIR}/planner/tests
)

# Set up unit testing
add_executable(testTree test_tree.cpp)
target_link_libraries(testTree pathPlanning)
add_test(testTree testTree)

add_executable(testRRT test_rrt.cpp)
target_link_libraries(testRRT pathPlanning)
add_test(testRRT testRRT)

add_executable(testRRTStar test_rrt_star.cpp)
target_link_libraries(testRRTStar pathPlanning)
add_test(testRRTStar testRRTStar)

add_executable(testAStar test_a_star.cpp)
target_link_libraries(testAStar pathPlanning)
add_test(testAStar testAStar)

add_executable(testHybridAStar test_hybrid_a_star.cpp)
target_link_libraries(testHybridAStar pathPlanning)
add_test(testHybridAStar testHybridAStar)

add_executable(testReedsShepp test_reeds_shepp.cpp)
target_link_libraries(testReedsShepp pathPlanning)
add_test(testReedsShepp testReedsShepp)
