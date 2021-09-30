#include "core/log.h"
#include "reeds_shepp.h"

namespace Planner {
	void Test(const Pose2D<>& start, const Pose2D<>& goal)
	{
		float unit = 1.0f;
		
		float t, u, v;
		auto word = ReedsSheep::Solver::GetShortestPathWord(start, goal, unit, t, u, v);
		
		auto path = ReedsSheep::Solver::GetShortestPath(start, goal, unit);
		
		PP_INFO("{}: {}", word, path.ComputeCost(1.0f, 1.0f, 0.0f));
	}
}

int main()
{
	PP_INIT_LOGGER;
	
	Planner::Pose2D<> start;
	Planner::Pose2D<> goal;
	
	// 0
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, 0.0, 0.0 };
	Planner::Test(start, goal);
	
	// 1: 
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, 1.6 };
	Planner::Test(start, goal);
	
	// 4
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, -5.0, -1.6 };
	Planner::Test(start, goal);
	
	// 5
	start = { -1.0, 1.0, 1.6 };
	goal = { 1.0, 1.0, -1.6 };
	Planner::Test(start, goal);
	
	// 6:
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, -1.6 };
	Planner::Test(start, goal);
	
	// 7
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 5.0, -3.1 };
	Planner::Test(start, goal);
	
	// 12
	start = { 1.0, 1.0, 1.6 };
	goal = { -1.0, -1.0, 1.6 };
	Planner::Test(start, goal);
	
	// 13
	start = { 1.0, 1.0, 1.6 };
	goal = { 3.0, -1.0, 1.6 };
	Planner::Test(start, goal);
	
	// 14
	start = { 1.0, -1.0, 1.6 };
	goal = { -1.0, 1.0, 1.6 };
	Planner::Test(start, goal);
	
	// 15
	start = { 1.0, -1.0, 1.6 };
	goal = { 3.0, 1.0, 1.6 };
	Planner::Test(start, goal);
	
	// 24
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.0, -1.0, 0.0 };
	Planner::Test(start, goal);
	
	// 25
	start = { 0.0, 0.0, 1.57 };
	goal = { 0.1, 0.0, 0.0 };
	Planner::Test(start, goal);
	
	// 26
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.0, 1.0, 0.0 };
	Planner::Test(start, goal);
	
	// 27
	start = { 0.0, 0.0, -1.57 };
	goal = { 0.1, 0.0, 0.0 };
	Planner::Test(start, goal);
	
	// 44: 
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, 3.1 };
	Planner::Test(start, goal);
	
	// 45: 
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, -3.1 };
	Planner::Test(start, goal);
	
	// 46
	start = { 1.0, 1.0, -1.6 };
	goal = { -3.0, 1.0, -3.1 };
	Planner::Test(start, goal);
	
	// 47: 
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, 0 };
	Planner::Test(start, goal);
	
	return 0;
}
