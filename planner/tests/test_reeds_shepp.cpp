#include "core/log.h"
#include "reeds_shepp.h"

namespace Planner {
	void Test()
	{
		Pose2D<> start = { 1.0, 1.0, 1.6 };
		Pose2D<> goal = { -3.0, 1.0, 3.1 };
		float unit = 1.0f;
		auto path = ReedsSheep::Solver::GetShortestPath(start, goal, unit);
		PP_INFO(path.ComputeCost(1.0f, 1.0f, 0.0f));
	}
}

int main()
{
	PP_INIT_LOGGER;
	
	Planner::Test();
	
	return 0;
}
