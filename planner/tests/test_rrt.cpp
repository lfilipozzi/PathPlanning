#include "core/base.h"
#include "algo/rrt.h"
#include "geometry/2dplane.h"
#include "state_space/rrt_state_space_2d.h"

namespace Planner {
	void TestRRTPath()
	{
		auto stateSpace = Planner::makeRef<RRTStateSpace2D>();

		Planner::RRT<Point2d, 2> rrt(stateSpace);
		Planner::RRTParameters parameters;
		rrt.SetParameters(parameters);

		Point2d start = { 0.0, 0.0 };
		Point2d goal = { 2.0, 2.0 };
		rrt.SetInitState(start);
		rrt.SetGoalState(goal);

		rrt.SearchPath();
		std::vector<Point2d> path = rrt.GetPath();

		assert(!path.empty());
		assert(stateSpace->ComputeDistance(start, path.front()) <= parameters.optimalSolutionTolerance);
		assert(stateSpace->ComputeDistance(goal, path.back()) <= parameters.optimalSolutionTolerance);
	}
}

int main()
{
	PP_INIT_LOGGER;
	Planner::TestRRTPath();

	return 0;
}
