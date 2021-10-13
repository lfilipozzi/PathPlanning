#include "core/base.h"
#include "algo/rrt.h"
#include "geometry/2dplane.h"
#include "core/log.h"
#include "state_space/rrt_state_space_2d.h"

namespace Planner {
	void TestRRTPath()
	{
		auto stateSpace = Planner::makeRef<RRTStateSpace2D>();

		Planner::RRT<Point2D, 2> rrt(stateSpace);
		Planner::RRTParameters parameters;
		rrt.SetParameters(parameters);

		Point2D start = { 0.0, 0.0 };
		Point2D goal = { 2.0, 2.0 };
		rrt.SetInitState(start);
		rrt.SetGoalState(goal);

		rrt.SearchPath();
		std::vector<Point2D> path = rrt.GetPath();

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
