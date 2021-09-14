#include "core/base.h"
#include "rrt.h"
#include "geometry/2dplane.h"
#include "core/log.h"

namespace Planner {
	void TestRRTPath()
	{
		Planner::Ref<TestRRTStateSpace> stateSpace = Planner::makeRef<TestRRTStateSpace>();

		Planner::RRT<Vertex, 2> rrt(stateSpace);
		Planner::RRT<Vertex, 2>::Parameters parameters;
		rrt.SetParameters(parameters);

		Vertex start = { 0.0, 0.0 };
		Vertex goal = { 2.0, 2.0 };
		rrt.SetInitState(start);
		rrt.SetGoalState(goal);

		rrt.SearchPath();
		std::vector<Vertex> path = rrt.GetPath();

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
