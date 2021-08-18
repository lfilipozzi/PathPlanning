#include "base.h"
#include "rrt.h"
#include "2dplane.h"

namespace Planner {
	void TestRRTPath()
	{
		Planner::Scope<TestStateSpace> stateSpaceScope = Planner::makeScope<TestStateSpace>();
		Planner::Scope<TestStateValidator> validatorScope = Planner::makeScope<TestStateValidator>();
		TestStateSpace* stateSpace = stateSpaceScope.get();

		Planner::RRT<Vertex, 2> rrt(std::move(stateSpaceScope), std::move(validatorScope));
		Planner::RRT<Vertex, 2>::Parameters parameters;
		rrt.SetParameters(parameters);

		Vertex start = { 0.0, 0.0 };
		Vertex goal = { 2.0, 2.0 };
		rrt.SetInitState(start);
		rrt.SetGoalState(goal);

		rrt.SearchPath();
		std::vector<Vertex> path = rrt.GetPath();

		assert(stateSpace->ComputeDistance(start, path.front()) <= parameters.optimalSolutionTolerance);
		assert(stateSpace->ComputeDistance(goal, path.back()) <= parameters.optimalSolutionTolerance);
	}
}

int main()
{
	Planner::TestRRTPath();

	return 0;
}
