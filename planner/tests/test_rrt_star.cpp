#include "base.h"
#include "rrt_star.h"
#include "2dplane.h"

namespace Planner {
	void TestRRTStarPath()
	{
		Planner::Scope<TestStateSpace> stateSpaceScope = Planner::makeScope<TestStateSpace>();
		TestStateSpace* stateSpace = stateSpaceScope.get();

		Planner::RRTStar<Vertex, 2> rrtStar(std::move(stateSpaceScope));
		Planner::RRTStar<Vertex, 2>::Parameters parameters;
		rrtStar.SetParameters(parameters);

		Vertex start = { 0.0, 0.0 };
		Vertex goal = { 2.0, 2.0 };
		rrtStar.SetInitState(start);
		rrtStar.SetGoalState(goal);

		rrtStar.SearchPath();
		std::vector<Vertex> path = rrtStar.GetPath();

		assert(!path.empty());
		assert(stateSpace->ComputeDistance(start, path.front()) <= parameters.optimalSolutionTolerance);
		assert(stateSpace->ComputeDistance(goal, path.back()) <= parameters.optimalSolutionTolerance);
	}
}

int main()
{
	Planner::TestRRTStarPath();

	return 0;
}
