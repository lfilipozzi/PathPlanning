#include "base.h"
#include "a_star.h"
#include "2dplane.h"

namespace Planner {
	void TestRRTPath()
	{
		Planner::Scope<TestStateSpace> stateSpaceScope = Planner::makeScope<TestStateSpace>();
		TestStateSpace* stateSpace = stateSpaceScope.get();

		Planner::AStar<Vertex> aStar(std::move(stateSpaceScope));
		Planner::AStar<Vertex>::Parameters parameters;
		aStar.SetParameters(parameters);

		Vertex start = { 0.0, 0.0 };
		Vertex goal = { 2.0, 2.0 };
		aStar.SetInitState(start);
		aStar.SetGoalState(goal);

		aStar.SearchPath();
		std::vector<Vertex> path = aStar.GetPath();

		assert(!path.empty());
		assert(stateSpace->ComputeDistance(start, path.front()) <= parameters.optimalSolutionTolerance);
		assert(stateSpace->ComputeDistance(goal, path.back()) <= parameters.optimalSolutionTolerance);
	}
}

int main()
{
	Planner::TestRRTPath();

	return 0;
}
