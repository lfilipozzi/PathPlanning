#include "core/base.h"
#include "a_star.h"
#include "2dplane.h"
#include "core/log.h"

namespace Planner {
	double Heuristic(const Vertex& from, const Vertex& to)
	{
		Vertex delta = from - to;
		return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
	}

	void TestRRTPath()
	{
		Planner::Scope<TestAStarStateSpace> stateSpaceScope = Planner::makeScope<TestAStarStateSpace>();
		TestStateSpace* stateSpace = stateSpaceScope.get();

		Planner::AStar<Vertex> aStar(std::move(stateSpaceScope), Heuristic);
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
	PP_INIT_LOGGER;
	Planner::TestRRTPath();

	return 0;
}
