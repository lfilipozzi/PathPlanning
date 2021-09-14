#include "core/base.h"
#include "a_star.h"
#include "geometry/2dplane.h"
#include "core/log.h"

namespace Planner {
	double Heuristic(const Vertex& from, const Vertex& to)
	{
		Vertex delta = from - to;
		return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
	}

	void TestAStar()
	{
		Planner::Ref<TestAStarStateSpace> stateSpace = Planner::makeRef<TestAStarStateSpace>();

		Planner::AStar<Vertex> aStar(stateSpace, Heuristic);
		Planner::AStar<Vertex>::Parameters parameters;
		aStar.SetParameters(parameters);

		Vertex start = { 0.0, 0.0 };
		Vertex goal = { 2.0, 2.0 };
		aStar.SetInitState(start);
		aStar.SetGoalState(goal);

		aStar.SearchPath();
		std::vector<Vertex> path = aStar.GetPath();

		assert(!path.empty());
		assert(stateSpace->DiscretizeState(start) == stateSpace->DiscretizeState(path.front()));
		assert(stateSpace->DiscretizeState(goal) == stateSpace->DiscretizeState(path.back()));
	}
}

int main()
{
	PP_INIT_LOGGER;
	Planner::TestAStar();

	return 0;
}
