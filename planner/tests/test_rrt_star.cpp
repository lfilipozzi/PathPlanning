#include "core/base.h"
#include "rrt_star.h"
#include "geometry/2dplane.h"
#include "core/log.h"
#include "state_space/rrt_star_state_space_2d.h"

namespace Planner {
	using Vertex = Point2D;

	void TestRRTStarPath()
	{
		auto stateSpace = Planner::makeRef<RRTStarStateSpace2D>();

		Planner::RRTStar<Vertex, 2> rrtStar(stateSpace);
		Planner::RRTStarParameters parameters;
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
	PP_INIT_LOGGER;
	Planner::TestRRTStarPath();

	return 0;
}
