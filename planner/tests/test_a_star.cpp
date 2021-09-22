#include "core/base.h"
#include "a_star.h"
#include "geometry/2dplane.h"
#include "core/log.h"
#include "state_space/a_star_state_space_2d.h"

namespace Planner {
	double Heuristic(const Point2DInt& from, const Point2DInt& to)
	{
		Point2DInt delta = from - to;
		return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
	}

	void TestAStar()
	{
		auto stateSpace = Planner::makeRef<AStarStateSpace2D>();

		Planner::AStar<Point2DInt> aStar(stateSpace, Heuristic);
		Planner::AStarParameters parameters;
		aStar.SetParameters(parameters);

		Point2DInt start = { 0.0, 0.0 };
		Point2DInt goal = { 10.0, 5.0 };
		aStar.SetInitState(start);
		aStar.SetGoalState(goal);

		aStar.SearchPath();
		std::vector<Point2DInt> path = aStar.GetPath();

		assert(!path.empty());
		assert(start == path.front());
		assert(goal == path.back());
	}
}

int main()
{
	PP_INIT_LOGGER;
	Planner::TestAStar();

	return 0;
}
