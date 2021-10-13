#include "core/base.h"
#include "algo/a_star.h"
#include "geometry/2dplane.h"
#include "state_space/a_star_state_space_2d.h"

namespace Planner {
	class Test2DHeuristic : public AStarHeuristic<Point2DInt> {
	public:
		virtual double GetHeuristicValue(const Point2DInt& from, const Point2DInt& to) override
		{
			Point2DInt delta = from - to;
			return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
		}
	};

	void TestAStar()
	{
		auto stateSpace = Planner::makeRef<AStarStateSpace2D>();
		auto heuristic = Planner::makeRef<Test2DHeuristic>();

		Planner::AStar<Point2DInt> aStar(stateSpace, heuristic);

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
