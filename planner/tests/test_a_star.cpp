#include "core/base.h"
#include "algo/a_star.h"
#include "geometry/2dplane.h"
#include "state_space/a_star_state_space_2d.h"

namespace Planner {
	class Test2DHeuristic : public AStarConcreteHeuristic<Point2i> {
	public:
		virtual double GetHeuristicValue(const Point2i& state) override
		{
			Point2i delta = state - m_goal;
			return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
		}
	};

	void TestAStar()
	{
		auto statePropagator = Planner::makeRef<AStarStatePropagator2D>();
		auto heuristic = Planner::makeRef<Test2DHeuristic>();

		Planner::AStar<Point2i> aStar;
		aStar.Initialize(statePropagator, heuristic);

		Point2i start = { 0.0, 0.0 };
		Point2i goal = { 10.0, 5.0 };
		aStar.SetInitState(start);
		aStar.SetGoalState(goal);

		aStar.SearchPath();
		std::vector<Point2i> path = aStar.GetPath();

		assert(!path.empty());
		assert(start == path.front());
		assert(goal == path.back());
	}
}

int main()
{
	PP_INIT;
	Planner::TestAStar();
	PP_END;

	return 0;
}
