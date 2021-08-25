#include "core/base.h"
#include "a_star.h"
#include "2dplane.h"
#include "core/assert.h"
#include "core/log.h"
#include "core/timer.h"

double Heuristic(const Vertex& from, const Vertex& to)
{
	Vertex delta = from - to;
	return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
}

int main(int /*argc*/, char** /*argv*/)
{
	PP_INIT_LOGGER;
	PP_INFO("Test");

	Planner::AStar<Vertex> search(Planner::makeRef<Planner::TestAStarStateSpace>(), Heuristic);

	Planner::AStar<Vertex>::Parameters parameters;
	search.SetParameters(parameters);

	Vertex start = { 0.0, 0.0 };
	Vertex goal = { 5.0, 1.0 };
	search.SetInitState(start);
	search.SetGoalState(goal);

	Planner::Timer timer;
	search.SearchPath();
	PP_INFO(timer.ElapsedMillis());
	std::vector<Vertex> path = search.GetPath();
	PP_INFO(timer.ElapsedMillis());

	std::cout << "Path: " << std::endl;
	for (auto point : path) {
		std::cout << "x=" << point.x() << ", y=" << point.y() << std::endl;
	}

	return 0;
}
