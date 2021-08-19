#include "base.h"
#include "rrt.h"
#include "2dplane.h"

int main(int /*argc*/, char** /*argv*/)
{
	Planner::Scope<TestStateSpace> stateSpace = Planner::makeScope<TestStateSpace>();

	Planner::RRT<Vertex, 2> rrt(std::move(stateSpace));

	Planner::RRT<Vertex, 2>::Parameters parameters;
	rrt.SetParameters(parameters);

	Vertex start = { 0.0, 0.0 };
	Vertex goal = { 2.0, 2.0 };
	rrt.SetInitState(start);
	rrt.SetGoalState(goal);

	rrt.SearchPath();
	std::vector<Vertex> path = rrt.GetPath();

	std::cout << "Path: " << std::endl;
	for (auto point : path) {
		std::cout << "x=" << point.x() << ", y=" << point.y() << std::endl;
	}

	return 0;
}
