#include "core/base.h"
#include "core/assert.h"
#include "core/log.h"
#include "core/timer.h"

#define MODE 1

#if MODE == 0

	#include "a_star.h"
	#include "geometry/2dplane.h"
	#include "planner/tests/state_space/a_star_state_space_2d.h"

double Heuristic(const Point2DInt& from, const Point2DInt& to)
{
	Point2DInt delta = from - to;
	return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
}

int main(int /*argc*/, char** /*argv*/)
{
	PP_INIT_LOGGER;

	Planner::AStar<Point2DInt> search(Planner::makeRef<Planner::AStarStateSpace2D>(), Heuristic);

	Planner::AStarParameters parameters;
	search.SetParameters(parameters);

	Point2DInt start = { 0.0, 0.0 };
	Point2DInt goal = { 5.0, 1.0 };
	search.SetInitState(start);
	search.SetGoalState(goal);

	Planner::Timer timer;
	search.SearchPath();
	PP_INFO(timer.ElapsedMillis());
	std::vector<Point2DInt> path = search.GetPath();
	PP_INFO(timer.ElapsedMillis());

	std::cout << "Path: " << std::endl;
	for (auto point : path) {
		std::cout << "x=" << point.x() << ", y=" << point.y() << std::endl;
	}

	return 0;
}

#elif MODE == 1

	#include "hybrid_a_star.h"
	#include "geometry/pose.h"

	#define _USE_MATH_DEFINES
	#include <cmath>

int main()
{
	PP_INIT_LOGGER;

	Planner::Ref<Planner::HybridAStar::StateSpace> stateSpace = Planner::makeRef<Planner::HybridAStar::StateSpace>();

	Planner::HybridAStar hybridAStar(stateSpace);

	Planner::Pose2D<> start = { 0.0, 0.0, 0.0 };
	Planner::Pose2D<> goal = { 10.0, 10.0, 0.78 };
	hybridAStar.SetInitState(start);
	hybridAStar.SetGoalState(goal);

	Planner::Timer timer;
	hybridAStar.SearchPath();
	PP_INFO(timer.ElapsedMillis());
	std::vector<Planner::Pose2D<>> path = hybridAStar.GetPath();
	PP_INFO(timer.ElapsedMillis());
	std::cout << "Path: " << std::endl;
	for (auto point : path) {
		std::cout << "x=" << point.x << ", y=" << point.y << ", theta=" << point.WrapTheta() * 180 * M_1_PI << std::endl;
	}

	return 0;
}

#elif MODE == 2

	#include "core/log.h"
	#include "reeds_shepp.h"

int main(int argc, char** argv)
{
	PP_INIT_LOGGER;

	if (argc != 7) {
		PP_INFO("Invalid number of arguments: received {}, need 6", argc);
		return -1;
	}

	Planner::Pose2D<> start { atof(argv[1]), atof(argv[2]), atof(argv[3]) };
	Planner::Pose2D<> goal { atof(argv[4]), atof(argv[5]), atof(argv[6]) };

	Planner::ReedsShepp::PathWords word = Planner::ReedsShepp::PathWords::NoPath;
	float unit = 1.0f;
	auto path = Planner::ReedsShepp::Solver::GetShortestPath(start, goal, unit, word);

	PP_INFO("Start: {}, {}, {}", start.x, start.y, start.theta);
	PP_INFO("Goal:  {}, {}, {}", goal.x, goal.y, goal.theta);
	PP_INFO("Word {}: Length {}", word, path.GetLength());
}

#endif
