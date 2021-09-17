#include "core/base.h"
#include "core/assert.h"
#include "core/log.h"
#include "core/timer.h"

#if 0

	#include "a_star.h"
	#include "geometry/2dplane.h"

double Heuristic(const Point2DInt& from, const Point2DInt& to)
{
	Point2DInt delta = from - to;
	return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
}

int main(int /*argc*/, char** /*argv*/)
{
	PP_INIT_LOGGER;

	Planner::AStar<Point2DInt> search(Planner::makeRef<Planner::TestAStarStateSpace>(), Heuristic);

	Planner::AStar<Point2DInt>::Parameters parameters;
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

#else

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

#endif
