#include "core/base.h"
#include "core/assert.h"
#include "core/log.h"
#include "core/timer.h"

#define MODE 1

#if MODE == 0

	#include "algo/a_star.h"
	#include "geometry/2dplane.h"
	#include "planner/tests/state_space/a_star_state_space_2d.h"

class Heuristic2D : public Planner::AStarHeuristic<Point2DInt> {
	virtual double GetHeuristicValue(const Point2DInt& from, const Point2DInt& to) override
	{
		Point2DInt delta = from - to;
		return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
	}
};

int main(int /*argc*/, char** /*argv*/)
{
	PP_INIT_LOGGER;

	Planner::AStar<Point2DInt> search(
		Planner::makeRef<Planner::AStarStateSpace2D>(), 
		Planner::makeRef<Heuristic2D>());

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

	#include "algo/hybrid_a_star.h"
	#include "geometry/pose.h"
	#include "state_validator/state_validator_free.h"

	#define _USE_MATH_DEFINES
	#include <cmath>

int main()
{
	PP_INIT_LOGGER;

	Planner::Ref<Planner::HybridAStar::StateSpace> stateSpace = Planner::makeRef<Planner::HybridAStar::StateSpace>();
	Planner::Ref<Planner::StateValidator<Planner::Pose2D<>>> stateValidator = Planner::makeRef<Planner::StateValidatorFree<Planner::Pose2D<>>>();
	Planner::HybridAStar hybridAStar(stateSpace, stateValidator);

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
	#include "geometry/reeds_shepp.h"

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
	double unit = 1.0;
	auto path = Planner::ReedsShepp::Solver::GetShortestPath(start, goal, unit, &word);

	PP_INFO("Start: {}, {}, {}", start.x, start.y, start.theta);
	PP_INFO("Goal:  {}, {}, {}", goal.x, goal.y, goal.theta);
	PP_INFO("Word {}: Length {}", word, path.GetLength(unit));
}

#elif MODE == 3

	#include "state_space/state_space_se2.h"

int main()
{
	PP_INIT_LOGGER;

	Planner::StateSpaceSE2 stateSpace;

	auto sample = stateSpace.SampleUniform();
	PP_INFO("x: {}; y: {}; theta: {}", sample.x, sample.y, sample.theta * 180.0 * M_1_PI);

	Planner::Pose2D<> mean;
	Planner::Pose2D<> stdDev(10.0, 10.0, 10.0 * M_PI / 180.0);
	sample = stateSpace.SampleGaussian(mean, stdDev);
	PP_INFO("x: {}; y: {}; theta: {}", sample.x, sample.y, sample.theta * 180.0 * M_1_PI);
}

#endif
