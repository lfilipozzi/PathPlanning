#include "core/base.h"
#include "core/timer.h"

#define MODE 4

#if MODE == 0

	#include "algo/a_star.h"
	#include "geometry/2dplane.h"
	#include "planner/tests/state_space/a_star_state_space_2d.h"
	#include <iostream>

class Heuristic2D : public Planner::AStarHeuristic<Planner::Point2i> {
	virtual double GetHeuristicValue(const Planner::Point2i& from, const Planner::Point2i& to) override
	{
		auto delta = from - to;
		return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
	}
};

int main(int /*argc*/, char** /*argv*/)
{
	PP_INIT_LOGGER;

	Planner::AStar<Planner::Point2i> search(
		Planner::makeRef<Planner::AStarStateSpace2D>(),
		Planner::makeRef<Heuristic2D>());

	Planner::Point2i start = { 0.0, 0.0 };
	Planner::Point2i goal = { 5.0, 1.0 };
	search.SetInitState(start);
	search.SetGoalState(goal);

	Planner::Timer timer;
	search.SearchPath();
	float searchTime = timer.ElapsedMillis();
	std::vector<Planner::Point2i> path = search.GetPath();
	PP_INFO("Search: {} ms", searchTime);

	std::cout << "Path: " << std::endl;
	for (auto point : path) {
		std::cout << "x=" << point.x() << ", y=" << point.y() << std::endl;
	}

	return 0;
}

#elif MODE == 1

	#include "algo/hybrid_a_star.h"
	#include "geometry/2dplane.h"
	#include "state_validator/state_validator_occupancy_map.h"
	#include "state_validator/binary_occupancy_map.h"

	#define _USE_MATH_DEFINES
	#include <cmath>
	#include <iostream>

using namespace Planner;

int main()
{
	PP_INIT_LOGGER;

	std::array<Pose2d, 2> bounds = { Pose2d(-50, -50, -M_PI), Pose2d(50, 50, M_PI) };
	Ref<HybridAStar::StateSpace> stateSpace = makeRef<HybridAStar::StateSpace>(bounds);
	Ref<BinaryOccupancyMap> map = makeRef<BinaryOccupancyMap>(0.1);
	Ref<StateValidatorOccupancyMap> stateValidator = makeRef<StateValidatorOccupancyMap>(stateSpace);
	stateValidator->SetOccupancyMap(map);

	HybridAStar hybridAStar(stateSpace, stateValidator);

	Timer timer;
	hybridAStar.Initialize();
	float initTime = timer.ElapsedMillis();

	Pose2d start = { 0.0, 0.0, 0.0 };
	Pose2d goal = { 10.0, 10.0, 0.78 };
	hybridAStar.SetInitState(start);
	hybridAStar.SetGoalState(goal);

	timer.Reset();
	hybridAStar.SearchPath();
	float searchTime = timer.ElapsedMillis();

	std::vector<Pose2d> path = hybridAStar.GetPath();
	std::cout << "Path: " << std::endl;
	for (auto point : path) {
		std::cout << "x=" << point.x() << ", y=" << point.y() << ", theta=" << point.WrapTheta() * 180 * M_1_PI << std::endl;
	}

	PP_INFO("Init: {} ms", initTime);
	PP_INFO("Search: {} ms", searchTime);

	return 0;
}

#elif MODE == 2

	#include "geometry/reeds_shepp.h"

int main(int argc, char** argv)
{
	PP_INIT_LOGGER;

	if (argc != 7) {
		PP_INFO("Invalid number of arguments: received {}, need 6", argc);
		return -1;
	}

	Planner::Pose2d start { atof(argv[1]), atof(argv[2]), atof(argv[3]) };
	Planner::Pose2d goal { atof(argv[4]), atof(argv[5]), atof(argv[6]) };

	Planner::ReedsShepp::PathWords word = Planner::ReedsShepp::PathWords::NoPath;
	double unit = 1.0;
	auto path = Planner::ReedsShepp::Solver::GetShortestPath(start, goal, unit, &word);

	PP_INFO("Start: {}, {}, {}", start.x(), start.y(), start.theta);
	PP_INFO("Goal:  {}, {}, {}", goal.x(), goal.y(), goal.theta);
	PP_INFO("Word {}: Length {}", word, path.GetLength(unit));
}

#elif MODE == 3

	#include "state_space/state_space_se2.h"

int main()
{
	PP_INIT_LOGGER;

	Planner::StateSpaceSE2 stateSpace;

	auto sample = stateSpace.SampleUniform();
	PP_INFO("x: {}; y: {}; theta: {}", sample.x(), sample.y(), sample.theta * 180.0 * M_1_PI);

	Planner::Pose2d mean;
	Planner::Pose2d stdDev(10.0, 10.0, 10.0 * M_PI / 180.0);
	sample = stateSpace.SampleGaussian(mean, stdDev);
	PP_INFO("x: {}; y: {}; theta: {}", sample.x(), sample.y(), sample.theta * 180.0 * M_1_PI);
}

#elif MODE == 4

	#include "state_validator/binary_occupancy_map.h"
	#include "state_validator/obstacle.h"
	#include "state_validator/gvd.h"
	#include "algo/hybrid_a_star_heuristics.h"

using namespace Planner;

int main()
{
	PP_INIT_LOGGER;

	float width = 5;
	float height = 5;
	float resolution = width / 300;
	auto map = makeRef<BinaryOccupancyMap>(resolution);
	map->InitializeSize(width, height);
	map->SetPosition({ 0.0, 0.0 });
	GVD gvd(map);
	ObstaclesHeuristic heuristic(map, 1.0, 1.0);
	Pose2d goal = { -0.5 * width, 0.5 * height, 0.0 };

	Ref<Obstacle> obstacle1 = makeRef<Obstacle>();
	{
		auto circle = makeRef<CircleShape>(1.0, 10);
		obstacle1->SetShape(circle);
		obstacle1->SetPose({ 1.0, -1.0, 0.0 });
	}

	Ref<Obstacle> obstacle2 = makeRef<Obstacle>();
	{
		auto rectangle = makeRef<RectangleShape>(2.0, 1.0);
		obstacle2->SetShape(rectangle);
		obstacle2->SetPose({ -1.0, 1.0, M_PI_4 });
	}

	Ref<Obstacle> obstacle3 = makeRef<Obstacle>();
	{
		auto rectangle = makeRef<RectangleShape>(0.8, 0.8);
		obstacle3->SetShape(rectangle);
		obstacle3->SetPose({ 1.0, 1.0, M_PI_4 });
	}

	map->AddObstacle(obstacle1);
	map->AddObstacle(obstacle2);
	map->AddObstacle(obstacle3);
	map->Update();

	gvd.Update();
	gvd.Visualize("results1.ppm");

	heuristic.Update(goal);
	heuristic.Visualize("heuristic1.ppm");

	map->RemoveObstacle(obstacle2);
	map->Update();

	gvd.Update();
	gvd.Visualize("results2.ppm");

	heuristic.Update(goal);
	heuristic.Visualize("heuristic2.ppm");
}

#endif
