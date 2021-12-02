#include "core/base.h"
#include "algo/rrt_star.h"
#include "geometry/2dplane.h"
#include "paths/path_r2.h"
#include "state_space/state_space_r2.h"
#include "state_validator/state_validator_free.h"

namespace Planner {

	void TestRRTStarPath()
	{
		std::array<Point2d, 2> bounds = { Point2d(0, 0), Point2d(5, 5) };
		auto stateSpace = makeRef<StateSpaceR2>(bounds);
		auto stateValidator = makeRef<StateValidatorFree<Point2d, 2, double>>(stateSpace);
		auto pathConnection = makeRef<PathConnectionR2>();

		RRTStarR2 rrtStar(stateSpace, stateValidator, pathConnection);
		RRTStarParameters parameters;
		rrtStar.SetParameters(parameters);

		Point2d start = { 0.0, 0.0 };
		Point2d goal = { 2.0, 2.0 };
		rrtStar.SetInitState(start);
		rrtStar.SetGoalState(goal);

		rrtStar.SearchPath();
		std::vector<Point2d> path = rrtStar.GetPath();
		std::cout << "Path: " << std::endl;
		for (auto point : path) {
			std::cout << "x=" << point.x() << ", y=" << point.y() << std::endl;
		}

		assert(!path.empty());
		double spatialTolerance = 1;
		auto deltaStart = path.front() - start;
		assert(deltaStart.norm() < spatialTolerance);
		auto deltaGoal = path.back() - goal;
		assert(deltaGoal.norm() < spatialTolerance);
	}
}

int main()
{
	PP_INIT;
	Planner::TestRRTStarPath();

	return 0;
}
