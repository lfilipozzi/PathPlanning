#include "core/base.h"
#include "algo/rrt.h"
#include "geometry/2dplane.h"
#include "paths/path_r2.h"
#include "state_space/state_space_point2d.h"
#include "state_validator/state_validator_free.h"

namespace Planner {
	void TestRRTPath()
	{
		std::array<Point2d, 2> bounds = { Point2d(0, 0), Point2d(5, 5) };
		auto stateSpace = makeRef<StateSpacePoint2d>(bounds);
		auto stateValidator = makeRef<StateValidatorFree<Point2d, 2, double>>(stateSpace);
		auto pathConnection = makeRef<PathConnectionR2>();

		PlanarRRT rrt(stateSpace, stateValidator, pathConnection);
		RRTParameters parameters;
		rrt.SetParameters(parameters);

		Point2d start = { 0.0, 0.0 };
		Point2d goal = { 2.0, 2.0 };
		rrt.SetInitState(start);
		rrt.SetGoalState(goal);

		rrt.SearchPath();
		std::vector<Point2d> path = rrt.GetPath();
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
	Planner::TestRRTPath();

	return 0;
}
