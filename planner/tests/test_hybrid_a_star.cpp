#include "core/base.h"
#include "algo/hybrid_a_star.h"
#include "geometry/2dplane.h"
#include "state_space/state_space_se2.h"
#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/obstacle_list_occupancy_map.h"

namespace Planner {
	void TestHybridAStar()
	{
		std::array<Pose2d, 2> bounds = { Pose2d(-10, -10, -M_PI), Pose2d(10, 10, M_PI) };
		Ref<StateSpaceSE2> stateSpace = makeRef<StateSpaceSE2>(bounds);
		Ref<ObstacleListOccupancyMap> map = makeRef<ObstacleListOccupancyMap>(0.1);
		Ref<StateValidatorOccupancyMap> stateValidator = makeRef<StateValidatorOccupancyMap>(stateSpace, map);

		HybridAStar::HybridAStar hybridAStar;
		hybridAStar.Initialize(stateValidator);

		Pose2d start = { 0.0, 0.0, 0.0 };
		Pose2d goal = { 8.0, 8.0, 0.78 };
		hybridAStar.SetInitState(start);
		hybridAStar.SetGoalState(goal);

		hybridAStar.SearchPath();
		std::vector<Pose2d> path = hybridAStar.GetPath();

		assert(!path.empty());
		double spatialTolerance = 1e-1;
		double angularTolerance = 5 * M_PI / 180;
		auto deltaStart = path.front() - start;
		assert(deltaStart.position.norm() < spatialTolerance);
		assert(abs(deltaStart.theta) < angularTolerance);
		auto deltaGoal = path.back() - goal;
		assert(deltaGoal.position.norm() < spatialTolerance);
		assert(abs(deltaGoal.theta) < angularTolerance);
	}
}

int main()
{
	PP_INIT;
	Planner::TestHybridAStar();

	return 0;
}
