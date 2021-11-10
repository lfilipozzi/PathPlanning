#include "core/base.h"
#include "algo/hybrid_a_star.h"
#include "geometry/2dplane.h"
#include "state_space/state_space_reeds_shepp.h"
#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/obstacle_list_occupancy_map.h"

namespace Planner {
	void TestHybridAStar()
	{
		std::array<Pose2d, 2> bounds = { Pose2d(-50, -50, -M_PI), Pose2d(50, 50, M_PI) };
		Ref<StateSpaceReedsShepp> stateSpace = makeRef<StateSpaceReedsShepp>(bounds);
		Ref<ObstacleListOccupancyMap> map = makeRef<ObstacleListOccupancyMap>(0.1);
		Ref<StateValidatorOccupancyMap> stateValidator = makeRef<StateValidatorOccupancyMap>(stateSpace, map);

		HybridAStar hybridAStar;
		hybridAStar.Initialize(stateValidator);

		Pose2d start = { 0.0, 0.0, 0.0 };
		Pose2d goal = { 10.0, 10.0, 0.78 };
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
	PP_INIT_LOGGER;
	Planner::TestHybridAStar();

	return 0;
}
