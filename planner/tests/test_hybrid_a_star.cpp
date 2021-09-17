#include "core/base.h"
#include "hybrid_a_star.h"
#include "core/log.h"
#include "geometry/pose.h"

namespace Planner {
	void TestHybridAStar()
	{
		Ref<HybridAStar::StateSpace> stateSpace = makeRef<HybridAStar::StateSpace>();

		HybridAStar hybridAStar(stateSpace);

		Pose2D<> start = { 0.0, 0.0, 0.0 };
		Pose2D<> goal = { 10.0, 10.0, 0.78 };
		hybridAStar.SetInitState(start);
		hybridAStar.SetGoalState(goal);

		hybridAStar.SearchPath();
		std::vector<Pose2D<>> path = hybridAStar.GetPath();

		assert(!path.empty());
		assert(stateSpace->DiscretizePose(start) == stateSpace->DiscretizePose(path.front()));
		assert(stateSpace->DiscretizePose(goal) == stateSpace->DiscretizePose(path.back()));
	}
}

int main()
{
	PP_INIT_LOGGER;
	Planner::TestHybridAStar();

	return 0;
}
