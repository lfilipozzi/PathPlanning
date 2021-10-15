#include "core/base.h"
#include "algo/hybrid_a_star.h"
#include "geometry/2dplane.h"
#include "state_validator/state_validator_free.h"

namespace Planner {
	void TestHybridAStar()
	{
		Ref<HybridAStar::StateSpace> stateSpace = makeRef<HybridAStar::StateSpace>();
		Ref<StateValidator<Pose2d>> stateValidator = makeRef<StateValidatorFree<Pose2d>>();
		stateSpace->SetBounds({ Pose2d(-50, -50, -M_PI ), Pose2d(50, 50, M_PI) });

		HybridAStar hybridAStar(stateSpace, stateValidator);

		Pose2d start = { 0.0, 0.0, 0.0 };
		Pose2d goal = { 10.0, 10.0, 0.78 };
		hybridAStar.SetInitState(start);
		hybridAStar.SetGoalState(goal);

		hybridAStar.SearchPath();
		std::vector<Pose2d> path = hybridAStar.GetPath();

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
