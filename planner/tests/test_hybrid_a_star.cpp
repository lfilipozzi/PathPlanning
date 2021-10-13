#include "core/base.h"
#include "algo/hybrid_a_star.h"
#include "geometry/2dplane.h"
#include "state_validator/state_validator_free.h"

namespace Planner {
	void TestHybridAStar()
	{
		Ref<HybridAStar::StateSpace> stateSpace = makeRef<HybridAStar::StateSpace>();
		Ref<StateValidator<Pose2D<>>> stateValidator = makeRef<StateValidatorFree<Pose2D<>>>();
		stateSpace->SetBounds({ { { -50, 50 }, { -50, 50 }, { -M_PI, M_PI } } });

		HybridAStar hybridAStar(stateSpace, stateValidator);

		Pose2D<> start = { 0.0, 0.0, 0.0 };
		Pose2D<> goal = { 10.0, 10.0, 0.78 };
		hybridAStar.SetInitState(start);
		hybridAStar.SetGoalState(goal);

		hybridAStar.SearchPath();
		std::vector<Pose2D<>> path = hybridAStar.GetPath();

		assert(!path.empty());
		assert(stateSpace->DiscretizePose(start) == stateSpace->DiscretizePose(path.front()));
		assert(stateSpace->DiscretizePose(goal) == stateSpace->DiscretizePose(path.back()));

		// Check we can change parameters
		hybridAStar.GetStateSpace()->numGeneratedMotion = 5;
		hybridAStar.GetStateSpace()->minTurningRadius = 1.0;
		hybridAStar.GetStateSpace()->forwardCostMultiplier = 1.0;
		hybridAStar.GetStateSpace()->reverseCostMultiplier = 1.0;
		hybridAStar.GetStateSpace()->directionSwitchingCost = 0.0;
		hybridAStar.GetStateSpace()->SetBounds({ { { -100, 100 }, { -100, 100 }, { -M_PI, M_PI } } });
	}
}

int main()
{
	PP_INIT_LOGGER;
	Planner::TestHybridAStar();

	return 0;
}
