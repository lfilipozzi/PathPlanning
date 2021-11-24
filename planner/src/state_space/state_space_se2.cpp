#include "state_space/state_space_se2.h"

namespace Planner {
	void StateSpaceSE2::EnforceBounds(Pose2d& state)
	{
		auto lb = bounds[0];
		auto ub = bounds[1];
		// clang-format off
		state.x()   = std::clamp(state.x(),   lb.x(),   ub.x());
		state.y()   = std::clamp(state.y(),   lb.y(),   ub.y());
		state.theta = std::clamp(state.theta, lb.theta, ub.theta);
		// clang-format on
	}

	bool StateSpaceSE2::ValidateBounds(const Pose2d& state)
	{
		auto lb = bounds[0];
		auto ub = bounds[1];
		// clang-format off
		if (state.x()   < lb.x()   || state.x()   > ub.x())   return false;
		if (state.y()   < lb.y()   || state.y()   > ub.y())   return false;
		if (state.theta < lb.theta || state.theta > ub.theta) return false;
		// clang-format on
		return true;
	}

	Pose2d StateSpaceSE2::SampleUniform()
	{
		Pose2d state;
		auto lb = bounds[0];
		auto ub = bounds[1];
		// clang-format off
		state.x()   = Random<double>::SampleUniform(lb.x(),   ub.x());
		state.y()   = Random<double>::SampleUniform(lb.y(),   ub.y());
		state.theta = Random<double>::SampleUniform(lb.theta, ub.theta);
		// clang-format off
		return state;
	}

	 Pose2d StateSpaceSE2::SampleGaussian(const Pose2d& mean, const Pose2d& stdDev)
	{
		Pose2d state;
		auto lb = bounds[0];
		auto ub = bounds[1];
		// clang-format off
		state.x()   = Random<double>::SampleGaussian(mean.x(),   stdDev.x());
		state.y()   = Random<double>::SampleGaussian(mean.y(),   stdDev.y());
		state.theta = Random<double>::SampleGaussian(mean.theta, stdDev.theta);
		// clang-format off
		EnforceBounds(state);
		return state;
	}
}
