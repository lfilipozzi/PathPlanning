#include "state_space/state_space.h"

namespace Planner {
	void PlanarStateSpace::EnforceBounds(Pose2d& state)
	{
		auto lb = bounds[0];
		auto ub = bounds[1];
		// clang-format off
		state.x()   = std::max(lb.x(),   std::min(state.x(),   ub.x()));
		state.y()   = std::max(lb.y(),   std::min(state.y(),   ub.y()));
		state.theta = std::max(lb.theta, std::min(state.theta, ub.theta));
		// clang-format on
	}

	/// @copydoc StateSpace::ValidateBounds
	bool PlanarStateSpace::ValidateBounds(const Pose2d& state)
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

	/// @copydoc StateSpace::SampleUniform
	Pose2d PlanarStateSpace::SampleUniform()
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

	/// @copydoc StateSpace::SampleGaussian
	 Pose2d PlanarStateSpace::SampleGaussian(const Pose2d& mean, const Pose2d& stdDev)
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
