#include "state_space/state_space_point2d.h"

namespace Planner {
	void StateSpacePoint2d::EnforceBounds(Point2d& state)
	{
		auto lb = this->bounds[0];
		auto ub = bounds[1];
		// clang-format off
		state.x() = std::clamp(state.x(), lb.x(), ub.x());
		state.y() = std::clamp(state.y(), lb.y(), ub.y());
		// clang-format on
	}

	/// @copydoc StateSpace::ValidateBounds
	bool StateSpacePoint2d::ValidateBounds(const Point2d& state)
	{
		auto lb = bounds[0];
		auto ub = bounds[1];
		// clang-format off
		if (state.x() < lb.x() || state.x() > ub.x()) return false;
		if (state.y() < lb.y() || state.y() > ub.y()) return false;
		// clang-format on
		return true;
	}

	/// @copydoc StateSpace::SampleUniform
	Point2d StateSpacePoint2d::SampleUniform()
	{
		Point2d state;
		auto lb = bounds[0];
		auto ub = bounds[1];
		// clang-format off
		state.x() = Random<double>::SampleUniform(lb.x(), ub.x());
		state.y() = Random<double>::SampleUniform(lb.y(), ub.y());
		// clang-format off
		return state;
	}

	/// @copydoc StateSpace::SampleGaussian
	 Point2d StateSpacePoint2d::SampleGaussian(const Point2d& mean, const Point2d& stdDev)
	{
		Point2d state;
		auto lb = bounds[0];
		auto ub = bounds[1];
		// clang-format off
		state.x() = Random<double>::SampleGaussian(mean.x(), stdDev.x());
		state.y() = Random<double>::SampleGaussian(mean.y(), stdDev.y());
		// clang-format off
		EnforceBounds(state);
		return state;
	}
}
