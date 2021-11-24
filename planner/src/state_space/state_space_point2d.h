#pragma once

#include "state_space/state_space.h"
#include "geometry/2dplane.h"

namespace Planner {

	/// @brief State-space of 2D point
	class StateSpacePoint2d : public StateSpace<Point2d, 2, double> {
	public:
		StateSpacePoint2d(const std::array<Point2d, 2>& bounds) :
			StateSpace(bounds) { }
		StateSpacePoint2d(const Point2d& lb, const Point2d& ub) :
			StateSpace(std::array<Point2d, 2>({ lb, ub })) { }
		virtual ~StateSpacePoint2d() = default;

		/// @copydoc StateSpace::EnforceBounds
		virtual void EnforceBounds(Point2d& state) override final;

		/// @copydoc StateSpace::ValidateBounds
		virtual bool ValidateBounds(const Point2d& state) override final;

		/// @copydoc StateSpace::SampleUniform
		virtual Point2d SampleUniform() override final;

		/// @copydoc StateSpace::SampleGaussian
		virtual Point2d SampleGaussian(const Point2d& mean, const Point2d& stdDev) override final;
	};

}
