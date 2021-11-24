#pragma once

#include "state_space/state_space.h"
#include "geometry/2dplane.h"

namespace Planner {

	/// @brief State-space of 2D point
	class StateSpaceR2 : public StateSpace<Point2d, 2, double> {
	public:
		StateSpaceR2(const std::array<Point2d, 2>& bounds) :
			StateSpace(bounds) { }
		StateSpaceR2(const Point2d& lb, const Point2d& ub) :
			StateSpace(std::array<Point2d, 2>({ lb, ub })) { }
		virtual ~StateSpaceR2() = default;

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
