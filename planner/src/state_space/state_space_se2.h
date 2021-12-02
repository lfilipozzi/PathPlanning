#pragma once

#include "state_space/state_space.h"
#include "geometry/2dplane.h"

namespace Planner {

	/// @brief Planar state space
	class StateSpaceSE2 : public StateSpace<Pose2d, 3, double> {
	public:
		StateSpaceSE2(const std::array<Pose2d, 2>& bounds) :
			StateSpace(bounds) { }
		StateSpaceSE2(const Pose2d& lb, const Pose2d& ub) :
			StateSpaceSE2(std::array<Pose2d, 2>({ lb, ub })) { }
		virtual ~StateSpaceSE2() = default;

		/// @copydoc StateSpace::EnforceBounds
		virtual void EnforceBounds(Pose2d& state) override final;

		/// @copydoc StateSpace::ValidateBounds
		virtual bool ValidateBounds(const Pose2d& state) override final;

		/// @copydoc StateSpace::SampleUniform
		virtual Pose2d SampleUniform() override final;

		/// @copydoc StateSpace::SampleGaussian
		virtual Pose2d SampleGaussian(const Pose2d& mean, const Pose2d& stdDev) override final;
	};

}
