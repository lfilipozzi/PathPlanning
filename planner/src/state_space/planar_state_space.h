#pragma once

#include "state_space/state_space.h"
#include "geometry/2dplane.h"

namespace Planner {

	/// @brief Planar state space
	class PlanarStateSpace : public StateSpace<Pose2d, 3, double> {
	public:
		PlanarStateSpace(const std::array<Pose2d, 2>& bounds) :
			StateSpace(bounds) { }
		PlanarStateSpace(const Pose2d& lb, const Pose2d& ub) :
			PlanarStateSpace(std::array<Pose2d, 2>({ lb, ub })) { }
		virtual ~PlanarStateSpace() = default;

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
