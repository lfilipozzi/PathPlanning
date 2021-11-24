#pragma once

#include "paths/path.h"
#include "geometry/2dplane.h"

namespace Planner {

	class KinematicBicycleModel;

	/// @brief Represent a planar path of constant steering angle using a
	/// kinematic bicycle model.
	class PathConstantSteer : public PathNonHolonomicSE2Base {
	public:
		PathConstantSteer(const Ref<KinematicBicycleModel>& model, const Pose2d& init, double steering = 0.0, double length = 0.0, Direction direction = Direction::NoMotion);

		/// @copydoc Planner::Path::Interpolate
		virtual Pose2d Interpolate(double ratio) const override;
		using PathNonHolonomicSE2Base::Interpolate;
		/// @copydoc Planner::Path::Truncate
		virtual void Truncate(double ratio) override;
		/// @copydoc Planner::PathNonHolonomic::GetDirection
		virtual Direction GetDirection(double ratio) const override;

		/// @brief Get Steering angle
		double GetSteeringAngle() const { return m_steering; }

	private:
		Ref<KinematicBicycleModel> m_model;
		double m_steering;
		Direction m_direction;
	};
}
