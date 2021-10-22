#pragma once

#include "paths/path.h"
#include "geometry/2dplane.h"

namespace Planner {

	class KinematicBicycleModel;

	/// @brief Represent a planar path of constant steering angle using a
	/// kinematic bicycle model.
	class PathConstantSteer : public Path<Pose2d> {
	public:
		PathConstantSteer(KinematicBicycleModel* model, const Pose2d& init, double steering = 0.0, double length = 0.0, Direction direction = Direction::Forward);

		/// @copydoc Planer::Path::Interpolate
		virtual Pose2d Interpolate(double ratio) const override;
		/// @copydoc Planner::Path::Truncate
		virtual void Truncate(double ratio) override;
		/// @copydoc Planner::Path::ComputeCost
		virtual double ComputeCost(double directionSwitchingCost, double reverseCostMultiplier, double forwardCostMultiplier) const override;

	private:
		KinematicBicycleModel* m_model;
		double m_steering;
		Direction m_direction;
	};
}
