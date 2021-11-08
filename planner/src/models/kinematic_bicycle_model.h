#pragma once
#include "geometry/2dplane.h"
#include "paths/path.h"

namespace Planner {
	/// @brief Model a kinematic bicycle model.
	/// @details The bicycle model is defined at its center (which is at a
	/// distance @rearToCG from the rear axle). All position are given at this
	/// point.
	class KinematicBicycleModel {
	public:
		KinematicBicycleModel(double wheelbase = 2.6, double rearToCenter = 0.0) :
			m_wheelbase(wheelbase), m_rearToCenter(rearToCenter) { }

		/// @brief Propagate the model from the pose @from with a constant
		/// steering angle @delta over a distance @dist.
		/// @param from The initial pose.
		/// @param steering The steering angle applied.
		/// @param dist The distance traveled by the rear wheel.
		/// @param direction The direction of motion.
		Pose2d ConstantSteer(const Pose2d& from, double steering, double dist, Direction direction = Direction::Forward);

		/// @brief Compute the steering angle corresponding to the turning
		/// radius.
		double GetSteeringAngleFromTurningRadius(double radius);

	private:
		double m_wheelbase;
		double m_rearToCenter;
	};
}
