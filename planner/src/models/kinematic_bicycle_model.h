#pragma once
#include "geometry/2dplane.h"
#include "paths/path.h"

namespace Planner {
	class KinematicBicycleModel {
	public:
		KinematicBicycleModel() = default;

		Pose2d ConstantSteer(const Pose2d& from, double steering, double dist, Direction direction = Direction::Forward);

		double GetSteeringAngleFromTurningRadius(double radius);

	private:
		double m_wheelbase = 2.6;
		double m_rearToCG = 1.3;
	};
}
