#include "models/kinematic_bicycle_model.h"

namespace Planner {
	Pose2d KinematicBicycleModel::ConstantSteer(const Pose2d& from, double steering, double dist, Direction direction)
	{
		if (direction == Direction::Backward)
			dist = -dist;

		// Get current pose
		Pose2d to = from;

		// Update pose
		// angle between center of curvature, CoG, and rear wheel
		double beta = atan(m_rearToCG * tan(steering) / m_wheelbase);
		// Change in yaw angle per distance traveled
		double DthetaDdist = cos(beta) * tan(steering) / m_wheelbase;
		if (DthetaDdist > 1e-9) {
			to.theta += dist * DthetaDdist;
			to.x() += 1 / DthetaDdist * ( sin(beta + to.theta) - sin(beta + from.theta));
			to.y() += 1 / DthetaDdist * (-cos(beta + to.theta) + cos(beta + from.theta));
		} else {
			to.x() += dist * cos(from.theta);
			to.y() += dist * sin(from.theta);
		}

		return to;
	}

	double KinematicBicycleModel::GetSteeringAngleFromTurningRadius(double radius)
	{
		return atan(m_wheelbase / radius);
	}
}
