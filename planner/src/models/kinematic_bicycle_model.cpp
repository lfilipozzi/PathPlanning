#include "models/kinematic_bicycle_model.h"

namespace Planner {
	Pose2D<> KinematicBicycleModel::ConstantSteer(const Pose2D<>& from, double steering, double dist, Direction direction)
	{
		if (direction == Direction::Backward)
			dist = -dist;

		// Get current pose
		Pose2D<> interp = from;

		// Update pose
		double& theta = interp.theta;
		double beta = atan(m_rearToCG * tan(steering) / m_wheelbase);
		interp.x += dist * cos(theta + beta);
		interp.y += dist * sin(theta + beta);
		interp.theta += dist * cos(beta) * tan(steering) / m_wheelbase;

		return interp;
	}

	double KinematicBicycleModel::GetSteeringAngleFromTurningRadius(double radius)
	{
		return atan(m_wheelbase / radius);
	}
}
