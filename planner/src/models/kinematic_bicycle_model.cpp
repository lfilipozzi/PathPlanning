#include "models/kinematic_bicycle_model.h"
#include "core/base.h"

namespace Planner {
	Pose2d KinematicBicycleModel::ConstantSteer(const Pose2d& from, double steering, double dist, Direction direction)
	{
		if (direction == Direction::Backward)
			dist = -dist;

		// Get current pose
		Pose2d to = from;

		// Compute angle between center of curvature, center, and rear wheel
		double tanSteering = tan(steering);
		double beta = atan(m_rearToCenter * tanSteering / m_wheelbase);
		double cosBeta = cos(beta);
		// Change in yaw angle per distance traveled
		double DthetaDdist = cosBeta * tanSteering / m_wheelbase;
		// Increase the distance if the point defining the position is not the rear axle
		dist = dist / cosBeta;
		// Update pose
		if (abs(DthetaDdist) > 1e-9) {
			to.theta += dist * DthetaDdist;
			to.x() += 1 / DthetaDdist * (sin(beta + to.theta) - sin(beta + from.theta));
			to.y() += 1 / DthetaDdist * (-cos(beta + to.theta) + cos(beta + from.theta));
		} else {
			to.x() += dist * cos(from.theta);
			to.y() += dist * sin(from.theta);
		}

		return to;
	}

	double KinematicBicycleModel::GetSteeringAngleFromTurningRadius(double radius)
	{
		if (radius < m_rearToCenter) {
			PP_WARN("The turning radius is too small. Received {0}, minimum possible value is {1}", radius, m_rearToCenter);
			return M_PI_2;
		}
		return atan(m_wheelbase / sqrt(std::pow(radius, 2) - std::pow(m_rearToCenter, 2)));
	}
}
