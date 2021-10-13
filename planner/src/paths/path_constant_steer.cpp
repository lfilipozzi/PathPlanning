#include "path_constant_steer.h"

namespace Planner {
	PathConstantSteer::PathConstantSteer(const Pose2D<>& init, double steering, double length, Direction direction) :
		Path<Pose2D<>>(init, length), m_steering(steering), m_direction(direction)
	{
		m_final = Interpolate(1.0);
	}

	Pose2D<> PathConstantSteer::Interpolate(double ratio) const
	{
		double L = 2.6; // Wheelbase
		double b = L / 2; // Distance to rear axle

		double d = m_length * ratio;
		if (m_direction == Direction::Backward)
			d = -d;

		// Get current pose
		Pose2D<> interp = m_init;

		// Update pose
		double& theta = interp.theta;
		double beta = atan(b * tan(m_steering) / L);
		interp.x += d * cos(theta + beta);
		interp.y += d * sin(theta + beta);
		interp.theta += d * cos(beta) * tan(m_steering) / L;

		return interp;
	}

	void PathConstantSteer::Truncate(double ratio)
	{
		m_final = Interpolate(ratio);
		m_length *= ratio;
	}
}
