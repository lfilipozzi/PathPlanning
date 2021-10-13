#pragma once

#include "paths/path.h"
#include "geometry/pose.h"

namespace Planner {

	/**
	 * @brief Represent a planar path of constant steering angle using a 
	 * kinematic bicycle model.
	 */
	class PathConstantSteer : public Path<Pose2D<>> {
		using State = Pose2D<>;

	public:
		PathConstantSteer(const State& init, double steering = 0.0, double length = 0.0, Direction direction = Direction::Forward) :
			Path<State>(init, length), m_steering(steering), m_direction(direction)
		{
			m_final = Interpolate(1.0);
		}

		virtual State Interpolate(double ratio) const override
		{
			double L = 2.6; // Wheelbase
			double b = L / 2; // Distance to rear axle

			double d = m_length * ratio;
			if (m_direction == Direction::Backward)
				d = -d;

			// Get current pose
			State interp = m_init;

			// Update pose
			double& theta = interp.theta;
			double beta = atan(b * tan(m_steering) / L);
			interp.x += d * cos(theta + beta);
			interp.y += d * sin(theta + beta);
			interp.theta += d * cos(beta) * tan(m_steering) / L;

			return interp;
		}

		virtual void Truncate(double ratio) override
		{
			m_final = Interpolate(ratio);
			m_length *= ratio;
		}

	private:
		double m_steering;
		Direction m_direction;
	};
}
