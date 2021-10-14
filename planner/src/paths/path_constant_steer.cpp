#include "path_constant_steer.h"
#include "models/kinematic_bicycle_model.h"

namespace Planner {
	PathConstantSteer::PathConstantSteer(KinematicBicycleModel* model, const Pose2D<>& init, double steering, double length, Direction direction) :
		Path<Pose2D<>>(init, length), m_model(model), m_steering(steering), m_direction(direction)
	{
		m_final = Interpolate(1.0);
	}

	Pose2D<> PathConstantSteer::Interpolate(double ratio) const
	{
		return m_model->ConstantSteer(m_init, m_steering, m_length * ratio, m_direction);
	}

	void PathConstantSteer::Truncate(double ratio)
	{
		m_final = Interpolate(ratio);
		m_length *= ratio;
	}
}
