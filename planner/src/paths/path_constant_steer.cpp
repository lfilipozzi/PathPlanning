#include "path_constant_steer.h"
#include "models/kinematic_bicycle_model.h"

namespace Planner {
	PathConstantSteer::PathConstantSteer(KinematicBicycleModel* model, const Pose2d& init, double steering, double length, Direction direction) :
		PlanarPath(init, length), m_model(model), m_steering(steering), m_direction(direction)
	{
		m_final = Interpolate(1.0);
	}

	Pose2d PathConstantSteer::Interpolate(double ratio) const
	{
		return m_model->ConstantSteer(m_init, m_steering, m_length * ratio, m_direction);
	}

	void PathConstantSteer::Truncate(double ratio)
	{
		m_final = Interpolate(ratio);
		m_length *= ratio;
	}

	Direction PathConstantSteer::GetDirection(double /*ratio*/) const
	{
		return m_direction;
	}

	double PathConstantSteer::ComputeCost(double /*directionSwitchingCost*/, double reverseCostMultiplier, double forwardCostMultiplier) const
	{
		switch (m_direction) {
		case Direction::Forward:
			return forwardCostMultiplier * m_length;
		case Direction::Backward:
			return reverseCostMultiplier * m_length;
		default:
			return 0.0;
		}
	}
}
