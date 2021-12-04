#include "path_constant_steer.h"
#include "models/kinematic_bicycle_model.h"

namespace Planner {
	PathConstantSteer::PathConstantSteer(const Ref<KinematicBicycleModel>& model, const Pose2d& init, double steering, double length, Direction direction) :
		PathNonHolonomicSE2Base(init, length), m_model(model), m_steering(steering), m_direction(direction)
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

	void PathConstantSteer::TimeFlipTransform()
	{
		PathNonHolonomicSE2Base::TimeFlipTransform();
		// clang-format off
		switch (m_direction) {
			case Direction::Forward: m_direction = Direction::Backward; break;
			case Direction::Backward: m_direction = Direction::Forward; break;
			default: break;
		}
		// clang-format on
	}

	Direction PathConstantSteer::GetDirection(double /*ratio*/) const
	{
		return m_direction;
	}
}
