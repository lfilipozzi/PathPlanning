#include "paths/path_se2.h"
#include "core/base.h"

namespace Planner {
	PathSE2::PathSE2(const Pose2d& from, const Pose2d& to) :
		PathSE2Base(from, (to.position - from.position).norm())
	{
		m_final = to;
	}

	Pose2d PathSE2::Interpolate(double ratio) const
	{
		Pose2d interp;
		interp.position = (1 - ratio) * m_init.position + ratio * m_final.position;
		interp.theta = (1 - ratio) * m_init.theta + ratio * m_final.theta;
		return interp;
	}

	void PathSE2::Truncate(double ratio)
	{
		m_final = Interpolate(ratio);
		m_length *= ratio;
	}

	Ref<PathSE2Base> PathConnectionSE2::Connect(const Pose2d& from, const Pose2d& to)
	{
		return makeRef<PathSE2>(from, to);
	}
}
