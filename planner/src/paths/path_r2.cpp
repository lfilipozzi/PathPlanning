#include "paths/path_r2.h"
#include "core/base.h"

namespace Planner {
	PathR2::PathR2(const Point2d& from, const Point2d& to) :
		PathR2Base(from, (to - from).norm())
	{
		m_final = to;
	}

	Point2d PathR2::Interpolate(double ratio) const
	{
		Point2d interp;
		interp = (1 - ratio) * m_init + ratio * m_final;
		return interp;
	}

	void PathR2::Truncate(double ratio)
	{
		m_final = Interpolate(ratio);
		m_length *= ratio;
	}

	Ref<PathR2Base> PathConnectionR2::Connect(const Point2d& from, const Point2d& to)
	{
		return makeRef<PathR2>(from, to);
	}
}
