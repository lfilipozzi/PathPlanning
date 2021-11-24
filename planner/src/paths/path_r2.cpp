#include "paths/path_r2.h"
#include "core/base.h"

namespace Planner {
	PathR2::PathR2(const Point2d& from, const Point2d& to) :
		Path<Point2d>(from, (to - from).norm())
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

	Direction PathR2::GetDirection(double /*ratio*/) const
	{
		return Direction::Forward;
	}

	double PathR2::ComputeCost(double /*directionSwitchingCost*/, double /*reverseCostMultiplier*/, double forwardCostMultiplier) const
	{
		return forwardCostMultiplier * m_length;
	}

	Ref<Path<Point2d>> PathConnectionR2::Connect(const Point2d& from, const Point2d& to)
	{
		return makeRef<PathR2>(from, to);
	}
}
