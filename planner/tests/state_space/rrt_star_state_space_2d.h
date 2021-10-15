#pragma once

#include "geometry/2dplane.h"
#include "algo/rrt_star.h"

namespace Planner {
	class RRTStarStateSpace2D : public RRTStarStateSpace<Point2d> {
	public:
		virtual double ComputeDistance(const Point2d& from, const Point2d& to) const override
		{
			auto delta = from - to;
			return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
		}

		virtual bool IsTransitionCollisionFree(const Point2d& /*from*/, const Point2d& /*to*/) override
		{
			return true;
		}

		virtual Point2d Sample() override
		{
			const double width = 5;
			const double height = 5;
			return Point2d(drand48() * width, drand48() * height);
		}

		virtual Point2d SteerTowards(const Point2d& source, const Point2d& target) override
		{
			Point2d delta = target - source;
			delta = delta / delta.norm();

			Point2d val = source + delta * 0.1;
			return val;
		}

		virtual std::tuple<double, bool> SteerExactly(const Point2d& /*source*/, const Point2d& /*target*/) override
		{
			return { 0.0, true };
		}
	};
}
