#pragma once

#include "geometry/2dplane.h"
#include "rrt_star.h"

namespace Planner {
	class RRTStarStateSpace2D : public RRTStarStateSpace<Point2D> {
	public:
		virtual double ComputeDistance(const Point2D& from, const Point2D& to) const override
		{
			auto delta = from - to;
			return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
		}

		virtual bool IsTransitionCollisionFree(const Point2D& /*from*/, const Point2D& /*to*/) override
		{
			return true;
		}

		virtual Point2D Sample() override
		{
			const double width = 5;
			const double height = 5;
			return Point2D(drand48() * width, drand48() * height);
		}

		virtual Point2D SteerTowards(const Point2D& source, const Point2D& target) override
		{
			Point2D delta = target - source;
			delta = delta / delta.norm();

			Point2D val = source + delta * 0.1;
			return val;
		}

		virtual std::tuple<double, bool> SteerExactly(const Point2D& source, const Point2D& target) override
		{
			return { 0.0, true };
		}
	};
}
