#pragma once

#include "geometry/2dplane.h"
#include "state_space_2d.h"
#include "rrt.h"

namespace Planner {
	class RRTStateSpace2D : public RRTStateSpace<Point2D>, public StateSpace2D<Point2D> {
	public:
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
	};
}
