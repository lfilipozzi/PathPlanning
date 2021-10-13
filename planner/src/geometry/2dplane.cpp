#include "geometry/2dplane.h"

namespace Planner {
	template <>
	int Pose2D<int>::WrapTheta() const
	{
		return theta;
	}
}
