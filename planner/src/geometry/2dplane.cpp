#include "geometry/2dplane.h"

namespace Planner {
	template <>
	int Pose2<int>::WrapTheta() const
	{
		return theta;
	}
}
