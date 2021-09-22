#pragma once

#include "state_space.h"

namespace Planner {
	template <typename T>
	class StateSpace2D : public virtual StateSpace<T> {
	public:
		virtual double ComputeDistance(const T& from, const T& to) const override
		{
			auto delta = from - to;
			return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
		}

		virtual bool IsTransitionCollisionFree(const T& /*from*/, const T& /*to*/) override
		{
			return true;
		}
	};
}
