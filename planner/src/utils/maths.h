#pragma once

#include <cmath>

namespace Planner {

	namespace Maths {
		template <typename T>
		T Modulo(T in, T mod)
		{
			// Return the modulo of x by y
			T out = fmod(in, mod);
			if (out < 0)
				out += mod;
			return out;
		}
	}
}
