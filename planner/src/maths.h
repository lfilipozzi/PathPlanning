#pragma once

#include <cmath>

namespace Planner {

	namespace Maths {
		float Modulo(float in, float mod)
		{
			// Return the modulo of x by y
			float out = fmod(in, mod);
			if (out < 0)
				out += mod;
			return out;
		}

		double Modulo(double in, double mod)
		{
			// Return the modulo of x by y
			double out = fmod(in, mod);
			if (out < 0)
				out += mod;
			return out;
		}
	}
}
