#pragma once

#include <vector>

namespace Planner {

	/**
	 * @brief Interface to define the configuration space.
	 */
	template <typename State>
	class StateSpace {
	public:
		StateSpace() = default;
		virtual ~StateSpace() = default;

		// TODO Add bounds on state-space

		/// @brief Define how to interpolate between states
		virtual State Interpolate(const State& from, const State& to, float ratio) = 0;

		/// @brief Compute the distance between two states.
		virtual double ComputeDistance(const State& from, const State& to) = 0;

		/// @brief Sample the configuration space using a uniform distribution.
		virtual State SampleUniform() = 0;
		/// @overload
		std::vector<State> SampleUniform(unsigned int numSamples)
		{
			std::vector<State> states;
			states.reserve(numSamples);
			for (unsigned int i = 0; i < numSamples; i++) {
				states[i] = SampleUniform();
			}
			return states;
		}

		/// @brief Sample the configuration space using a Gaussian distribution.
		virtual State SampleGaussian(const State& state, float stdDev) = 0;
		/// @overload
		std::vector<State> SampleGaussian(const State& state, float stdDev, unsigned int numSamples)
		{
			std::vector<State> states;
			states.reserve(numSamples);
			for (unsigned int i = 0; i < numSamples; i++) {
				states[i] = SampleGuassian(state, stdDev);
			}
			return states;
		}
	};
}
