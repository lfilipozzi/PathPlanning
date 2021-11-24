#pragma once

#include "utils/random.h"

#include <vector>
#include <array>

namespace Planner {

	/// @brief Interface to define the configuration space.
	/// @details The data stored in the template type State must be continuous in
	/// memory without offset and of type T.
	template <typename State, int Dimension, typename T>
	class StateSpace {

	public:
		StateSpace(const std::array<State, 2>& bounds) :
			bounds(bounds)
		{
			Random<T>::Init();
		}
		StateSpace(const State& lb, const State& ub) :
			StateSpace({ lb, ub }) { }
		virtual ~StateSpace() = default;

		/// @brief Modify the state if necessary to enforce state bounds
		/// @param[in,out] state The state to enforce bound.
		virtual void EnforceBounds(State& state) = 0;

		/// @brief Check if a state validate the bounds
		virtual bool ValidateBounds(const State& state) = 0;

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
		virtual State SampleGaussian(const State& meanState, const State& stdDev) = 0;
		/// @overload
		std::vector<State> SampleGaussian(const State& meanState, const State& stdDev, unsigned int numSamples)
		{
			std::vector<State> states;
			states.reserve(numSamples);
			for (unsigned int i = 0; i < numSamples; i++) {
				states[i] = SampleGuassian(meanState, stdDev);
			}
			return states;
		}

	public:
		static constexpr int dimension = Dimension;
		const std::array<State, 2> bounds;
	};

}
