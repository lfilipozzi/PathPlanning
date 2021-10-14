#pragma once

#include "utils/random.h"

#include <vector>
#include <array>

namespace Planner {

	/// @brief Interface to define the configuration space.
	/// @details The data stored in the template type State must be continuous in
	/// memory without offset and of type T.
	template <typename State, int Dimension, typename T = double>
	class StateSpace {
		static_assert(sizeof(State) == Dimension * sizeof(T));

	public:
		StateSpace(std::array<std::array<double, 2>, Dimension> bounds) :
			m_bounds(bounds)
		{
			Random<T>::Init();
		}
		virtual ~StateSpace() = default;

		/// @brief Compute the distance between two states.
		virtual double ComputeDistance(const State& from, const State& to) = 0;

		void SetBounds(std::array<std::array<double, 2>, Dimension> bounds) { m_bounds = bounds; }
		const std::array<std::array<double, 2>, Dimension>& GetBounds() const { return m_bounds; }
		std::array<std::array<double, 2>, Dimension>& GetBounds() { return m_bounds; }

		/// @brief Modify the state if necessary to enforce state bounds
		/// @param[in,out] state The state to enforce bound.
		void EnforceBounds(State& state)
		{
			T* basePtr = reinterpret_cast<T*>(&state);
			for (unsigned int i = 0; i < Dimension; i++) {
				auto lb = m_bounds[i][0];
				auto ub = m_bounds[i][1];
				T* ptr = basePtr + i;
				*ptr = std::max(lb, std::min(*ptr, ub));
			}
		}

		/// @brief Check if a state validate the bounds
		bool ValidateBounds(const State& state)
		{
			bool validate = true;

			const T* basePtr = reinterpret_cast<const T*>(&state);
			for (unsigned int i = 0; i < Dimension; i++) {
				auto lb = m_bounds[i][0];
				auto ub = m_bounds[i][1];
				T* ptr = basePtr + i;
				if (*ptr < lb || *ptr > ub) {
					validate = false;
					break;
				}
			}

			return validate;
		}

		/// @brief Sample the configuration space using a uniform distribution.
		State SampleUniform()
		{
			State state;
			std::array<T, Dimension> sample;
			for (unsigned int i = 0; i < Dimension; i++) {
				auto lb = m_bounds[i][0];
				auto ub = m_bounds[i][1];
				sample[i] = Random<T>::SampleUniform(lb, ub);
			}
			memcpy(&state, sample.data(), sizeof(State));
			return state;
		}
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
		State SampleGaussian(const State& meanState, const State& stdDev)
		{
			State state;
			std::array<T, Dimension> sample;
			const T* meanStateBasePtr = reinterpret_cast<const T*>(&meanState);
			const T* stdDevBasePtr = reinterpret_cast<const T*>(&stdDev);
			for (unsigned int i = 0; i < Dimension; i++) {
				T mean = *(meanStateBasePtr + i);
				T dev = *(stdDevBasePtr + i);
				sample[i] = Random<T>::SampleGaussian(mean, dev);
			}
			memcpy(&state, sample.data(), sizeof(State));
			EnforceBounds(state);
			return state;
		}
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

	protected:
		std::array<std::array<double, 2>, Dimension> m_bounds;
	};
}
