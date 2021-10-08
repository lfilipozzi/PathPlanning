#pragma once

#include <vector>
#include <array>
#include <random>

namespace Planner {

	/**
	 * @brief Interface to define the configuration space.
	 * @details The data stored in the template type State must be continuous in
	 * memory without offset and of type T.
	 */
	template <typename State, int Dimension, typename T = double>
	class StateSpace {
		static_assert(sizeof(State) == Dimension * sizeof(T));

	public:
		StateSpace(std::array<std::array<double, 2>, Dimension> bounds) :
			m_bounds(bounds)
		{
			if (!s_randomDevice) {
				s_randomDevice = makeScope<std::random_device>();
				s_engine = makeScope<std::mt19937_64>((*s_randomDevice)());
				s_uniformDistribution = std::uniform_real_distribution<>(0.0, std::nextafter(1.0, std::numeric_limits<T>::max()));
				s_gaussianDistribution = std::normal_distribution<>(0.0, 1.0);
			}
		}
		virtual ~StateSpace() = default;

		void SetBounds(std::array<std::array<double, 2>, Dimension> bounds) { m_bounds = bounds; }
		std::array<std::array<double, 2>, Dimension> GetBounds() const { return m_bounds; };

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

		/// @brief Compute the distance between two states.
		virtual double ComputeDistance(const State& from, const State& to) = 0;

		/// @brief Sample the configuration space using a uniform distribution.
		State SampleUniform()
		{
			State state;
			std::array<T, Dimension> sample;
			for (unsigned int i = 0; i < Dimension; i++) {
				auto lb = m_bounds[i][0];
				auto ub = m_bounds[i][1];
				auto range = ub - lb;
				sample[i] = lb + range * s_uniformDistribution(*s_engine);
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
				sample[i] = mean + dev * s_gaussianDistribution(*s_engine);
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

	private:
		inline static Scope<std::random_device> s_randomDevice;
		inline static Scope<std::mt19937_64> s_engine;
		inline static std::uniform_real_distribution<> s_uniformDistribution;
		inline static std::normal_distribution<> s_gaussianDistribution;
	};
}
