#pragma once

#include "core/base.h"

#include <random>

namespace Planner {

	template <typename T>
	class Random {
	public:
		static void Init()
		{
			if (!s_randomDevice) {
				s_randomDevice = makeScope<std::random_device>();
				s_engine = makeScope<std::mt19937_64>((*s_randomDevice)());
				s_uniformDistribution = std::uniform_real_distribution<T>(0.0, std::nextafter(1.0, std::numeric_limits<T>::max()));
				s_gaussianDistribution = std::normal_distribution<T>(0.0, 1.0);
			}
		}

		/// @brief Sample uniformly on the interval [lb, ub].
		static T SampleUniform(T lb, T ub)
		{
			auto range = ub - lb;
			return lb + range * s_uniformDistribution(*s_engine);
		}

		/// @brief Sample with a Gaussian distribution of mean @mean and
		/// standard deviation @stdDev.
		static T SampleGaussian(const T& mean, const T& stdDev)
		{
			return mean + stdDev * s_gaussianDistribution(*s_engine);
		}

	private:
		Random() = default;

	private:
		inline static Scope<std::random_device> s_randomDevice;
		inline static Scope<std::mt19937_64> s_engine;
		inline static std::uniform_real_distribution<T> s_uniformDistribution;
		inline static std::normal_distribution<T> s_gaussianDistribution;
	};
}
