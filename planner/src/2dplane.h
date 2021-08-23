#pragma once

#include <eigen3/Eigen/Dense>

#include "state_space.h"

using Vertex = Eigen::Vector2d;

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
	template <>
	struct hash<Vertex> {
		std::size_t operator()(const Vertex& state) const
		{
			using std::hash;
			using std::size_t;
			using std::string;

			std::size_t seed = 0;
			hash_combine(seed, state.x());
			hash_combine(seed, state.y());
			return seed;
		}
	};
}

namespace Planner {
	class TestStateSpace : public virtual StateSpace<Vertex> {
	public:
		virtual double ComputeDistance(const Vertex& from, const Vertex& to) const override
		{
			Vertex delta = from - to;
			return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
		}

		virtual bool IsTransitionCollisionFree(const Vertex& /*from*/, const Vertex& /*to*/) override
		{
			return true;
		}
	};

	class TestRRTStateSpace : public RRTStateSpace<Vertex>, public TestStateSpace {
	public:
		virtual Vertex Sample() override
		{
			const double width = 5;
			const double height = 5;
			return Vertex(drand48() * width, drand48() * height);
		}

		virtual Vertex SteerTowards(const Vertex& source, const Vertex& target) override
		{
			Vertex delta = target - source;
			delta = delta / delta.norm();

			Vertex val = source + delta * 0.1;
			return val;
		}

		virtual std::tuple<double, bool> SteerExactly(const Vertex& source, const Vertex& target) override
		{
			return { 0.0, true };
		}
	};

	class TestAStarStateSpace : public AStarStateSpace<Vertex>, public TestStateSpace {
	public:
		virtual std::tuple<double, bool> SteerExactly(const Vertex& source, const Vertex& target) override
		{
			return { 0.0, true };
		}

		virtual std::vector<Vertex> GetNeighborStates(Vertex state) override
		{
			return std::vector<Vertex>();
		}
	};
}
