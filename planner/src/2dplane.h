#pragma once

#include <eigen3/Eigen/Dense>

#include "core/hash.h"
#include "state_space.h"

using Vertex = Eigen::Vector2d;

namespace std {
	template <>
	struct hash<Vertex> {
		std::size_t operator()(const Vertex& state) const
		{
			std::size_t seed = 0;
			HashCombine(seed, state.x());
			HashCombine(seed, state.y());
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
			Vertex delta = source - target;
			double dist = sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
			return { dist, true };
		}

		virtual std::vector<Vertex> GetNeighborStates(Vertex state) override
		{
			double delta = 0.2;
			std::vector<Vertex> neighbors;
			neighbors.reserve(8);
			neighbors.push_back({ state.x() + 1.0 * delta, state.y() + 1.0 * delta });
			neighbors.push_back({ state.x() + 1.0 * delta, state.y() + 0.0 * delta });
			neighbors.push_back({ state.x() + 1.0 * delta, state.y() - 1.0 * delta });
			neighbors.push_back({ state.x() + 0.0 * delta, state.y() - 1.0 * delta });
			neighbors.push_back({ state.x() - 1.0 * delta, state.y() - 1.0 * delta });
			neighbors.push_back({ state.x() - 1.0 * delta, state.y() + 0.0 * delta });
			neighbors.push_back({ state.x() - 1.0 * delta, state.y() + 1.0 * delta });
			neighbors.push_back({ state.x() + 0.0 * delta, state.y() + 1.0 * delta });
			return neighbors;
		}
	};
}
