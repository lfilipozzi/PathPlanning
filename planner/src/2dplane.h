#pragma once

#include <eigen3/Eigen/Dense>

#include "state_space.h"
#include "state_validator.h"

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

class TestStateValidator : public Planner::StateValidator<Vertex> {
public:
	virtual bool ValidateState(const Vertex& state) override
	{
		const double width = 5;
		const double height = 5;
		return state.x() >= 0 && state.y() >= 0 && state.x() < width && state.y() < height;
	}

	virtual bool ValidateTransition(const Vertex& /*from*/, const Vertex& /*to*/) override
	{
		return true;
	}
};

class TestStateSpace : public Planner::StateSpace<Vertex> {
public:
	virtual double ComputeDistance(const Vertex& from, const Vertex& to) const override
	{
		Vertex delta = from - to;
		return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
	}

	virtual Vertex CreateRandomState() override
	{
		const double width = 5;
		const double height = 5;
		return Vertex(drand48() * width, drand48() * height);
	}

	virtual Vertex CreateIncrementalState(const Vertex& source, const Vertex& target, double stepSize) override
	{
		Vertex delta = target - source;
		delta = delta / delta.norm();

		Vertex val = source + delta * stepSize;
		return val;
	}
};
