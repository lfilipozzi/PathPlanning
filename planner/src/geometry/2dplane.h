#pragma once

#include <eigen3/Eigen/Dense>

#include "core/hash.h"
#include "state_space.h"

using Point2D = Eigen::Vector2d;
using Point2DInt = Eigen::Vector2i;

namespace std {
	template <>
	struct hash<Point2D> {
		std::size_t operator()(const Point2D& state) const
		{
			std::size_t seed = 0;
			HashCombine(seed, state.x());
			HashCombine(seed, state.y());
			return seed;
		}
	};

	template <>
	struct hash<Point2DInt> {
		std::size_t operator()(const Point2DInt& state) const
		{
			std::size_t seed = 0;
			HashCombine(seed, state.x());
			HashCombine(seed, state.y());
			return seed;
		}
	};
}

namespace Planner {
	template <typename T>
	class TestStateSpaceEigen2D : public virtual StateSpace<T> {
	public:
		virtual double ComputeDistance(const T& from, const T& to) const override
		{
			auto delta = from - to;
			return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
		}

		virtual bool IsTransitionCollisionFree(const T& /*from*/, const T& /*to*/) override
		{
			return true;
		}
	};

	class TestRRTStateSpace : public RRTStateSpace<Point2D>, public TestStateSpaceEigen2D<Point2D> {
	public:
		virtual Point2D Sample() override
		{
			const double width = 5;
			const double height = 5;
			return Point2D(drand48() * width, drand48() * height);
		}

		virtual Point2D SteerTowards(const Point2D& source, const Point2D& target) override
		{
			Point2D delta = target - source;
			delta = delta / delta.norm();

			Point2D val = source + delta * 0.1;
			return val;
		}

		virtual std::tuple<double, bool> SteerExactly(const Point2D& source, const Point2D& target) override
		{
			return { 0.0, true };
		}
	};

	class TestAStarStateSpace : public AStarStateSpace<Point2DInt>, public TestStateSpaceEigen2D<Point2DInt> {
	public:
		virtual std::tuple<double, bool> GetTransition(const Point2DInt& source, const Point2DInt& target) override
		{
			auto delta = source - target;
			double dist = sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
			return { dist, true };
		}

		virtual std::vector<Point2DInt> GetNeighborStates(const Point2DInt& state) override
		{
			double delta = 1.0;
			std::vector<Point2DInt> neighbors;
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
