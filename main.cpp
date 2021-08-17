#include <iostream>

#include "tree.h"
#include "rrt.h"

#include <eigen3/Eigen/Dense>

using T = Eigen::Vector2d;

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
	template <>
	struct hash<T>
	{
		std::size_t operator()(const T& state) const
		{
			using std::size_t;
			using std::hash;
			using std::string;

			std::size_t seed = 0;
			hash_combine(seed, state.x());
			hash_combine(seed, state.y());
			return seed;
		}
	};
}

class TestStateValidator : public Planner::StateValidator<Eigen::Vector2d> {
public:
	virtual bool ValidateState(const T& state) override
	{
		const double width = 5;
		const double height = 5;
		return state.x() >= 0 && state.y() >= 0 && state.x() < width && state.y() < height;
	}

	virtual bool ValidateTransition(const T& /*from*/, const T& /*to*/) override
	{
		return true;
	}
};

class TestStateSpace : public Planner::StateSpace<Eigen::Vector2d> {
public:
	virtual double ComputeDistance(const T& from, const T& to) const override
	{
		T delta = from - to;
		return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
	}

	virtual T CreateRandomState() override
	{
		const double width = 5;
		const double height = 5;
		return T(drand48() * width, drand48() * height);
	}

	virtual T CreateIncrementalState(const T& source, const T& target, double stepSize) override
	{
		T delta = target - source;
		delta = delta / delta.norm();

		T val = source + delta * stepSize;
		return val;
	}
};

int main(int /*argc*/, char** /*argv*/)
{
	Planner::Scope<TestStateSpace> stateSpace = Planner::makeScope<TestStateSpace>();
	Planner::Scope<TestStateValidator> validator = Planner::makeScope<TestStateValidator>();

	Planner::RRT<Eigen::Vector2d, 2> rrt(std::move(stateSpace), std::move(validator));

	Planner::RRT<Eigen::Vector2d, 2>::Parameters parameters;
	rrt.SetParameters(parameters);

	Eigen::Vector2d start = { 0.0, 0.0 };
	Eigen::Vector2d goal = { 2.0, 2.0 };
	rrt.SetInitState(start);
	rrt.SetGoalState(goal);

	rrt.SearchPath();
	std::vector<Eigen::Vector2d> path = rrt.GetPath();

	std::cout << "Path: " << std::endl;
	for (auto point : path) {
		std::cout << "x=" << point.x() << ", y=" << point.y() << std::endl;
	}

	rrt.Clear();
	
	return 0;
}

// int main(void) {
// 	T pointA;
// 	Planner::Scope<Planner::Tree<T,2>::Node> nodeA= Planner::makeScope<Planner::Tree<T,2>::Node>(pointA);
// 	
// 	T pointB;
// 	Planner::Scope<Planner::Tree<T,2>::Node> nodeB= Planner::makeScope<Planner::Tree<T,2>::Node>(pointB);
// 	
// 	std::unordered_map<T, Planner::Tree<T,2>::Node*> m_exploredNodeMap;
// 	m_exploredNodeMap.insert(std::make_pair(nodeA->GetState(), nodeA.get()));
// 	m_exploredNodeMap.insert(std::make_pair(nodeB->GetState(), nodeB.get()));
// 	
// 	nodeA.reset();
// 	m_exploredNodeMap.clear();
// 	
// // 	m_exploredNodeMap.clear();
// // 	node.reset();
// }

// int main(void) {
// 	T pointA;
// 	T pointB;
// 	T pointC;
// 	T pointD;
// 	
// 	Planner::Tree<T,2> tree;
// 	auto nodeA = tree.CreateRootNode(pointA);
// 	auto nodeB = tree.Extend(pointB, nodeA);
// 	auto nodeC = tree.Extend(pointC, nodeB);
// 	auto nodeD = tree.Extend(pointD, nodeB);
// 	
// 	tree.Clear();
// }





