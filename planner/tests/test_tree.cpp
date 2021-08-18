#include "tree.h"
#include "2dplane.h"

namespace Planner {
	void TestNodeGetDepth()
	{
		Planner::Tree<Vertex, 2> tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		auto nodeD = tree.Extend(pointD, nodeC);

		assert(nodeA->GetDepth() == 0);
		assert(nodeB->GetDepth() == 1);
		assert(nodeC->GetDepth() == 2);
		assert(nodeD->GetDepth() == 3);
	}

	void TestNodeIsChildOf()
	{
		Planner::Tree<Vertex, 2> tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		auto nodeD = tree.Extend(pointD, nodeC);

		assert(nodeB->IsChildOf(nodeA));
		assert(nodeC->IsChildOf(nodeA));
		assert(!nodeA->IsChildOf(nodeD));
	}

	void TestNodeIsParentOf()
	{
		Planner::Tree<Vertex, 2> tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		auto nodeD = tree.Extend(pointD, nodeC);

		assert(nodeA->IsParentOf(nodeB));
		assert(nodeA->IsParentOf(nodeC));
		assert(!nodeD->IsParentOf(nodeA));
	}

	void TestGetNearestNodes()
	{
		Planner::Tree<Vertex, 2> tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		auto nodeD = tree.Extend(pointD, nodeC);

		Vertex point = { 1.1, 0 };
		auto nearest = tree.GetNearestNode(point);
		assert(nearest == nodeB);
	}

	void TestGetNearestNode()
	{
		Planner::Tree<Vertex, 2> tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		auto nodeD = tree.Extend(pointD, nodeC);

		unsigned int nn = 2;
		Vertex point = { 2.4, 0.2 };
		auto nearestNodes = tree.GetNearestNodes(point, nn);
		assert(nearestNodes.size() == nn);
		assert(std::count(nearestNodes.begin(), nearestNodes.end(), nodeC));
		assert(std::count(nearestNodes.begin(), nearestNodes.end(), nodeD));
	}

	void TestRewire()
	{
		Planner::Tree<Vertex, 2> tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		auto nodeD = tree.Extend(pointD, nodeC);

		// The new parent is not a child
		tree.Rewire(nodeD, nodeB);
		assert(nodeD->GetParent() == nodeB);

		// The new parent is a child
		tree.Rewire(nodeA, nodeC);
		assert(nodeA->GetParent() == nodeC);
	}
}

int main()
{
	Planner::TestNodeGetDepth();
	Planner::TestNodeIsChildOf();
	Planner::TestNodeIsParentOf();
	Planner::TestGetNearestNodes();
	Planner::TestGetNearestNode();
	Planner::TestRewire();

	return 0;
}
