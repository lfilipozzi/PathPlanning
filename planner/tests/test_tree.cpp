#include "utils/tree.h"
#include "core/base.h"
#include "geometry/2dplane.h"

namespace Planner {
	using Vertex = Point2d;
	using VertexTree = Tree<Vertex, 2>;

	void TestNodeGetDepth()
	{
		VertexTree tree;

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

	void TestNodeIsDescendantOf()
	{
		VertexTree tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		auto nodeD = tree.Extend(pointD, nodeC);

		assert(nodeB->IsDescendantOf(nodeA));
		assert(nodeC->IsDescendantOf(nodeA));
		assert(!nodeA->IsDescendantOf(nodeD));
	}

	void TestNodeIsAncestorOf()
	{
		VertexTree tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		auto nodeD = tree.Extend(pointD, nodeC);

		assert(nodeA->IsAncestorOf(nodeB));
		assert(nodeA->IsAncestorOf(nodeC));
		assert(!nodeD->IsAncestorOf(nodeA));
	}

	void TestNodeReplaceChild()
	{
		VertexTree tree;

		Vertex pointA = { 0, 0 };
		Vertex pointB = { 1, 0 };
		Vertex pointC = { 2, 0 };
		Vertex pointD = { 3, 0 };
		Vertex pointE = { 4, 0 };

		auto nodeA = tree.CreateRootNode(pointA);
		auto nodeB = tree.Extend(pointB, nodeA);
		auto nodeC = tree.Extend(pointC, nodeB);
		[[maybe_unused]] auto nodeD = tree.Extend(pointD, nodeC);

		auto scopeE = makeScope<VertexTree::Node>(pointE);
		auto nodeE = scopeE.get();
		auto scope = nodeA->ReplaceChild(nodeB, std::move(scopeE), true);

		assert(nodeA->IsAncestorOf(nodeE));
		assert(nodeE->IsAncestorOf(nodeC));
		assert(scope->GetParent() == nullptr);
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

	void TestReparent()
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
		tree.Reparent(nodeD, nodeB);
		assert(nodeD->GetParent() == nodeB);

		// The new parent is a child
		tree.Reparent(nodeA, nodeC);
		assert(nodeA->GetParent() == nodeC);
	}
}

int main()
{
	PP_INIT;
	Planner::TestNodeGetDepth();
	Planner::TestNodeIsDescendantOf();
	Planner::TestNodeIsAncestorOf();
	Planner::TestNodeReplaceChild();
	Planner::TestGetNearestNodes();
	Planner::TestGetNearestNode();
	Planner::TestReparent();

	return 0;
}
