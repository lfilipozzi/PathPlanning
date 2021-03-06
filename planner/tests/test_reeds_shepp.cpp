#include "core/base.h"
#include "paths/path_reeds_shepp.h"

#include <unordered_set>

namespace Planner {
	void Test(const Pose2d& start, const Pose2d& goal, const std::unordered_set<ReedsShepp::PathWords>& optimalWords)
	{
		ReedsShepp::PathWords word = ReedsShepp::PathWords::NoPath;
		double unit = 1.0;
		double reverseCostMultiplier = 1.0, forwardCostMultiplier = 1.0, directionSwitchingCost = 0.0;

		// Check solver find correct shortest path
		auto pathSegment = ReedsShepp::Solver::GetShortestPath(start, goal, unit, &word);
		assert(optimalWords.find(word) != optimalWords.end());

		// Check the constructed path leads to the final state
		PathReedsShepp path = { start, pathSegment, unit };
		double tol = 1e-6;
		assert(std::abs(path.GetFinalState().x() - goal.x()) < tol);
		assert(std::abs(path.GetFinalState().y() - goal.y()) < tol);
		assert(std::abs(path.GetFinalState().theta - goal.theta) < tol * M_PI / 180.0);

		// Check solver finds optimal path (when cost correspond to distance traveled)
		ReedsShepp::Solver::GetOptimalPath(start, goal, unit, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost, &word);
		assert(optimalWords.find(word) != optimalWords.end());
	}
}

int main()
{
	PP_INIT;

	Planner::ReedsShepp::PathWords optimalWord;
	Planner::Pose2d start;
	Planner::Pose2d goal;

	// CSC, same turn
	// 0: LfSfLf
	optimalWord = Planner::ReedsShepp::PathWords::LfSfLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 10.0, 10.0, 1.6 };
	Planner::Test(start, goal, { optimalWord });
	// 1: LbSbLb
	optimalWord = Planner::ReedsShepp::PathWords::LbSbLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -10.0, 10.0, -1.6 };
	Planner::Test(start, goal, { optimalWord });
	// 2: RfSfRf
	optimalWord = Planner::ReedsShepp::PathWords::RfSfRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 10.0, -10.0, -1.6 };
	Planner::Test(start, goal, { optimalWord });
	// 3: RbSbRb
	optimalWord = Planner::ReedsShepp::PathWords::RbSbRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -10.0, -10.0, 1.6 };
	Planner::Test(start, goal, { optimalWord });

	// CSC, different turn
	// 4: LfSfRf
	optimalWord = Planner::ReedsShepp::PathWords::LfSfRf;
	start = { 0.0, 0.0, 1.5 };
	goal = { -3.0, 3.0, 1.5 };
	Planner::Test(start, goal, { optimalWord });
	// 5: LbSbRb
	optimalWord = Planner::ReedsShepp::PathWords::LbSbRb;
	start = { 0.0, 0.0, 1.5 };
	goal = { -3.0, -3.0, 1.5 };
	Planner::Test(start, goal, { optimalWord });
	// 6: RfSfLf
	optimalWord = Planner::ReedsShepp::PathWords::RfSfLf;
	start = { 0.0, 0.0, 1.5 };
	goal = { 3.0, 3.0, 1.5 };
	Planner::Test(start, goal, { optimalWord });
	// 7: RbSbLb
	optimalWord = Planner::ReedsShepp::PathWords::RbSbLb;
	start = { 0.0, 0.0, 1.5 };
	goal = { 3.0, -3.0, 1.5 };
	Planner::Test(start, goal, { optimalWord });

	// C|C|C
	// 8: LfRbLf
	optimalWord = Planner::ReedsShepp::PathWords::LfRbLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.1, 0.1, 3.1 };
	Planner::Test(start, goal, { optimalWord });
	// 9: LbRfLb
	optimalWord = Planner::ReedsShepp::PathWords::LbRfLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -0.1, 0.1, -3.1 };
	Planner::Test(start, goal, { optimalWord });
	// 10: RfLbRf. Note: this path is equivalent to LbRfLb
	optimalWord = Planner::ReedsShepp::PathWords::RfLbRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -0.1, -0.1, -3.1 };
	Planner::Test(start, goal, { optimalWord, Planner::ReedsShepp::PathWords::LbRfLb });
	// 11: RbLfRb. Note: this path is equivalent to LfRbLf
	optimalWord = Planner::ReedsShepp::PathWords::RbLfRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.1, -0.1, 3.1 };
	Planner::Test(start, goal, { optimalWord, Planner::ReedsShepp::PathWords::LfRbLf });

	// C|CC
	// 12: LfRbLb
	optimalWord = Planner::ReedsShepp::PathWords::LfRbLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -0.5, -0.5, 0.6 };
	Planner::Test(start, goal, { optimalWord });
	// 13: LbRfLf
	optimalWord = Planner::ReedsShepp::PathWords::LbRfLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.5, -0.5, -0.6 };
	Planner::Test(start, goal, { optimalWord });
	// 14: RfLbRb
	optimalWord = Planner::ReedsShepp::PathWords::RfLbRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -0.5, 0.5, -0.6 };
	Planner::Test(start, goal, { optimalWord });
	// 15: RbLfRf
	optimalWord = Planner::ReedsShepp::PathWords::RbLfRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.5, 0.5, 0.6 };
	Planner::Test(start, goal, { optimalWord });

	// CC|C
	// 16: LfRfLb
	optimalWord = Planner::ReedsShepp::PathWords::LfRfLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, 0.0, -1.6 };
	Planner::Test(start, goal, { optimalWord });
	// 17: LbRbLf
	optimalWord = Planner::ReedsShepp::PathWords::LbRbLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, 0.0, 1.6 };
	Planner::Test(start, goal, { optimalWord });
	// 18: RfLfRb
	optimalWord = Planner::ReedsShepp::PathWords::RfLfRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, 0.0, 1.6 };
	Planner::Test(start, goal, { optimalWord });
	// 19: RbLbRf
	optimalWord = Planner::ReedsShepp::PathWords::RbLbRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, 0.0, -1.6 };
	Planner::Test(start, goal, { optimalWord });

	// CCu|CuC
	// 20: LfRufLubRb
	optimalWord = Planner::ReedsShepp::PathWords::LfRufLubRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.1, 0.1, -0.5 };
	Planner::Test(start, goal, { optimalWord });
	// 21: LbRubLufRf
	optimalWord = Planner::ReedsShepp::PathWords::LbRubLufRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -0.1, 0.1, 0.5 };
	Planner::Test(start, goal, { optimalWord });
	// 22: RfLufRubLb
	optimalWord = Planner::ReedsShepp::PathWords::RfLufRubLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.1, -0.1, 0.5 };
	Planner::Test(start, goal, { optimalWord });
	// 23: RbLubRufLf
	optimalWord = Planner::ReedsShepp::PathWords::RbLubRufLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -0.1, -0.1, -0.5 };
	Planner::Test(start, goal, { optimalWord });

	// C|CuCu|C
	// 24: LfRubLubRf
	optimalWord = Planner::ReedsShepp::PathWords::LfRubLubRf;
	start = { 0.0, 0.0, 3.1 };
	goal = { 0.5, 0.5, 3.1 };
	Planner::Test(start, goal, { optimalWord });
	// 25: LbRufLufRb
	optimalWord = Planner::ReedsShepp::PathWords::LbRufLufRb;
	start = { 0.0, 0.0, 3.1 };
	goal = { -0.5, 0.5, 3.1 };
	Planner::Test(start, goal, { optimalWord });
	// 26: RfLubRubLf
	optimalWord = Planner::ReedsShepp::PathWords::RfLubRubLf;
	start = { 0.0, 0.0, 3.1 };
	goal = { 0.5, -0.5, 3.1 };
	Planner::Test(start, goal, { optimalWord });
	// 27: RbLufRufLb
	optimalWord = Planner::ReedsShepp::PathWords::RbLufRufLb;
	start = { 0.0, 0.0, 3.1 };
	goal = { -0.5, -0.5, 3.1 };
	Planner::Test(start, goal, { optimalWord });

	// C|C(pi/2)SC, same turn
	// 28: LfRbpi2SbLb
	optimalWord = Planner::ReedsShepp::PathWords::LfRbpi2SbLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, -2.0, 2.0 };
	Planner::Test(start, goal, { optimalWord });
	// 29: LbRfpi2SfLf
	optimalWord = Planner::ReedsShepp::PathWords::LbRfpi2SfLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, -2.0, -2.0 };
	Planner::Test(start, goal, { optimalWord });
	// 30: RfLbpi2SbRb
	optimalWord = Planner::ReedsShepp::PathWords::RfLbpi2SbRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, 2.0, -2.0 };
	Planner::Test(start, goal, { optimalWord });
	// 31: RbLfpi2SfRf
	optimalWord = Planner::ReedsShepp::PathWords::RbLfpi2SfRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, 2.0, 2.0 };
	Planner::Test(start, goal, { optimalWord });

	// C|C(pi/2)SC, different turn
	// 32: LfRbpi2SbRb
	optimalWord = Planner::ReedsShepp::PathWords::LfRbpi2SbRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, -2.0, 3.0 };
	Planner::Test(start, goal, { optimalWord });
	// 33: LbRfpi2SfRf
	optimalWord = Planner::ReedsShepp::PathWords::LbRfpi2SfRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, -2.0, -3.0 };
	Planner::Test(start, goal, { optimalWord });
	// 34: RfLbpi2SbLb
	optimalWord = Planner::ReedsShepp::PathWords::RfLbpi2SbLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, 2.0, -3.0 };
	Planner::Test(start, goal, { optimalWord });
	// 35: RbLfpi2SfLf
	optimalWord = Planner::ReedsShepp::PathWords::RbLfpi2SfLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, 2.0, 3.0 };
	Planner::Test(start, goal, { optimalWord });

	// CSC(pi/2)|C, same turn
	// 36: LfSfRfpi2Lb
	optimalWord = Planner::ReedsShepp::PathWords::LfSfRfpi2Lb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, 1.0, -2.0 };
	Planner::Test(start, goal, { optimalWord });
	// 37: LbSbRbpi2Lf
	optimalWord = Planner::ReedsShepp::PathWords::LbSbRbpi2Lf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, 1.0, 2.0 };
	Planner::Test(start, goal, { optimalWord });
	// 38: RfSfLfpi2Rb
	optimalWord = Planner::ReedsShepp::PathWords::RfSfLfpi2Rb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, -1.0, 2.0 };
	Planner::Test(start, goal, { optimalWord });
	// 39: RbSbLbpi2Rf
	optimalWord = Planner::ReedsShepp::PathWords::RbSbLbpi2Rf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, -1.0, -2.0 };
	Planner::Test(start, goal, { optimalWord });

	// CSC(pi/2)|C, different turn
	// 40: LfSfLfpi2Rb
	optimalWord = Planner::ReedsShepp::PathWords::LfSfLfpi2Rb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, 2.0, 3.1 };
	Planner::Test(start, goal, { optimalWord });
	// 41: LbSbLbpi2Rf
	optimalWord = Planner::ReedsShepp::PathWords::LbSbLbpi2Rf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, 2.0, -3.1 };
	Planner::Test(start, goal, { optimalWord });
	// 42: RfSfRfpi2Lb
	optimalWord = Planner::ReedsShepp::PathWords::RfSfRfpi2Lb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, -2.0, -3.1 };
	Planner::Test(start, goal, { optimalWord });
	// 43: RbSbRbpi2Lf
	optimalWord = Planner::ReedsShepp::PathWords::RbSbRbpi2Lf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, -2.0, 3.1 };
	Planner::Test(start, goal, { optimalWord });

	// C|C(pi/2)SC(pi/2)|C
	// 44: LfRbpi2SbLbpi2Rf
	optimalWord = Planner::ReedsShepp::PathWords::LfRbpi2SbLbpi2Rf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.0, -3.0, 0.2 };
	Planner::Test(start, goal, { optimalWord });
	// 45: LbRfpi2SfLfpi2Rb
	optimalWord = Planner::ReedsShepp::PathWords::LbRfpi2SfLfpi2Rb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.0, -3.0, -0.2 };
	Planner::Test(start, goal, { optimalWord });
	// 46: RfLbpi2SbRbpi2Lf
	optimalWord = Planner::ReedsShepp::PathWords::RfLbpi2SbRbpi2Lf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.0, 3.0, -0.2 };
	Planner::Test(start, goal, { optimalWord });
	// 47: RbLfpi2SfRfpi2Lb
	optimalWord = Planner::ReedsShepp::PathWords::RbLfpi2SfRfpi2Lb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.0, 3.0, 0.2 };
	Planner::Test(start, goal, { optimalWord });

	return 0;
}
