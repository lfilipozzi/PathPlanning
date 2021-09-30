#include "core/log.h"
#include "reeds_shepp.h"

namespace Planner {
	void Test(const Pose2D<>& start, const Pose2D<>& goal, ReedsSheep::PathWords optimalWord)
	{
		ReedsSheep::PathWords word = ReedsSheep::PathWords::NoPath;
		float unit = 1.0f;
		ReedsSheep::Solver::GetShortestPath(start, goal, unit, word);

		assert(word == optimalWord);
	}
}

int main()
{
	PP_INIT_LOGGER;

	Planner::ReedsSheep::PathWords optimalWord;
	Planner::Pose2D<> start;
	Planner::Pose2D<> goal;

	// CSC, same turn
	// 0: LfSfLf
	optimalWord = Planner::ReedsSheep::PathWords::LfSfLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 10.0, 10.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 1: LbSbLb
	optimalWord = Planner::ReedsSheep::PathWords::LbSbLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -10.0, 10.0, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 2: RfSfRf
	optimalWord = Planner::ReedsSheep::PathWords::RfSfRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 10.0, -10.0, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 3: RbSbRb
	optimalWord = Planner::ReedsSheep::PathWords::RbSbRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -10.0, -10.0, 1.6 };
	Planner::Test(start, goal, optimalWord);

	// CSC, different turn
	// 4: LfSfRf
	optimalWord = Planner::ReedsSheep::PathWords::LfSfRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, 2.0, 0.1 };
	Planner::Test(start, goal, optimalWord);
	// 5: LbSbRb
	optimalWord = Planner::ReedsSheep::PathWords::LbSbRb;
	start = { 0.0, 0.0, 1.6 };
	goal = { 2.0, 2.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 6: RfSfLf
	optimalWord = Planner::ReedsSheep::PathWords::RfSfLf;
	start = { 0.0, 0.0, 1.6 };
	goal = { -2.0, -2.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 7: RbSbLb
	optimalWord = Planner::ReedsSheep::PathWords::RbSbLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, -2.0, 0.1 };
	Planner::Test(start, goal, optimalWord);

	// C|C|C
	// 8: LfRbLf
	optimalWord = Planner::ReedsSheep::PathWords::LfRbLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.1, 0.1, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 9: LbRfLb
	optimalWord = Planner::ReedsSheep::PathWords::LbRfLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -0.1, 0.1, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 10: RfLbRf
	optimalWord = Planner::ReedsSheep::PathWords::RfLbRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -0.1, 0, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 11: RbLfRb
	optimalWord = Planner::ReedsSheep::PathWords::RbLfRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 0.1, 0, 1.6 };
	Planner::Test(start, goal, optimalWord);

	// C|CC
	// 12: LfRbLb
	optimalWord = Planner::ReedsSheep::PathWords::LfRbLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, -2.0, 0.0 };
	Planner::Test(start, goal, optimalWord);
	// 13: LbRfLf
	optimalWord = Planner::ReedsSheep::PathWords::LbRfLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, -2.0, 0.0 };
	Planner::Test(start, goal, optimalWord);
	// 14: RfLbRb
	optimalWord = Planner::ReedsSheep::PathWords::RfLbRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { -2.0, 2.0, 0.0 };
	Planner::Test(start, goal, optimalWord);
	// 15: RbLfRf
	optimalWord = Planner::ReedsSheep::PathWords::RbLfRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { 2.0, 2.0, 0.0 };
	Planner::Test(start, goal, optimalWord);

	// CC|C
	// 16: LfRfLb
	optimalWord = Planner::ReedsSheep::PathWords::LfRfLb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, 0.0, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 17: LbRbLf
	optimalWord = Planner::ReedsSheep::PathWords::LbRbLf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, 0.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 18: RfLfRb
	optimalWord = Planner::ReedsSheep::PathWords::RfLfRb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, 0.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 19: RbLbRf
	optimalWord = Planner::ReedsSheep::PathWords::RbLbRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, 0.0, -1.6 };
	Planner::Test(start, goal, optimalWord);

	// CCu|CuC
	// 20: LfRufLubRb
	optimalWord = Planner::ReedsSheep::PathWords::LfRufLubRb;
	start = { 0.0, 0.0, 0.3 };
	goal = { 0.1, 0.1, -0.3 };
	Planner::Test(start, goal, optimalWord);
	// 21: LbRubLufRf
	optimalWord = Planner::ReedsSheep::PathWords::LbRubLufRf;
	start = { 0.0, 0.0, -0.3 };
	goal = { -0.1, 0.1, 0.3 };
	Planner::Test(start, goal, optimalWord);
	// 22: RfLufRubLb
	optimalWord = Planner::ReedsSheep::PathWords::RfLufRubLb;
	start = { 0.0, 0.0, -0.3 };
	goal = { 0.1, -0.1, 0.3 };
	Planner::Test(start, goal, optimalWord);
	// 23: RbLubRufLf
	optimalWord = Planner::ReedsSheep::PathWords::RbLubRufLf;
	start = { 0.0, 0.0, 0.3 };
	goal = { -0.1, -0.1, -0.3 };
	Planner::Test(start, goal, optimalWord);

	// C|CuCu|C
	// 24: LfRubLubRf
	optimalWord = Planner::ReedsSheep::PathWords::LfRubLubRf;
	start = { 1.0, 1.0, -1.6 };
	goal = { 3.0, 1.0, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 25: LbRufLufRb
	optimalWord = Planner::ReedsSheep::PathWords::LbRufLufRb;
	start = { -1.0, -1.0, 1.6 };
	goal = { -3.0, -1.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 26: RfLubRubLf
	optimalWord = Planner::ReedsSheep::PathWords::RfLubRubLf;
	start = { 1.0, 1.0, 1.6 };
	goal = { 3.0, 1.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 27: RbLufRufLb
	optimalWord = Planner::ReedsSheep::PathWords::RbLufRufLb;
	start = { -1.0, -1.0, -1.6 };
	goal = { -3.0, -1.0, -1.6 };
	Planner::Test(start, goal, optimalWord);

	// C|C(pi/2)SC, same turn
	// 28: LfRbpi2SbLb
	optimalWord = Planner::ReedsSheep::PathWords::LfRbpi2SbLb;
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, 3.1 };
	Planner::Test(start, goal, optimalWord);
	// 29: LbRfpi2SfLf
	optimalWord = Planner::ReedsSheep::PathWords::LbRfpi2SfLf;
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, 0 };
	Planner::Test(start, goal, optimalWord);
	// 30: RfLbpi2SbRb
	optimalWord = Planner::ReedsSheep::PathWords::RfLbpi2SbRb;
	start = { 1.0, 1.0, -1.6 };
	goal = { -3.0, 1.0, -3.1 };
	Planner::Test(start, goal, optimalWord);
	// 31: RbLfpi2SfRf
	optimalWord = Planner::ReedsSheep::PathWords::RbLfpi2SfRf;
	start = { 1.0, 1.0, -1.6 };
	goal = { -3.0, 1.0, 0 };
	Planner::Test(start, goal, optimalWord);

	// C|C(pi/2)SC, different turn
	// 32: LfRbpi2SbRb
	optimalWord = Planner::ReedsSheep::PathWords::LfRbpi2SbRb;
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 33: LbRfpi2SfRf
	optimalWord = Planner::ReedsSheep::PathWords::LbRfpi2SfRf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, -2.0, -3.1 };
	Planner::Test(start, goal, optimalWord);
	// 34: RfLbpi2SbLb
	optimalWord = Planner::ReedsSheep::PathWords::RfLbpi2SbLb;
	start = { 0.0, 0.0, 3.1 };
	goal = { -2.0, -2.0, 0.0 };
	Planner::Test(start, goal, optimalWord);
	// 35: RbLfpi2SfLf
	optimalWord = Planner::ReedsSheep::PathWords::RbLfpi2SfLf;
	start = { 1.0, 1.0, 1.6 };
	goal = { 3.0, 1.0, -1.6 };
	Planner::Test(start, goal, optimalWord);

	// CSC(pi/2)|C, same turn
	// 36: LfSfRfpi2Lb
	optimalWord = Planner::ReedsSheep::PathWords::LfSfRfpi2Lb;
	start = { 0.0, 0.0, -1.6 };
	goal = { -2.0, 2.0, -3.1 };
	Planner::Test(start, goal, optimalWord);
	// 37: LbSbRbpi2Lf
	optimalWord = Planner::ReedsSheep::PathWords::LbSbRbpi2Lf;
	start = { 0.0, 0.0, -3.1 };
	goal = { 2.0, -2.0, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 38: RfSfLfpi2Rb
	optimalWord = Planner::ReedsSheep::PathWords::RfSfLfpi2Rb;
	start = { 0.0, 0.0, -3.1 };
	goal = { -2.0, 2.0, -1.6 };
	Planner::Test(start, goal, optimalWord);
	// 39: RbSbLbpi2Rf
	optimalWord = Planner::ReedsSheep::PathWords::RbSbLbpi2Rf;
	start = { 0.0, 0.0, -1.6 };
	goal = { 2.0, -2.0, -3.1 };
	Planner::Test(start, goal, optimalWord);

	// CSC(pi/2)|C, different turn
	// 40: LfSfLfpi2Rb
	optimalWord = Planner::ReedsSheep::PathWords::LfSfLfpi2Rb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, 2.0, 3.1 };
	Planner::Test(start, goal, optimalWord);
	// 41: LbSbLbpi2Rf
	optimalWord = Planner::ReedsSheep::PathWords::LbSbLbpi2Rf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, 2.0, -3.1 };
	Planner::Test(start, goal, optimalWord);
	// 42: RfSfRfpi2Lb
	optimalWord = Planner::ReedsSheep::PathWords::RfSfRfpi2Lb;
	start = { 0.0, 0.0, 0.0 };
	goal = { 1.0, -2.0, -3.1 };
	Planner::Test(start, goal, optimalWord);
	// 43: RbSbRbpi2Lf
	optimalWord = Planner::ReedsSheep::PathWords::RbSbRbpi2Lf;
	start = { 0.0, 0.0, 0.0 };
	goal = { -1.0, -2.0, 3.1 };
	Planner::Test(start, goal, optimalWord);

	// C|C(pi/2)SC(pi/2)|C
	// 44: LfRbpi2SbLbpi2Rf
	optimalWord = Planner::ReedsSheep::PathWords::LfRbpi2SbLbpi2Rf;
	start = { 1.0, -1.0, 1.6 };
	goal = { -3.0, 1.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 45: LbRfpi2SfLfpi2Rb
	optimalWord = Planner::ReedsSheep::PathWords::LbRfpi2SfLfpi2Rb;
	start = { 1.0, 1.0, 1.6 };
	goal = { -3.0, 1.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 46: RfLbpi2SbRbpi2Lf
	optimalWord = Planner::ReedsSheep::PathWords::RfLbpi2SbRbpi2Lf;
	start = { -1.0, 1.0, 1.6 };
	goal = { 3.0, 1.0, 1.6 };
	Planner::Test(start, goal, optimalWord);
	// 47: RbLfpi2SfRfpi2Lb
	optimalWord = Planner::ReedsSheep::PathWords::RbLfpi2SfRfpi2Lb;
	start = { 1.0, 1.0, 1.6 };
	goal = { 3.0, -1.0, 1.6 };
	Planner::Test(start, goal, optimalWord);

	return 0;
}
