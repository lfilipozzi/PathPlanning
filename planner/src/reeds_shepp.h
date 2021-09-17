#pragma once

namespace Planner {

	namespace ReedsSheep {
		enum class PathWords {
			LfSfLf, // Reeds-Shepp 8.1: CSC, same turn
			LbSbLb,
			RfSfRf,
			RbSbRb,

			LfSfRf, // Reeds-Shepp 8.2: CSC, different turn
			LbSbRb,
			RfSfLf,
			RbSbLb,

			LfRbLf, // Reeds-Shepp 8.3: C|C|C
			LbRfLb,
			RfLbRf,
			RbLfRb,

			LfRbLb, // Reeds-Shepp 8.4: C|CC
			LbRfLf,
			RfLbRb,
			RbLfRf,

			LfRfLb, // Reeds-Shepp 8.4: CC|C
			LbRbLf,
			RfLfRb,
			RbLbRf,

			LfRufLubRb, // Reeds-Shepp 8.7: CCu|CuC
			LbRubLufRf,
			RfLufRubLb,
			RbLubRufLf,

			LfRubLubRf, // Reeds-Shepp 8.8: C|CuCu|C
			LbRufLufRb,
			RfLubRubLf,
			RbLufRufLb,

			LfRbpi2SbLb, // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
			LbRfpi2SfLf,
			RfLbpi2SbRb,
			RbLfpi2SfRf,

			LfRbpi2SbRb, // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
			LbRfpi2SfRf,
			RfLbpi2SbLb,
			RbLfpi2SfLf,

			LfSfRfpi2Lb, // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
			LbSbRbpi2Lf,
			RfSfLfpi2Rb,
			RbSbLbpi2Rf,

			LfSfLfpi2Rb, // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
			LbSbLbpi2Rf,
			RfSfRfpi2Lb,
			RbSbRbpi2Lf,

			LfRbpi2SbLbpi2Rf, // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
			LbRfpi2SfLfpi2Rb,
			RfLbpi2SbRbpi2Lf,
			RbLfpi2SfRfpi2Lb
		};

		class Solver {
		};

	};
}
