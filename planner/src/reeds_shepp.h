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
			RbLfpi2SfRfpi2Lb,
		};

		typedef int ActionSet; // FIXME
		
		class Solver {
		public:
			/**
			 * @brief
			 */
			static ActionSet Solve(const Pose& start, const Pose& goal, float unit)// FIXME TODO need to define pose: create a struct which defines a Eigen3vec and modify HybridA* (also remove HashPose and construct to be always within -pi and pi)?
			{
				
			}
			
			/**
			 * @brief
			 * @param[in] goal
			 * @param[in] word
			 * @param[out] t
			 * @param[out] u
			 * @param[out] v
			 */
			static float CalculatePathLength(const Pose& goal, PathWords word, float& t, float& u, float& v)
			{
				switch (word) {
				// Reeds-Shepp 8.1: CSC, same turn
				case PathWords::LfSfLf: return;
				case PathWords::LbSbLb: return;
				case PathWords::RfSfRf: return;
				case PathWords::RbSbRb: return;
				
				// Reeds-Shepp 8.2: CSC, different turn
				case PathWords::LfSfRf: return;
				case PathWords::LbSbRb: return;
				case PathWords::RfSfLf: return;
				case PathWords::RbSbLb: return;
				
				// Reeds-Shepp 8.3: C|C|C
				case PathWords::LfRbLf: return;
				case PathWords::LbRfLb: return;
				case PathWords::RfLbRf: return;
				case PathWords::RbLfRb: return;
				
				// Reeds-Shepp 8.4: C|CC
				case PathWords::LfRbLb: return;
				case PathWords::LbRfLf: return;
				case PathWords::RfLbRb: return;
				case PathWords::RbLfRf: return;
				
				// Reeds-Shepp 8.4: CC|C
				case PathWords::LfRfLb: return;
				case PathWords::LbRbLf: return;
				case PathWords::RfLfRb: return;
				case PathWords::RbLbRf: return;
				
				// Reeds-Shepp 8.7: CCu|CuC 
				case PathWords::LfRufLubRb: return;
				case PathWords::LbRubLufRf: return;
				case PathWords::RfLufRubLb: return;
				case PathWords::RbLubRufLf: return;
				
				// Reeds-Shepp 8.8: C|CuCu|C
				case PathWords::LfRubLubRf: return;
				case PathWords::LbRufLufRb: return;
				case PathWords::RfLubRubLf: return;
				case PathWords::RbLufRufLb: return;
				
				// Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
				case PathWords::LfRbpi2SbLb: return;
				case PathWords::LbRfpi2SfLf: return;
				case PathWords::RfLbpi2SbRb: return;
				case PathWords::RbLfpi2SfRf: return;
				
				// Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
				case PathWords::LfRbpi2SbRb: return;
				case PathWords::LbRfpi2SfRf: return;
				case PathWords::RfLbpi2SbLb: return;
				case PathWords::RbLfpi2SfLf: return;
				
				// Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
				case PathWords::LfSfRfpi2Lb: return;
				case PathWords::LbSbRbpi2Lf: return;
				case PathWords::RfSfLfpi2Rb: return;
				case PathWords::RbSbLbpi2Rf: return;
				
				// Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
				case PathWords::LfSfLfpi2Rb: return;
				case PathWords::LbSbLbpi2Rf: return;
				case PathWords::RfSfRfpi2Lb: return;
				case PathWords::RbSbRbpi2Lf: return;
				
				// Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
				case PathWords::LfRbpi2SbLbpi2Rf: return;
				case PathWords::LbRfpi2SfLfpi2Rb: return;
				case PathWords::RfLbpi2SbRbpi2Lf: return;
				case PathWords::RbLfpi2SfRfpi2Lb: return;
				}
			}
			
			static ActionSet GetPath(PathWords word, float t, float u, float v)
			{
				switch (word) {
				// Reeds-Shepp 8.1: CSC, same turn
				case PathWords::LfSfLf: return;
				case PathWords::LbSbLb: return;
				case PathWords::RfSfRf: return;
				case PathWords::RbSbRb: return;
				
				// Reeds-Shepp 8.2: CSC, different turn
				case PathWords::LfSfRf: return;
				case PathWords::LbSbRb: return;
				case PathWords::RfSfLf: return;
				case PathWords::RbSbLb: return;
				
				// Reeds-Shepp 8.3: C|C|C
				case PathWords::LfRbLf: return;
				case PathWords::LbRfLb: return;
				case PathWords::RfLbRf: return;
				case PathWords::RbLfRb: return;
				
				// Reeds-Shepp 8.4: C|CC
				case PathWords::LfRbLb: return;
				case PathWords::LbRfLf: return;
				case PathWords::RfLbRb: return;
				case PathWords::RbLfRf: return;
				
				// Reeds-Shepp 8.4: CC|C
				case PathWords::LfRfLb: return;
				case PathWords::LbRbLf: return;
				case PathWords::RfLfRb: return;
				case PathWords::RbLbRf: return;
				
				// Reeds-Shepp 8.7: CCu|CuC 
				case PathWords::LfRufLubRb: return;
				case PathWords::LbRubLufRf: return;
				case PathWords::RfLufRubLb: return;
				case PathWords::RbLubRufLf: return;
				
				// Reeds-Shepp 8.8: C|CuCu|C
				case PathWords::LfRubLubRf: return;
				case PathWords::LbRufLufRb: return;
				case PathWords::RfLubRubLf: return;
				case PathWords::RbLufRufLb: return;
				
				// Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
				case PathWords::LfRbpi2SbLb: return;
				case PathWords::LbRfpi2SfLf: return;
				case PathWords::RfLbpi2SbRb: return;
				case PathWords::RbLfpi2SfRf: return;
				
				// Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
				case PathWords::LfRbpi2SbRb: return;
				case PathWords::LbRfpi2SfRf: return;
				case PathWords::RfLbpi2SbLb: return;
				case PathWords::RbLfpi2SfLf: return;
				
				// Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
				case PathWords::LfSfRfpi2Lb: return;
				case PathWords::LbSbRbpi2Lf: return;
				case PathWords::RfSfLfpi2Rb: return;
				case PathWords::RbSbLbpi2Rf: return;
				
				// Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
				case PathWords::LfSfLfpi2Rb: return;
				case PathWords::LbSbLbpi2Rf: return;
				case PathWords::RfSfRfpi2Lb: return;
				case PathWords::RbSbRbpi2Lf: return;
				
				// Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
				case PathWords::LfRbpi2SbLbpi2Rf: return;
				case PathWords::LbRfpi2SfLfpi2Rb: return;
				case PathWords::RfLbpi2SbRbpi2Lf: return;
				case PathWords::RbLfpi2SfRfpi2Lb: return;
				}
			}
			
		private:
			static ActionSet TimeflipTransform(const ActionSet& actions)
			{
				
			}
			
			static ActionSet ReflectTransform(const ActionSet& actions)
			{
				
			}
			
			static float LfSfLf(const Pose& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.1
				t = 0; u = 0; v = 0;

				float x = goal.x - sin(goal.orientation);
				float y = goal.y - 1 + cos(goal.orientation);

				u = sqrt(x * x + y * y);
				t = atan2(y, x);
				v = MathHelper.WrapAngle(goal.Orientation - t);

				if (isInvalidAngle(t) || isInvalidAngle(v))
					return float.PositiveInfinity;

				return t + u + v;
			}
			
		private:
			static const int s_numPathWords = 48;
		};

	};
}
