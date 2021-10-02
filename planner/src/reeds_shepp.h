#pragma once

#include "geometry/pose.h"
#include "core/assert.h"
#include "state_space.h"

#include <array>
#include <limits>
#include <cmath>

namespace Planner {

	namespace ReedsShepp {
		/**
		 * @brief Path word for Reeds-Shepp optimal path
		 */
		enum class PathWords {
			NoPath = -1,

			// Reeds-Shepp 8.1: CSC, same turn
			LfSfLf = 0,
			LbSbLb,
			RfSfRf,
			RbSbRb,

			// Reeds-Shepp 8.2: CSC, different turn
			LfSfRf,
			LbSbRb,
			RfSfLf,
			RbSbLb,

			// Reeds-Shepp 8.3: C|C|C
			LfRbLf,
			LbRfLb,
			RfLbRf,
			RbLfRb,

			// Reeds-Shepp 8.4: C|CC
			LfRbLb,
			LbRfLf,
			RfLbRb,
			RbLfRf,

			// Reeds-Shepp 8.4: CC|C
			LfRfLb,
			LbRbLf,
			RfLfRb,
			RbLbRf,

			// Reeds-Shepp 8.7: CCu|CuC
			LfRufLubRb,
			LbRubLufRf,
			RfLufRubLb,
			RbLubRufLf,

			// Reeds-Shepp 8.8: C|CuCu|C
			LfRubLubRf,
			LbRufLufRb,
			RfLubRubLf,
			RbLufRufLb,

			// Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
			LfRbpi2SbLb,
			LbRfpi2SfLf,
			RfLbpi2SbRb,
			RbLfpi2SfRf,

			// Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
			LfRbpi2SbRb,
			LbRfpi2SfRf,
			RfLbpi2SbLb,
			RbLfpi2SfLf,

			// Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
			LfSfRfpi2Lb,
			LbSbRbpi2Lf,
			RfSfLfpi2Rb,
			RbSbLbpi2Rf,

			// Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
			LfSfLfpi2Rb,
			LbSbLbpi2Rf,
			RfSfRfpi2Lb,
			RbSbRbpi2Lf,

			// Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
			LfRbpi2SbLbpi2Rf,
			LbRfpi2SfLfpi2Rb,
			RfLbpi2SbRbpi2Lf,
			RbLfpi2SfRfpi2Lb,

			NumPathWords,
		};

		/**
		 * @brief Check if a word refers to a valid Reed-Shepp path word.
		 */
		bool IsPathWordValid(PathWords word)
		{
			int i = static_cast<int>(word);
			return i >= 0 && i < static_cast<int>(PathWords::NumPathWords);
		}

		enum class Steer {
			Left,
			Straight,
			Right
		};

		enum class Gear {
			Forward,
			Backward
		};

		/**
		 * @brief Represents a motion with a constant steering and direction over some length.
		 */
		struct Motion {
			Steer steer;
			Gear gear;
			float length = std::numeric_limits<float>::infinity();

			/**
			 * @brief Initialize an motion with an invalid motion.
			 */
			Motion() = default;

			Motion(Steer steer, Gear gear, float length) :
				steer(steer), gear(gear), length(length) { }

			/**
			 * @brief Check if an motion is valid
			 */
			bool IsValid() const
			{
				return length != std::numeric_limits<float>::infinity();
			}
		};

		/**
		 * @brief Represents a Reed-Shepp path segment.
		 * @details All paths are initialized with an infinite length. Paths
		 * with an infinite length correspond to a non-valid path.
		 */
		class PathSegment {
		public:
			/**
			 * @brief Construct a set of invalid motions.
			 */
			PathSegment()
			{
				// Initialize all motions with an infinite length
				for (auto& motion : m_motions) {
					motion.steer = Steer::Straight;
					motion.gear = Gear::Forward;
					motion.length = std::numeric_limits<float>::infinity();
				}
				m_length = 0.0f;
			}

			/**
			 * @brief Add a motion to the path.
			 */
			void AddMotion(Steer steer, Gear gear, float length)
			{
				// Find index of first invalid motion
				int index = -1;
				for (int i = 0; i < s_numMotion; i++) {
					if (!m_motions[i].IsValid()) {
						index = i;
						break;
					}
				}
				PP_ASSERT(index >= 0, "Cannot add a motion.");

				m_motions[index].steer = steer;
				m_motions[index].gear = gear;
				m_motions[index].length = length;
				m_length += std::abs(length);
			}

			/**
			 * @brief Get length.
			 */
			float GetLength() const { return m_length; }

			/**
			 * @brief Compute the cost of the path.
			 */
			float ComputeCost(float unit, float reverseCostMultiplier, float forwardCostMultiplier, float gearSwitchCost) const
			{
				if (!m_motions[0].IsValid())
					return std::numeric_limits<float>::infinity();

				if (reverseCostMultiplier == 1.0f && gearSwitchCost == 0.0f)
					return m_length * unit;

				float cost = 0;

				Gear prevGear = m_motions[0].gear;
				for (const auto& motion : m_motions) {
					if (!motion.IsValid())
						break;

					float motionCost = motion.length * unit;
					if (motion.gear == Gear::Forward)
						motionCost *= forwardCostMultiplier;
					else if (motion.gear == Gear::Backward)
						motionCost *= reverseCostMultiplier;
					if (motion.gear != prevGear)
						motionCost += gearSwitchCost;

					prevGear = motion.gear;
					cost += motionCost;
				}

				return cost;
			}

			/**
			 * @brief Inverse the direction of motion.
			 * @details After this operation the PathSegment is moved.
			 * @return The time flipped path segment.
			 */
			PathSegment&& TimeflipTransform()
			{
				for (auto& motion : m_motions)
					motion.gear = motion.gear == Gear::Backward ? Gear::Forward : Gear::Backward;
				return std::move(*this);
			}

			/**
			 * @brief Inverse the lateral direction of motion.
			 * @details After this operation the PathSegment set is moved.
			 * @return The reflected path segment.
			 */
			PathSegment&& ReflectTransform()
			{
				for (auto& motion : m_motions) {
					if (motion.steer == Steer::Left)
						motion.steer = Steer::Right;
					else if (motion.steer == Steer::Right)
						motion.steer = Steer::Left;
				}
				return std::move(*this);
			}

		private:
			static constexpr unsigned int s_numMotion = 5;
			std::array<Motion, s_numMotion> m_motions;
			float m_length = 0.0f;
		};

		class Solver {
		public:
			/**
			 * @brief Return the shortest path from the start to the goal.
			 * @param[in] start The initial pose.
			 * @param[in] goal The goal pose.
			 * @param[in] unit Normalization factor.
			 * @param[out] optimalWord The word representing the optimal path.
			 */
			static PathSegment GetShortestPath(const Pose2D<>& start, const Pose2D<>& goal, float unit, PathWords& optimalWord)
			{
				float t, u, v;
				optimalWord = GetShortestPathWord(start, goal, unit, t, u, v);

				if (!IsPathWordValid(optimalWord))
					return PathSegment();

				return GetPath(optimalWord, t, u, v);
			}

			/**
			 * @overload
			 */
			static PathSegment GetShortestPath(const Pose2D<>& start, const Pose2D<>& goal, float unit)
			{
				PathWords optimalWord = PathWords::NoPath;
				return GetShortestPath(start, goal, unit, optimalWord);
			}

		private:
			/**
			 * @brief Return the path word to generate the shortest path from 
			 * the start to the goal.
			 * @param[in] start The initial pose.
			 * @param[in] goal The goal pose.
			 * @param[in] unit Normalization factor.
			 * @param[out] t Length of the first motion primitive.
			 * @param[out] u Length of the second motion primitive.
			 * @param[out] v Length of the third motion primitive.
			 */
			static PathWords GetShortestPathWord(const Pose2D<>& start, const Pose2D<>& goal, float unit, float& t, float& u, float& v)
			{
				// Translate the goal so that the start position is at the origin with orientation 0
				Pose2D<> newGoal = goal - start;
				newGoal.x = newGoal.x / unit;
				newGoal.y = newGoal.y / unit;
				newGoal.theta = newGoal.WrapTheta();

				// clang-format off
				const std::array<Pose2D<>, 4> goals = {
					Pose2D<>( newGoal.x,  newGoal.y,  newGoal.theta),
					Pose2D<>(-newGoal.x,  newGoal.y, -newGoal.theta),
					Pose2D<>( newGoal.x, -newGoal.y, -newGoal.theta),
					Pose2D<>(-newGoal.x, -newGoal.y,  newGoal.theta),
				};
				// clang-format on

				// Iterate over pathwords to find the optimal path
				float bestPathLength = std::numeric_limits<float>::infinity();
				PathWords bestWord = PathWords::NoPath;
				std::array<float, 4> lengths;
				for (auto& l : lengths) {
					l = std::numeric_limits<float>::infinity();
				}
				std::array<float, 4> tt, tu, tv;
				for (int w = 0; w < static_cast<int>(PathWords::NumPathWords); w += 4) {

					switch (static_cast<PathWords>(w)) {
					// Reeds-Shepp 8.1: CSC, same turn
					case PathWords::LfSfLf:
						lengths[0] = LfSfLf(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfSfLf(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfSfLf(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfSfLf(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.2: CSC, different turn
					case PathWords::LfSfRf:
						lengths[0] = LfSfRf(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfSfRf(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfSfRf(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfSfRf(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.3: C|C|C
					case PathWords::LfRbLf:
						lengths[0] = LfRbLf(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfRbLf(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfRbLf(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfRbLf(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.4: C|CC
					case PathWords::LfRbLb:
						lengths[0] = LfRbLb(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfRbLb(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfRbLb(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfRbLb(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.4: CC|C
					case PathWords::LfRfLb:
						lengths[0] = LfRfLb(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfRfLb(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfRfLb(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfRfLb(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.7: CCu|CuC
					case PathWords::LfRufLubRb:
						lengths[0] = LfRufLubRb(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfRufLubRb(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfRufLubRb(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfRufLubRb(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.8: C|CuCu|C
					case PathWords::LfRubLubRf:
						lengths[0] = LfRubLubRf(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfRubLubRf(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfRubLubRf(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfRubLubRf(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
					case PathWords::LfRbpi2SbLb:
						lengths[0] = LfRbpi2SbLb(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfRbpi2SbLb(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfRbpi2SbLb(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfRbpi2SbLb(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
					case PathWords::LfRbpi2SbRb:
						lengths[0] = LfRbpi2SbRb(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfRbpi2SbRb(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfRbpi2SbRb(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfRbpi2SbRb(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
					case PathWords::LfSfRfpi2Lb:
						lengths[0] = LfSfRfpi2Lb(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfSfRfpi2Lb(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfSfRfpi2Lb(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfSfRfpi2Lb(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
					case PathWords::LfSfLfpi2Rb:
						lengths[0] = LfSfLfpi2Rb(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfSfLfpi2Rb(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfSfLfpi2Rb(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfSfLfpi2Rb(goals[3], tt[3], tu[3], tv[3]);
						break;

					// Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
					case PathWords::LfRbpi2SbLbpi2Rf:
						lengths[0] = LfRbpi2SbLbpi2Rf(goals[0], tt[0], tu[0], tv[0]);
						lengths[1] = LfRbpi2SbLbpi2Rf(goals[1], tt[1], tu[1], tv[1]);
						lengths[2] = LfRbpi2SbLbpi2Rf(goals[2], tt[2], tu[2], tv[2]);
						lengths[3] = LfRbpi2SbLbpi2Rf(goals[3], tt[3], tu[3], tv[3]);
						break;

					default:
						PP_ASSERT(false, "No equivalent Reeds-Shepp base path word.");
						break;
					}

					for (unsigned int i = 0; i < 4; i++) {
						if (lengths[i] < bestPathLength) {
							PP_ASSERT(lengths[i] > 0, "Invalid length");
							bestPathLength = lengths[i];
							bestWord = static_cast<PathWords>(w + i);
							t = tt[i];
							u = tu[i];
							v = tv[i];
						}
					}
				}
				return bestWord;
			}

			static PathSegment GetPath(PathWords word, float t, float u, float v)
			{
				// clang-format off
				switch (word) {
				// Reeds-Shepp 8.1: CSC, same turn
				case PathWords::LfSfLf: return LfSfLfPath(t, u, v);
				case PathWords::LbSbLb: return LfSfLfPath(t, u, v).TimeflipTransform();
				case PathWords::RfSfRf: return LfSfLfPath(t, u, v).ReflectTransform();
				case PathWords::RbSbRb: return LfSfLfPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.2: CSC, different turn
				case PathWords::LfSfRf: return LfSfRfPath(t, u, v);
				case PathWords::LbSbRb: return LfSfRfPath(t, u, v).TimeflipTransform();
				case PathWords::RfSfLf: return LfSfRfPath(t, u, v).ReflectTransform();
				case PathWords::RbSbLb: return LfSfRfPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.3: C|C|C
				case PathWords::LfRbLf: return LfRbLfPath(t, u, v);
				case PathWords::LbRfLb: return LfRbLfPath(t, u, v).TimeflipTransform();
				case PathWords::RfLbRf: return LfRbLfPath(t, u, v).ReflectTransform();
				case PathWords::RbLfRb: return LfRbLfPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.4: C|CC
				case PathWords::LfRbLb: return LfRbLbPath(t, u, v);
				case PathWords::LbRfLf: return LfRbLbPath(t, u, v).TimeflipTransform();
				case PathWords::RfLbRb: return LfRbLbPath(t, u, v).ReflectTransform();
				case PathWords::RbLfRf: return LfRbLbPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.4: CC|C
				case PathWords::LfRfLb: return LfRfLbPath(t, u, v);
				case PathWords::LbRbLf: return LfRfLbPath(t, u, v).TimeflipTransform();
				case PathWords::RfLfRb: return LfRfLbPath(t, u, v).ReflectTransform();
				case PathWords::RbLbRf: return LfRfLbPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.7: CCu|CuC 
				case PathWords::LfRufLubRb: return LfRufLubRbPath(t, u, v);
				case PathWords::LbRubLufRf: return LfRufLubRbPath(t, u, v).TimeflipTransform();
				case PathWords::RfLufRubLb: return LfRufLubRbPath(t, u, v).ReflectTransform();
				case PathWords::RbLubRufLf: return LfRufLubRbPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.8: C|CuCu|C
				case PathWords::LfRubLubRf: return LfRubLubRfPath(t, u, v);
				case PathWords::LbRufLufRb: return LfRubLubRfPath(t, u, v).TimeflipTransform();
				case PathWords::RfLubRubLf: return LfRubLubRfPath(t, u, v).ReflectTransform();
				case PathWords::RbLufRufLb: return LfRubLubRfPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
				case PathWords::LfRbpi2SbLb: return LfRbpi2SbLbPath(t, u, v);
				case PathWords::LbRfpi2SfLf: return LfRbpi2SbLbPath(t, u, v).TimeflipTransform();
				case PathWords::RfLbpi2SbRb: return LfRbpi2SbLbPath(t, u, v).ReflectTransform();
				case PathWords::RbLfpi2SfRf: return LfRbpi2SbLbPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
				case PathWords::LfRbpi2SbRb: return LfRbpi2SbRbPath(t, u, v);
				case PathWords::LbRfpi2SfRf: return LfRbpi2SbRbPath(t, u, v).TimeflipTransform();
				case PathWords::RfLbpi2SbLb: return LfRbpi2SbRbPath(t, u, v).ReflectTransform();
				case PathWords::RbLfpi2SfLf: return LfRbpi2SbRbPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
				case PathWords::LfSfRfpi2Lb: return LfSfRfpi2LbPath(t, u, v);
				case PathWords::LbSbRbpi2Lf: return LfSfRfpi2LbPath(t, u, v).TimeflipTransform();
				case PathWords::RfSfLfpi2Rb: return LfSfRfpi2LbPath(t, u, v).ReflectTransform();
				case PathWords::RbSbLbpi2Rf: return LfSfRfpi2LbPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
				case PathWords::LfSfLfpi2Rb: return LfSfLfpi2RbPath(t, u, v);
				case PathWords::LbSbLbpi2Rf: return LfSfLfpi2RbPath(t, u, v).TimeflipTransform();
				case PathWords::RfSfRfpi2Lb: return LfSfLfpi2RbPath(t, u, v).ReflectTransform();
				case PathWords::RbSbRbpi2Lf: return LfSfLfpi2RbPath(t, u, v).TimeflipTransform().ReflectTransform();
				
				// Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
				case PathWords::LfRbpi2SbLbpi2Rf: return LfRbpi2SbLbpi2RfPath(t, u, v);
				case PathWords::LbRfpi2SfLfpi2Rb: return LfRbpi2SbLbpi2RfPath(t, u, v).TimeflipTransform();
				case PathWords::RfLbpi2SbRbpi2Lf: return LfRbpi2SbLbpi2RfPath(t, u, v).ReflectTransform();
				case PathWords::RbLfpi2SfRfpi2Lb: return LfRbpi2SbLbpi2RfPath(t, u, v).TimeflipTransform().ReflectTransform();

				default:
					PP_ASSERT(false, "Word is not a valid Reeds-Shepp path word.");
					return PathSegment();
				}
				// clang-format on
			}

			static float Modulo(float in, float mod)
			{
				// Return the modulo of x by y
				float out = fmod(in, mod);
				if (out < 0)
					out += mod;
				return out;
			}

			static bool IsAngleInvalid(float theta)
			{
				return theta < 0 || theta > M_PI;
			}

			static float WrapAngle(float theta)
			{
				return Modulo(theta + M_PI, 2 * M_PI) - M_PI;
			}

			static float LfSfLf(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.1
				t = 0;
				u = 0;
				v = 0;

				float x = goal.x - sin(goal.theta);
				float y = goal.y - 1 + cos(goal.theta);

				u = sqrt(x * x + y * y);
				t = atan2(y, x);
				v = WrapAngle(goal.theta - t);

				if (IsAngleInvalid(t) || IsAngleInvalid(v))
					return std::numeric_limits<float>::infinity();

				return t + u + v;
			}

			static float LfSfRf(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.2
				t = 0;
				u = 0;
				v = 0;

				float x = goal.x + sin(goal.theta);
				float y = goal.y - 1 - cos(goal.theta);

				float u1squared = x * x + y * y;
				float t1 = std::atan2(y, x);

				if (u1squared < 4)
					std::numeric_limits<float>::infinity();

				u = sqrt(u1squared - 4);
				float phi = atan2(2, u);
				t = WrapAngle(t1 + phi);
				v = WrapAngle(t - goal.theta);

				if (IsAngleInvalid(t) || IsAngleInvalid(v))
					return std::numeric_limits<float>::infinity();

				return t + u + v;
			}

			static float LfRbLf(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.3
				// Uses a modified formula adapted from the c_c_c function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x - sin(goal.theta);
				float eta = goal.y - 1 + cos(goal.theta);

				float u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 4)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);
				float alpha = acos(u1 / 4.0f);
				t = Modulo(M_PI_2 + alpha + phi, 2 * M_PI);
				u = Modulo(M_PI - 2 * alpha, 2 * M_PI);
				v = Modulo(goal.theta - t - u, 2 * M_PI);

				if (IsAngleInvalid(t) || IsAngleInvalid(u) || IsAngleInvalid(v))
					return std::numeric_limits<float>::infinity();

				return t + u + v;
			}

			static float LfRbLb(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.4
				// Uses a modified formula adapted from the c_cc function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x - sin(goal.theta);
				float eta = goal.y - 1 + cos(goal.theta);

				float u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 4)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);
				float alpha = acos(u1 / 4.0f);
				t = Modulo(M_PI_2 + alpha + phi, 2 * M_PI);
				u = Modulo(M_PI - 2 * alpha, 2 * M_PI);
				v = Modulo(t + u - goal.theta, 2 * M_PI);

				return t + u + v;
			}

			static float LfRfLb(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.4
				// Uses a modified formula adapted from the cc_c function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x - sin(goal.theta);
				float eta = goal.y - 1 + cos(goal.theta);

				float u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 4)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);
				u = acos((8 - u1 * u1) / 8.0f);
				float va = sin(u);
				float alpha = asin(2 * va / u1);
				t = Modulo(M_PI_2 - alpha + phi, 2 * M_PI);
				v = Modulo(t - u - goal.theta, 2 * M_PI);

				return t + u + v;
			}

			static float LfRufLubRb(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.7
				// Uses a modified formula adapted from the ccu_cuc function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x + sin(goal.theta);
				float eta = goal.y - 1 - cos(goal.theta);

				float u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 4)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);

				if (u1 > 2) {
					float alpha = acos(u1 / 4 - 0.5);
					t = Modulo(M_PI_2 + phi - alpha, 2 * M_PI);
					u = Modulo(M_PI - alpha, 2 * M_PI);
					v = Modulo(goal.theta - t + 2 * u, 2 * M_PI);
				} else {
					float alpha = acos(u1 / 4 + 0.5);
					t = Modulo(M_PI_2 + phi + alpha, 2 * M_PI);
					u = Modulo(alpha, 2 * M_PI);
					v = Modulo(goal.theta - t + 2 * u, 2 * M_PI);
				}

				return t + u + u + v;
			}

			static float LfRubLubRf(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.8
				// Uses a modified formula adapted from the c_cucu_c function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x + sin(goal.theta);
				float eta = goal.y - 1 - cos(goal.theta);

				float u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 6)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);
				float va1 = 1.25f - u1 * u1 / 16;
				if (va1 < 0 || va1 > 1)
					return std::numeric_limits<float>::infinity();

				u = acos(va1);
				float va2 = sin(u);
				float alpha = asin(2 * va2 / u1);
				t = Modulo(M_PI_2 + phi + alpha, 2 * M_PI);
				v = Modulo(t - goal.theta, 2 * M_PI);

				return t + u + u + v;
			}

			static float LfRbpi2SbLb(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.9
				// Uses a modified formula adapted from the c_c2sca function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x - sin(goal.theta);
				float eta = goal.y - 1 + cos(goal.theta);

				float u1squared = xi * xi + eta * eta;
				if (u1squared < 4)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);

				u = sqrt(u1squared - 4) - 2;
				if (u < 0)
					return std::numeric_limits<float>::infinity();

				float alpha = atan2(2, u + 2);
				t = Modulo(M_PI_2 + phi + alpha, 2 * M_PI);
				v = Modulo(t + M_PI_2 - goal.theta, 2 * M_PI);

				return t + M_PI_2 + u + v;
			}

			static float LfRbpi2SbRb(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.10
				// Uses a modified formula adapted from the c_c2scb function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x + sin(goal.theta);
				float eta = goal.y - 1 - cos(goal.theta);

				float u1 = sqrt(xi * xi + eta * eta);
				if (u1 < 2)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);

				t = Modulo(M_PI_2 + phi, 2 * M_PI);
				u = u1 - 2;
				v = Modulo(goal.theta - t - M_PI_2, 2 * M_PI);

				return t + M_PI_2 + u + v;
			}

			static float LfSfRfpi2Lb(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.9 (reversed)
				// Uses a modified formula adapted from the csc2_ca function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x - sin(goal.theta);
				float eta = goal.y - 1 + cos(goal.theta);

				float u1squared = xi * xi + eta * eta;
				if (u1squared < 4)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);

				u = sqrt(u1squared - 4) - 2;
				if (u < 0)
					return std::numeric_limits<float>::infinity();

				float alpha = atan2(u + 2, 2);
				t = Modulo(M_PI_2 + phi - alpha, 2 * M_PI);
				v = Modulo(t - M_PI_2 - goal.theta, 2 * M_PI);

				return t + u + M_PI_2 + v;
			}

			static float LfSfLfpi2Rb(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.10 (reversed)
				// Uses a modified formula adapted from the csc2_cb function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x + sin(goal.theta);
				float eta = goal.y - 1 - cos(goal.theta);

				float u1 = sqrt(xi * xi + eta * eta);
				if (u1 < 2)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);

				t = Modulo(phi, 2 * M_PI);
				u = u1 - 2;
				v = Modulo(-t - M_PI_2 + goal.theta, 2 * M_PI);

				return t + u + M_PI_2 + v;
			}

			static float LfRbpi2SbLbpi2Rf(const Pose2D<>& goal, float& t, float& u, float& v)
			{
				// Reeds-Shepp 8.11
				// Uses a modified formula adapted from the c_c2sc2_c function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				t = 0;
				u = 0;
				v = 0;

				float xi = goal.x + sin(goal.theta);
				float eta = goal.y - 1 - cos(goal.theta);

				float u1squared = xi * xi + eta * eta;
				if (u1squared < 16)
					return std::numeric_limits<float>::infinity();

				float phi = atan2(eta, xi);

				u = sqrt(u1squared - 4) - 4;
				if (u < 0)
					return std::numeric_limits<float>::infinity();

				float alpha = atan2(2, u + 4);
				t = Modulo(M_PI_2 + phi + alpha, 2 * M_PI);
				v = Modulo(t - goal.theta, 2 * M_PI);

				return t + u + v + M_PI;
			}

			static PathSegment LfSfLfPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Straight, Gear::Forward, u);
				motions.AddMotion(Steer::Left, Gear::Forward, v);
				return motions;
			}

			static PathSegment LfSfRfPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Straight, Gear::Forward, u);
				motions.AddMotion(Steer::Right, Gear::Forward, v);
				return motions;
			}

			static PathSegment LfRbLfPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Right, Gear::Backward, u);
				motions.AddMotion(Steer::Left, Gear::Forward, v);
				return motions;
			}

			static PathSegment LfRbLbPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Right, Gear::Backward, u);
				motions.AddMotion(Steer::Left, Gear::Backward, v);
				return motions;
			}

			static PathSegment LfRfLbPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Right, Gear::Forward, u);
				motions.AddMotion(Steer::Left, Gear::Backward, v);
				return motions;
			}

			static PathSegment LfRufLubRbPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Right, Gear::Forward, u);
				motions.AddMotion(Steer::Left, Gear::Backward, u);
				motions.AddMotion(Steer::Right, Gear::Backward, v);
				return motions;
			}

			static PathSegment LfRubLubRfPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Right, Gear::Backward, u);
				motions.AddMotion(Steer::Left, Gear::Backward, u);
				motions.AddMotion(Steer::Right, Gear::Forward, v);
				return motions;
			}

			static PathSegment LfRbpi2SbLbPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Right, Gear::Backward, M_PI_2);
				motions.AddMotion(Steer::Straight, Gear::Backward, u);
				motions.AddMotion(Steer::Left, Gear::Backward, v);
				return motions;
			}

			static PathSegment LfRbpi2SbRbPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Right, Gear::Backward, M_PI_2);
				motions.AddMotion(Steer::Straight, Gear::Backward, u);
				motions.AddMotion(Steer::Right, Gear::Backward, v);
				return motions;
			}

			static PathSegment LfSfRfpi2LbPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Straight, Gear::Forward, u);
				motions.AddMotion(Steer::Right, Gear::Forward, M_PI_2);
				motions.AddMotion(Steer::Left, Gear::Backward, v);
				return motions;
			}

			static PathSegment LfSfLfpi2RbPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Straight, Gear::Forward, u);
				motions.AddMotion(Steer::Left, Gear::Forward, M_PI_2);
				motions.AddMotion(Steer::Right, Gear::Backward, v);
				return motions;
			}

			static PathSegment LfRbpi2SbLbpi2RfPath(float t, float u, float v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Gear::Forward, t);
				motions.AddMotion(Steer::Right, Gear::Backward, M_PI_2);
				motions.AddMotion(Steer::Straight, Gear::Backward, u);
				motions.AddMotion(Steer::Left, Gear::Backward, M_PI_2);
				motions.AddMotion(Steer::Right, Gear::Forward, v);
				return motions;
			}
		};
	}

	class StateSpaceReedsShepp : public StateSpace<Pose2D<>> {
// 		using namespace Planner::ReedsShepp;

	public:
		StateSpaceReedsShepp() = default;
		~StateSpaceReedsShepp() = default;

		virtual double ComputeDistance(const Pose2D<>& from, const Pose2D<>& to) override
		{
			auto path = ReedsShepp::Solver::GetShortestPath(from, to, m_minTurningRadius);
			return path.GetLength();
		}

// 		double ComputeCost(const Pose2D<>& from, const Pose2D<>& to) const
// 		{
// 			// TODO
// 		}

		void SetMinTurningRadius(double turningRadius) { m_minTurningRadius = turningRadius; }
		double GetMinTurningRadius() const { return m_minTurningRadius; }

		void SetReverseCostMultiplier(double cost) { m_reverseCostMultiplier = cost; }
		double GetReverseCostMultiplier() const { return m_reverseCostMultiplier; }

		void SetForwardCostMultiplier(double cost) { m_forwardCostMultiplier = cost; }
		double GetForwardCostMultiplier() const { return m_forwardCostMultiplier; }

		void SetGearSwitchCost(double cost) { m_gearSwitchCost = cost; }
		double GetGearSwitchCost() const { return m_gearSwitchCost; }

	private:
		double m_minTurningRadius = 1;
		double m_reverseCostMultiplier = 1, m_forwardCostMultiplier = 1;
		double m_gearSwitchCost;
	};
}
