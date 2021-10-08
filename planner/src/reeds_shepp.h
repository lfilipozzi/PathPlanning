#pragma once

#include "maths.h"
#include "geometry/pose.h"
#include "core/assert.h"
#include "path.h"
#include "state_space.h"

#include <array>
#include <limits>
#include <cmath>

namespace Planner {

	class PathReedsShepp;

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

		/**
		 * @brief Represents a motion with a constant steering and direction over some length.
		 */
		struct Motion {
			Steer steer;
			Direction direction = Direction::NoMotion;
			double length = std::numeric_limits<double>::infinity();

			/**
			 * @brief Initialize an motion with an invalid motion.
			 */
			Motion() = default;

			Motion(Steer steer, Direction direction, double length) :
				steer(steer), direction(direction), length(length) { }

			/**
			 * @brief Check if an motion is valid
			 */
			bool IsValid() const
			{
				return length != std::numeric_limits<double>::infinity() && direction != Direction::NoMotion;
			}
		};

		/**
		 * @brief Represents a Reed-Shepp path segment.
		 * @details All paths are initialized with an infinite length. Paths
		 * with an infinite length correspond to a non-valid path. Lengths of each
		 * motion are normalized.
		 */
		class PathSegment {
			friend class ::Planner::PathReedsShepp;

		public:
			static constexpr unsigned int numMotion = 5;

		public:
			/**
			 * @brief Construct a set of invalid motions.
			 */
			PathSegment()
			{
				// Initialize all motions with an infinite length
				for (auto& motion : m_motions) {
					motion = Motion();
				}
				m_length = 0.0;
			}

			/**
			 * @brief Add a motion to the path.
			 */
			void AddMotion(Steer steer, Direction direction, double length)
			{
				// Find index of first invalid motion
				int index = -1;
				for (int i = 0; i < numMotion; i++) {
					if (!m_motions[i].IsValid()) {
						index = i;
						break;
					}
				}
				PP_ASSERT(index >= 0, "Cannot add a motion.");

				m_motions[index].steer = steer;
				m_motions[index].direction = direction;
				m_motions[index].length = length;
				m_length += std::abs(length);
			}

			/**
			 * @brief Get number of valid motion in the path.
			 */
			int GetNumMotions() const
			{
				int index = 0;
				while (m_motions[index].IsValid()) {
					index++;
					if (index == numMotion)
						break;
				}
				return index;
			}

			/**
			 * @brief Return the i-th motion.
			 */
			const Motion& GetMotion(int i) const { return m_motions[i]; }

			/**
			 * @brief Return the motions
			 */
			const std::array<Motion, numMotion>& GetMotions() const { return m_motions; }

			/**
			 * @brief Get length.
			 */
			double GetLength(double minTurningRadius) const { return m_length * minTurningRadius; }

			/**
			 * @brief Compute the cost of the path.
			 */
			float ComputeCost(double minTurningRadius, float reverseCostMultiplier, float forwardCostMultiplier, float directionSwitchingCost) const
			{
				if (!m_motions[0].IsValid())
					return std::numeric_limits<float>::infinity();

				if (reverseCostMultiplier == 1.0f && directionSwitchingCost == 0.0f)
					return m_length * minTurningRadius;

				float cost = 0;

				Direction prevDirection = m_motions[0].direction;
				for (const auto& motion : m_motions) {
					if (!motion.IsValid())
						break;

					float motionCost = motion.length * minTurningRadius;
					if (motion.direction == Direction::Forward)
						motionCost *= forwardCostMultiplier;
					else if (motion.direction == Direction::Backward)
						motionCost *= reverseCostMultiplier;
					if (motion.direction != prevDirection)
						motionCost += directionSwitchingCost;

					prevDirection = motion.direction;
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
					motion.direction = motion.direction == Direction::Backward ? Direction::Forward : Direction::Backward;
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
			std::array<Motion, numMotion> m_motions;
			double m_length = 0.0;
		};

		class Solver {
		public:
			/**
			 * @brief Return the shortest distance between the start and the 
			 * goal without computing a Reeds-Shepp path.
			 * @param[in] start The initial pose.
			 * @param[in] goal The goal pose.
			 * @param[in] minTurningRadius Minimum turning radius.
			 * @param[out] word The word representing the optimal path.
			 * @param[out] out Parameters to reconstruct the path.
			 */
			static double GetShortestDistance(const Pose2D<>& start, const Pose2D<>& goal, double minTurningRadius, PathWords* word = nullptr, std::tuple<double, double, double>* out = nullptr)
			{
				const std::array<Pose2D<>, 4> goals = GetGoalArray(start, goal, minTurningRadius);

				// Iterate over pathwords to find the shortest path
				double smallestLength = std::numeric_limits<double>::infinity();
				PathWords smallestWord = PathWords::NoPath;
				double t, u, v;
				for (int w = 0; w < static_cast<int>(PathWords::NumPathWords); w++) {
					double tt, uu, vv;
					PathWords word = static_cast<PathWords>(w);
					double length = GetMotionLengths(word, goals, tt, uu, vv);

					if (length < smallestLength) {
						PP_ASSERT(length >= 0, "Invalid length");
						smallestLength = length;
						smallestWord = word;
						t = tt;
						u = uu;
						v = vv;
					}
				}

				if (word)
					*word = smallestWord;
				if (out)
					*out = { t, u, v };

				return smallestLength;
			}

			/**
			 * @brief Return the shortest path from the start to the goal.
			 * @param[in] start The initial pose.
			 * @param[in] goal The goal pose.
			 * @param[in] minTurningRadius Minimum turning radius.
			 * @param[out] word The word representing the optimal path.
			 */
			static PathSegment GetShortestPath(const Pose2D<>& start, const Pose2D<>& goal, double minTurningRadius, PathWords* word = nullptr)
			{
				std::tuple<double, double, double> params;
				PathWords smallestWord = PathWords::NoPath;
				GetShortestDistance(start, goal, minTurningRadius, &smallestWord, &params);
				if (!IsPathWordValid(smallestWord))
					return PathSegment();

				if (word)
					*word = smallestWord;

				auto& [t, u, v] = params;
				return GetPath(smallestWord, t, u, v);
			}

			/**
			 * @brief Return the optimal path from the start to the goal.
			 * @param[in] start The initial pose.
			 * @param[in] goal The goal pose.
			 * @param[in] minTurningRadius Minimum turning radius.
			 * @param[in] reverseCostMultiplier
			 * @param[in] forwardCostMultiplier
			 * @param[in] directionSwitchingCost
			 * @param[out] word The word representing the optimal path.
			 */
			static PathSegment GetOptimalPath(const Pose2D<>& start, const Pose2D<>& goal, double minTurningRadius, float reverseCostMultiplier, float forwardCostMultiplier, float directionSwitchingCost, PathWords* word = nullptr)
			{
				const std::array<Pose2D<>, 4> goals = GetGoalArray(start, goal, minTurningRadius);

				// Iterate over pathwords to find the shortest path
				float optimalCost = std::numeric_limits<float>::infinity();
				PathWords optimalWord = PathWords::NoPath;
				PathSegment optimalPath;
				for (int w = 0; w < static_cast<int>(PathWords::NumPathWords); w++) {
					double t, u, v;
					PathWords word = static_cast<PathWords>(w);
					double length = GetMotionLengths(word, goals, t, u, v);
					if (length == std::numeric_limits<double>::infinity())
						continue;
					PathSegment path = GetPath(word, t, u, v);
					float cost = path.ComputeCost(minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);

					if (cost < optimalCost) {
						optimalCost = cost;
						optimalWord = word;
						optimalPath = path;
					}
				}

				if (word)
					*word = optimalWord;
				if (!IsPathWordValid(optimalWord))
					return PathSegment();
				return optimalPath;
			}

		private:
			static std::array<Pose2D<>, 4> GetGoalArray(const Pose2D<>& start, const Pose2D<>& goal, double minTurningRadius)
			{
				// Translate the goal so that the start position is at the origin with orientation 0
				Pose2D<> newGoal = goal - start;
				newGoal.x = newGoal.x / minTurningRadius;
				newGoal.y = newGoal.y / minTurningRadius;
				newGoal.theta = newGoal.WrapTheta();

				// clang-format off
				std::array<Pose2D<>, 4> goals = {
					Pose2D<>( newGoal.x,  newGoal.y,  newGoal.theta),
					Pose2D<>(-newGoal.x,  newGoal.y, -newGoal.theta),
					Pose2D<>( newGoal.x, -newGoal.y, -newGoal.theta),
					Pose2D<>(-newGoal.x, -newGoal.y,  newGoal.theta),
				};
				// clang-format on

				return goals;
			}

			[[nodiscard]] static double GetMotionLengths(PathWords word, const std::array<Pose2D<>, 4> goals, double& t, double& u, double& v)
			{
				int w = static_cast<int>(word);

				switch (static_cast<PathWords>(4 * (w / 4))) {
				// clang-format off
				case PathWords::LfSfLf:           return LfSfLf          (goals[w % 4], t, u, v);
				case PathWords::LfSfRf:           return LfSfRf          (goals[w % 4], t, u, v);
				case PathWords::LfRbLf:           return LfRbLf          (goals[w % 4], t, u, v);
				case PathWords::LfRbLb:           return LfRbLb          (goals[w % 4], t, u, v);
				case PathWords::LfRfLb:           return LfRfLb          (goals[w % 4], t, u, v);
				case PathWords::LfRufLubRb:       return LfRufLubRb      (goals[w % 4], t, u, v);
				case PathWords::LfRubLubRf:       return LfRubLubRf      (goals[w % 4], t, u, v);
				case PathWords::LfRbpi2SbLb:      return LfRbpi2SbLb     (goals[w % 4], t, u, v);
				case PathWords::LfRbpi2SbRb:      return LfRbpi2SbRb     (goals[w % 4], t, u, v);
				case PathWords::LfSfRfpi2Lb:      return LfSfRfpi2Lb     (goals[w % 4], t, u, v);
				case PathWords::LfSfLfpi2Rb:      return LfSfLfpi2Rb     (goals[w % 4], t, u, v);
				case PathWords::LfRbpi2SbLbpi2Rf: return LfRbpi2SbLbpi2Rf(goals[w % 4], t, u, v);
					// clang-format on

				default:
					PP_ASSERT(false, "No equivalent Reeds-Shepp base path word.");
					t = 0;
					u = 0;
					v = 0;
					return std::numeric_limits<double>::infinity();
				}
			}

			[[nodiscard]] static PathSegment GetPath(PathWords word, double t, double u, double v)
			{
				PathSegment path;
				int w = static_cast<int>(word);

				switch (static_cast<PathWords>(4 * (w / 4))) {
				// clang-format off
				case PathWords::LfSfLf:           path = LfSfLfPath          (t, u, v); break;
				case PathWords::LfSfRf:           path = LfSfRfPath          (t, u, v); break;
				case PathWords::LfRbLf:           path = LfRbLfPath          (t, u, v); break;
				case PathWords::LfRbLb:           path = LfRbLbPath          (t, u, v); break;
				case PathWords::LfRfLb:           path = LfRfLbPath          (t, u, v); break;
				case PathWords::LfRufLubRb:       path = LfRufLubRbPath      (t, u, v); break;
				case PathWords::LfRubLubRf:       path = LfRubLubRfPath      (t, u, v); break;
				case PathWords::LfRbpi2SbLb:      path = LfRbpi2SbLbPath     (t, u, v); break;
				case PathWords::LfRbpi2SbRb:      path = LfRbpi2SbRbPath     (t, u, v); break;
				case PathWords::LfSfRfpi2Lb:      path = LfSfRfpi2LbPath     (t, u, v); break;
				case PathWords::LfSfLfpi2Rb:      path = LfSfLfpi2RbPath     (t, u, v); break;
				case PathWords::LfRbpi2SbLbpi2Rf: path = LfRbpi2SbLbpi2RfPath(t, u, v); break;
					// clang-format on

				default:
					PP_ASSERT(false, "Word is not a valid Reeds-Shepp path word.");
					return PathSegment();
				}

				switch (w % 4) {
				case 1:
					path = path.TimeflipTransform();
					break;
				case 2:
					path = path.ReflectTransform();
					break;
				case 3:
					path = path.TimeflipTransform().ReflectTransform();
					break;
				default:
					break;
				}
				return path;
			}

			static bool IsAngleInvalid(double theta)
			{
				return theta < 0 || theta > M_PI;
			}

			static double WrapAngle(double theta)
			{
				return Maths::Modulo(theta + M_PI, 2 * M_PI) - M_PI;
			}

			static double LfSfLf(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.1
				double x = goal.x - sin(goal.theta);
				double y = goal.y - 1 + cos(goal.theta);

				u = sqrt(x * x + y * y);
				t = atan2(y, x);
				v = WrapAngle(goal.theta - t);

				if (IsAngleInvalid(t) || IsAngleInvalid(v))
					return std::numeric_limits<double>::infinity();

				return t + u + v;
			}

			static double LfSfRf(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.2
				double x = goal.x + sin(goal.theta);
				double y = goal.y - 1 - cos(goal.theta);

				double u1squared = x * x + y * y;
				double t1 = std::atan2(y, x);

				if (u1squared < 4)
					std::numeric_limits<double>::infinity();

				u = sqrt(u1squared - 4);
				double phi = atan2(2, u);
				t = WrapAngle(t1 + phi);
				v = WrapAngle(t - goal.theta);

				if (IsAngleInvalid(t) || IsAngleInvalid(v))
					return std::numeric_limits<double>::infinity();

				return t + u + v;
			}

			static double LfRbLf(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.3
				// Uses a modified formula adapted from the c_c_c function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x - sin(goal.theta);
				double eta = goal.y - 1 + cos(goal.theta);

				double u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 4)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);
				double alpha = acos(u1 / 4.0);
				t = Maths::Modulo(M_PI_2 + alpha + phi, 2 * M_PI);
				u = Maths::Modulo(M_PI - 2 * alpha, 2 * M_PI);
				v = Maths::Modulo(goal.theta - t - u, 2 * M_PI);

				if (IsAngleInvalid(t) || IsAngleInvalid(u) || IsAngleInvalid(v))
					return std::numeric_limits<double>::infinity();

				return t + u + v;
			}

			static double LfRbLb(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.4
				// Uses a modified formula adapted from the c_cc function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x - sin(goal.theta);
				double eta = goal.y - 1 + cos(goal.theta);

				double u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 4)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);
				double alpha = acos(u1 / 4.0);
				t = Maths::Modulo(M_PI_2 + alpha + phi, 2 * M_PI);
				u = Maths::Modulo(M_PI - 2 * alpha, 2 * M_PI);
				v = Maths::Modulo(t + u - goal.theta, 2 * M_PI);

				return t + u + v;
			}

			static double LfRfLb(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.4
				// Uses a modified formula adapted from the cc_c function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x - sin(goal.theta);
				double eta = goal.y - 1 + cos(goal.theta);

				double u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 4)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);
				u = acos((8 - u1 * u1) / 8.0);
				double va = sin(u);
				double alpha = asin(2 * va / u1);
				t = Maths::Modulo(M_PI_2 - alpha + phi, 2 * M_PI);
				v = Maths::Modulo(t - u - goal.theta, 2 * M_PI);

				return t + u + v;
			}

			static double LfRufLubRb(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.7
				// Uses a modified formula adapted from the ccu_cuc function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x + sin(goal.theta);
				double eta = goal.y - 1 - cos(goal.theta);

				double u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 4)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);

				if (u1 > 2) {
					double alpha = acos(u1 / 4 - 0.5);
					t = Maths::Modulo(M_PI_2 + phi - alpha, 2 * M_PI);
					u = Maths::Modulo(M_PI - alpha, 2 * M_PI);
					v = Maths::Modulo(goal.theta - t + 2 * u, 2 * M_PI);
				} else {
					double alpha = acos(u1 / 4 + 0.5);
					t = Maths::Modulo(M_PI_2 + phi + alpha, 2 * M_PI);
					u = Maths::Modulo(alpha, 2 * M_PI);
					v = Maths::Modulo(goal.theta - t + 2 * u, 2 * M_PI);
				}

				return t + u + u + v;
			}

			static double LfRubLubRf(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.8
				// Uses a modified formula adapted from the c_cucu_c function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x + sin(goal.theta);
				double eta = goal.y - 1 - cos(goal.theta);

				double u1 = sqrt(xi * xi + eta * eta);
				if (u1 > 6)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);
				double va1 = 1.25f - u1 * u1 / 16;
				if (va1 < 0 || va1 > 1)
					return std::numeric_limits<double>::infinity();

				u = acos(va1);
				double va2 = sin(u);
				double alpha = asin(2 * va2 / u1);
				t = Maths::Modulo(M_PI_2 + phi + alpha, 2 * M_PI);
				v = Maths::Modulo(t - goal.theta, 2 * M_PI);

				return t + u + u + v;
			}

			static double LfRbpi2SbLb(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.9
				// Uses a modified formula adapted from the c_c2sca function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x - sin(goal.theta);
				double eta = goal.y - 1 + cos(goal.theta);

				double u1squared = xi * xi + eta * eta;
				if (u1squared < 4)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);

				u = sqrt(u1squared - 4) - 2;
				if (u < 0)
					return std::numeric_limits<double>::infinity();

				double alpha = atan2(2, u + 2);
				t = Maths::Modulo(M_PI_2 + phi + alpha, 2 * M_PI);
				v = Maths::Modulo(t + M_PI_2 - goal.theta, 2 * M_PI);

				return t + M_PI_2 + u + v;
			}

			static double LfRbpi2SbRb(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.10
				// Uses a modified formula adapted from the c_c2scb function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x + sin(goal.theta);
				double eta = goal.y - 1 - cos(goal.theta);

				double u1 = sqrt(xi * xi + eta * eta);
				if (u1 < 2)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);

				t = Maths::Modulo(M_PI_2 + phi, 2 * M_PI);
				u = u1 - 2;
				v = Maths::Modulo(goal.theta - t - M_PI_2, 2 * M_PI);

				return t + M_PI_2 + u + v;
			}

			static double LfSfRfpi2Lb(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.9 (reversed)
				// Uses a modified formula adapted from the csc2_ca function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x - sin(goal.theta);
				double eta = goal.y - 1 + cos(goal.theta);

				double u1squared = xi * xi + eta * eta;
				if (u1squared < 4)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);

				u = sqrt(u1squared - 4) - 2;
				if (u < 0)
					return std::numeric_limits<double>::infinity();

				double alpha = atan2(u + 2, 2);
				t = Maths::Modulo(M_PI_2 + phi - alpha, 2 * M_PI);
				v = Maths::Modulo(t - M_PI_2 - goal.theta, 2 * M_PI);

				return t + u + M_PI_2 + v;
			}

			static double LfSfLfpi2Rb(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.10 (reversed)
				// Uses a modified formula adapted from the csc2_cb function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x + sin(goal.theta);
				double eta = goal.y - 1 - cos(goal.theta);

				double u1 = sqrt(xi * xi + eta * eta);
				if (u1 < 2)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);

				t = Maths::Modulo(phi, 2 * M_PI);
				u = u1 - 2;
				v = Maths::Modulo(-t - M_PI_2 + goal.theta, 2 * M_PI);

				return t + u + M_PI_2 + v;
			}

			static double LfRbpi2SbLbpi2Rf(const Pose2D<>& goal, double& t, double& u, double& v)
			{
				// Reeds-Shepp 8.11
				// Uses a modified formula adapted from the c_c2sc2_c function
				// from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
				double xi = goal.x + sin(goal.theta);
				double eta = goal.y - 1 - cos(goal.theta);

				double u1squared = xi * xi + eta * eta;
				if (u1squared < 16)
					return std::numeric_limits<double>::infinity();

				double phi = atan2(eta, xi);

				u = sqrt(u1squared - 4) - 4;
				if (u < 0)
					return std::numeric_limits<double>::infinity();

				double alpha = atan2(2, u + 4);
				t = Maths::Modulo(M_PI_2 + phi + alpha, 2 * M_PI);
				v = Maths::Modulo(t - goal.theta, 2 * M_PI);

				return t + u + v + M_PI;
			}

			static PathSegment LfSfLfPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Straight, Direction::Forward, u);
				motions.AddMotion(Steer::Left, Direction::Forward, v);
				return motions;
			}

			static PathSegment LfSfRfPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Straight, Direction::Forward, u);
				motions.AddMotion(Steer::Right, Direction::Forward, v);
				return motions;
			}

			static PathSegment LfRbLfPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Right, Direction::Backward, u);
				motions.AddMotion(Steer::Left, Direction::Forward, v);
				return motions;
			}

			static PathSegment LfRbLbPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Right, Direction::Backward, u);
				motions.AddMotion(Steer::Left, Direction::Backward, v);
				return motions;
			}

			static PathSegment LfRfLbPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Right, Direction::Forward, u);
				motions.AddMotion(Steer::Left, Direction::Backward, v);
				return motions;
			}

			static PathSegment LfRufLubRbPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Right, Direction::Forward, u);
				motions.AddMotion(Steer::Left, Direction::Backward, u);
				motions.AddMotion(Steer::Right, Direction::Backward, v);
				return motions;
			}

			static PathSegment LfRubLubRfPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Right, Direction::Backward, u);
				motions.AddMotion(Steer::Left, Direction::Backward, u);
				motions.AddMotion(Steer::Right, Direction::Forward, v);
				return motions;
			}

			static PathSegment LfRbpi2SbLbPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Right, Direction::Backward, M_PI_2);
				motions.AddMotion(Steer::Straight, Direction::Backward, u);
				motions.AddMotion(Steer::Left, Direction::Backward, v);
				return motions;
			}

			static PathSegment LfRbpi2SbRbPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Right, Direction::Backward, M_PI_2);
				motions.AddMotion(Steer::Straight, Direction::Backward, u);
				motions.AddMotion(Steer::Right, Direction::Backward, v);
				return motions;
			}

			static PathSegment LfSfRfpi2LbPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Straight, Direction::Forward, u);
				motions.AddMotion(Steer::Right, Direction::Forward, M_PI_2);
				motions.AddMotion(Steer::Left, Direction::Backward, v);
				return motions;
			}

			static PathSegment LfSfLfpi2RbPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Straight, Direction::Forward, u);
				motions.AddMotion(Steer::Left, Direction::Forward, M_PI_2);
				motions.AddMotion(Steer::Right, Direction::Backward, v);
				return motions;
			}

			static PathSegment LfRbpi2SbLbpi2RfPath(double t, double u, double v)
			{
				PathSegment motions;
				motions.AddMotion(Steer::Left, Direction::Forward, t);
				motions.AddMotion(Steer::Right, Direction::Backward, M_PI_2);
				motions.AddMotion(Steer::Straight, Direction::Backward, u);
				motions.AddMotion(Steer::Left, Direction::Backward, M_PI_2);
				motions.AddMotion(Steer::Right, Direction::Forward, v);
				return motions;
			}
		};
	}

	class PathReedsShepp : public Path<Pose2D<>> {
		using State = Pose2D<>;

	public:
		PathReedsShepp(const State& init, const ReedsShepp::PathSegment& pathSegment, double minTurningRadius) :
			Path<State>(init, pathSegment.GetLength(minTurningRadius)),
			m_pathSegment(pathSegment), m_minTurningRadius(minTurningRadius)
		{
			m_final = Interpolate(1.0);
		}

		virtual State Interpolate(double ratio) const override
		{
			PP_ASSERT(ratio >= 0 && ratio <= 1, "Ratio should be between 0 and 1.");

			const double& totalLength = m_length;
			if (totalLength == 0)
				return m_init;

			State interp = m_init;
			double length = 0;
			for (const auto& motion : m_pathSegment.m_motions) {
				if (!motion.IsValid())
					break;

				double motionLength = motion.length * m_minTurningRadius;
				if (motionLength == 0)
					continue;

				double motionRatio = (ratio * totalLength - length) / motionLength;
				motionRatio = std::min(motionRatio, 1.0);

				// clang-format off
				switch (motion.steer) {
					case Steer::Straight: interp = Straight(interp, motion.direction,               motion.length * motionRatio); break;
					case Steer::Left:     interp = Turn    (interp, motion.direction, motion.steer, motion.length * motionRatio); break;
					case Steer::Right:    interp = Turn    (interp, motion.direction, motion.steer, motion.length * motionRatio); break;
				}
				// clang-format on

				length += motionLength;
				if (length >= ratio * totalLength)
					break;
			}

			return interp;
		}

		virtual void Truncate(double ratio) override
		{
			PP_ASSERT(ratio >= 0 && ratio <= 1, "Ratio should be between 0 and 1.");

			// Update the final state and modify the Reeds-Shepp motions
			const double& totalLength = m_length;
			if (totalLength == 0)
				m_final = m_init;
			else {
				State interp = m_init;
				double length = 0;
				for (int i = 0; i < ReedsShepp::PathSegment::numMotion; i++) {
					const auto& motion = m_pathSegment.m_motions[i];
					if (!motion.IsValid())
						break;

					double motionLength = motion.length * m_minTurningRadius;
					if (motionLength == 0)
						continue;

					double motionRatio = (ratio * totalLength - length) / motionLength;
					motionRatio = std::min(motionRatio, 1.0);

					// clang-format off
					switch (motion.steer) {
						case Steer::Straight: interp = Straight(interp, motion.direction,               motion.length * motionRatio); break;
						case Steer::Left:     interp = Turn    (interp, motion.direction, motion.steer, motion.length * motionRatio); break;
						case Steer::Right:    interp = Turn    (interp, motion.direction, motion.steer, motion.length * motionRatio); break;
					}
					// clang-format on

					length += motionLength;
					if (length >= ratio * totalLength) {
						m_pathSegment.m_motions[i].length *= motionRatio;
						for (int ii = i + 1; ii < ReedsShepp::PathSegment::numMotion; ii++) {
							m_pathSegment.m_motions[i] = ReedsShepp::Motion();
						}
						break;
					}
				}
				m_final = interp;
			}

			// Update the path length
			m_length *= ratio;
		}

	private:
		State Straight(const State& start, Direction direction, double length) const
		{
			if (direction == Direction::Backward)
				length = -length;

			length *= m_minTurningRadius;

			State end = {
				start.x + length * cos(start.theta),
				start.y + length * sin(start.theta),
				start.theta
			};
			return end;
		}

		State Turn(const State& start, Direction direction, Steer steer, double turnAngle) const
		{
			if (direction == Direction::Backward)
				turnAngle = -turnAngle;
			double phi = turnAngle / 2;
			double cosPhi = cos(phi);
			double sinPhi = sin(phi);
			double L = 2 * sinPhi * m_minTurningRadius;
			double x = L * cosPhi;
			double y = L * sinPhi;
			if (steer == Steer::Right) {
				y *= -1;
				turnAngle *= -1;
			}

			return start + State(x, y, turnAngle);
		}

	private:
		ReedsShepp::PathSegment m_pathSegment;
		double m_minTurningRadius = 1;
	};

	class StateSpaceReedsShepp : public StateSpace<Pose2D<>, 3> {
		using Pose = Pose2D<>;

	public:
		StateSpaceReedsShepp(std::array<std::array<double, 2>, 3> bounds = { { { -100, 100 }, { -100, 100 }, { -M_PI, M_PI } } }) :
			StateSpace<Pose, 3>(bounds) { }
		~StateSpaceReedsShepp() = default;

		/// @brief Compute the distance of a Reeds-Shepp path.
		double ComputeDistance(const ReedsShepp::PathSegment& path)
		{
			return path.GetLength(minTurningRadius);
		}
		/// @brief Compute the shortest distance from the pose @from to @to
		/// when driving Reeds-Shepp paths.
		virtual double ComputeDistance(const Pose& from, const Pose& to) override
		{
			return ReedsShepp::Solver::GetShortestDistance(from, to, minTurningRadius);
		}
		/// @brief Compute the shortest path between two poses
		Ref<PathReedsShepp> ComputeShortestPath(const Pose& from, const Pose& to)
		{
			auto pathSegment = ReedsShepp::Solver::GetShortestPath(from, to, minTurningRadius);
			return makeRef<PathReedsShepp>(from, pathSegment, minTurningRadius);
		}

		/// @brief Compute the cost of a Reeds-Shepp path.
		double ComputeCost(const ReedsShepp::PathSegment& path) const
		{
			return path.ComputeCost(minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
		}
		/// @brief Compute the cost associated the the most optimal Reeds-Shepp
		/// path.
		double ComputeCost(const Pose2D<>& from, const Pose2D<>& to) const
		{
			auto path = ReedsShepp::Solver::GetOptimalPath(from, to, minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
			return ComputeCost(path);
		}
		/// @brief Compute the optimal path between two poses
		Ref<PathReedsShepp> ComputeOptimalPath(const Pose& from, const Pose& to)
		{
			auto pathSegment = ReedsShepp::Solver::GetOptimalPath(from, to, minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
			return makeRef<PathReedsShepp>(from, pathSegment, minTurningRadius);
		}

	public:
		double minTurningRadius = 1.0;
		double reverseCostMultiplier = 1.0, forwardCostMultiplier = 1.0;
		double directionSwitchingCost = 0.0;
	};
}
