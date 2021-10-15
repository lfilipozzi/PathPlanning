#pragma once

#include "geometry/2dplane.h"
#include "paths/path.h"

#include <array>

namespace Planner {

	class PathReedsShepp;

	namespace ReedsShepp {
		/// @brief Path word for Reeds-Shepp optimal path
		enum class PathWords {
			NoPath = -1,

			// clang-format off
			// Reeds-Shepp 8.1: CSC, same turn
			LfSfLf = 0, LbSbLb, RfSfRf, RbSbRb,
			// Reeds-Shepp 8.2: CSC, different turn
			LfSfRf, LbSbRb, RfSfLf, RbSbLb,
			// Reeds-Shepp 8.3: C|C|C
			LfRbLf, LbRfLb, RfLbRf, RbLfRb,
			// Reeds-Shepp 8.4: C|CC
			LfRbLb, LbRfLf, RfLbRb, RbLfRf,
			// Reeds-Shepp 8.4: CC|C
			LfRfLb, LbRbLf, RfLfRb, RbLbRf,
			// Reeds-Shepp 8.7: CCu|CuC
			LfRufLubRb, LbRubLufRf, RfLufRubLb, RbLubRufLf,
			// Reeds-Shepp 8.8: C|CuCu|C
			LfRubLubRf, LbRufLufRb, RfLubRubLf, RbLufRufLb,
			// Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
			LfRbpi2SbLb, LbRfpi2SfLf, RfLbpi2SbRb, RbLfpi2SfRf,
			// Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
			LfRbpi2SbRb, LbRfpi2SfRf, RfLbpi2SbLb, RbLfpi2SfLf,
			// Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
			LfSfRfpi2Lb, LbSbRbpi2Lf, RfSfLfpi2Rb, RbSbLbpi2Rf,
			// Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
			LfSfLfpi2Rb, LbSbLbpi2Rf, RfSfRfpi2Lb, RbSbRbpi2Lf,
			// Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
			LfRbpi2SbLbpi2Rf, LbRfpi2SfLfpi2Rb, RfLbpi2SbRbpi2Lf, RbLfpi2SfRfpi2Lb,
			// clang-format on

			NumPathWords,
		};

		/// @brief Check if a word refers to a valid Reed-Shepp path word.
		bool IsPathWordValid(PathWords word);

		/// @brief Represents a motion with a constant steering and direction over some length.
		struct Motion {
			Steer steer;
			Direction direction = Direction::NoMotion;
			double length = std::numeric_limits<double>::infinity();

			Motion() = default;
			Motion(Steer steer, Direction direction, double length) :
				steer(steer), direction(direction), length(length) { }

			/// @brief Check if an motion is valid
			bool IsValid() const;
		};

		/// @brief Represents a Reed-Shepp path segment.
		/// @details All paths are initialized with an infinite length. Paths
		/// with an infinite length correspond to a non-valid path. Lengths of each
		/// motion are normalized.
		class PathSegment {
			friend class ::Planner::PathReedsShepp;

		public:
			static constexpr unsigned int numMotion = 5;

		public:
			/// @brief Construct a set of invalid motions.
			PathSegment();

			/// @brief Add a motion to the path.
			void AddMotion(Steer steer, Direction direction, double length);

			/// @brief Get number of valid motion in the path.
			int GetNumMotions() const;
			/// @brief Return the i-th motion.
			const Motion& GetMotion(int i) const { return m_motions[i]; }
			/// @brief Return the motions
			const std::array<Motion, numMotion>& GetMotions() const { return m_motions; }

			/// @brief Get length.
			double GetLength(double minTurningRadius) const { return m_length * minTurningRadius; }
			/// @brief Compute the cost of the path.
			float ComputeCost(double minTurningRadius, float reverseCostMultiplier, float forwardCostMultiplier, float directionSwitchingCost) const;

			/// @brief Inverse the direction of motion.
			/// @return A pointer to @this.
			PathSegment* TimeflipTransform();

			/// @brief Inverse the lateral direction of motion.
			/// @return A pointer to @this.
			PathSegment* ReflectTransform();

		private:
			std::array<Motion, numMotion> m_motions;
			double m_length = 0.0;
		};

		class Solver {
		public:
			/// @brief Return the shortest distance between the start and the
			/// goal without computing a Reeds-Shepp path.
			/// @param[in] start The initial pose.
			/// @param[in] goal The goal pose.
			/// @param[in] minTurningRadius Minimum turning radius.
			/// @param[out] word The word representing the optimal path.
			/// @param[out] out Parameters to reconstruct the path.
			static double GetShortestDistance(const Pose2d& start, const Pose2d& goal, double minTurningRadius, PathWords* word = nullptr, std::tuple<double, double, double>* out = nullptr);

			/// @brief Return the shortest path from the start to the goal.
			/// @param[in] start The initial pose.
			/// @param[in] goal The goal pose.
			/// @param[in] minTurningRadius Minimum turning radius.
			/// @param[out] word The word representing the optimal path.
			static PathSegment GetShortestPath(const Pose2d& start, const Pose2d& goal, double minTurningRadius, PathWords* word = nullptr);

			/// @brief Return the optimal path from the start to the goal.
			/// @param[in] start The initial pose.
			/// @param[in] goal The goal pose.
			/// @param[in] minTurningRadius Minimum turning radius.
			/// @param[in] reverseCostMultiplier
			/// @param[in] forwardCostMultiplier
			/// @param[in] directionSwitchingCost
			/// @param[out] word The word representing the optimal path.
			static PathSegment GetOptimalPath(const Pose2d& start, const Pose2d& goal, double minTurningRadius, float reverseCostMultiplier, float forwardCostMultiplier, float directionSwitchingCost, PathWords* word = nullptr);
		};
	}
}
