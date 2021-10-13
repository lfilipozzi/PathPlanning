#include "paths/path_reeds_shepp.h"
#include "core/base.h"

namespace Planner {
	PathReedsShepp::PathReedsShepp(const Pose2D<>& init, const ReedsShepp::PathSegment& pathSegment, double minTurningRadius) :
		Path<Pose2D<>>(init, pathSegment.GetLength(minTurningRadius)),
		m_pathSegment(pathSegment), m_minTurningRadius(minTurningRadius)
	{
		m_final = Interpolate(1.0);
	}

	Pose2D<> PathReedsShepp::Interpolate(double ratio) const
	{
		PP_ASSERT(ratio >= 0 && ratio <= 1, "Ratio should be between 0 and 1.");

		const double& totalLength = m_length;
		if (totalLength == 0)
			return m_init;

		Pose2D<> interp = m_init;
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

	void PathReedsShepp::Truncate(double ratio)
	{
		PP_ASSERT(ratio >= 0 && ratio <= 1, "Ratio should be between 0 and 1.");

		// Update the final state and modify the Reeds-Shepp motions
		const double& totalLength = m_length;
		if (totalLength == 0)
			m_final = m_init;
		else {
			Pose2D<> interp = m_init;
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

	Pose2D<> PathReedsShepp::Straight(const Pose2D<>& start, Direction direction, double length) const
	{
		if (direction == Direction::Backward)
			length = -length;

		length *= m_minTurningRadius;

		Pose2D<> end(
			start.x + length * cos(start.theta),
			start.y + length * sin(start.theta),
			start.theta);
		return end;
	}

	Pose2D<> PathReedsShepp::Turn(const Pose2D<>& start, Direction direction, Steer steer, double turnAngle) const
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

		return start + Pose2D<>(x, y, turnAngle);
	}
}
