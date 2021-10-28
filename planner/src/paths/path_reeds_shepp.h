#pragma once

#include "paths/path.h"
#include "geometry/reeds_shepp.h"

namespace Planner {
	class PathReedsShepp : public PlanarPath {
	public:
		PathReedsShepp(const Pose2d& init, const ReedsShepp::PathSegment& pathSegment, double minTurningRadius);

		/// @copydoc Planner::Path::Interpolate
		virtual Pose2d Interpolate(double ratio) const override;
		/// @copydoc Planer::Path::Truncate
		virtual void Truncate(double ratio) override;
		/// @copydoc Planner::Path::GetDirection
		virtual Direction GetDirection(double ratio) const override;
		/// @copydoc Planner::Path::ComputeCost
		virtual double ComputeCost(double directionSwitchingCost, double reverseCostMultiplier, double forwardCostMultiplier) const override;

	private:
		Pose2d Straight(const Pose2d& start, Direction direction, double length) const;
		Pose2d Turn(const Pose2d& start, Direction direction, Steer steer, double turnAngle) const;

	private:
		ReedsShepp::PathSegment m_pathSegment;
		double m_minTurningRadius = 1;
	};
}
