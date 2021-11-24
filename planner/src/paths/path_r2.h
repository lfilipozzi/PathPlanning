#pragma once

#include "paths/path.h"

namespace Planner {

	class PathR2 : public Path<Point2d> {
	public:
		PathR2(const Point2d& from, const Point2d& to);

		/// @copydoc Planner::Path::Interpolate
		virtual Point2d Interpolate(double ratio) const override;
		/// @copydoc Planer::Path::Truncate
		virtual void Truncate(double ratio) override;
		/// @copydoc Planner::Path::GetDirection
		virtual Direction GetDirection(double ratio) const override;
		/// @copydoc Planner::Path::ComputeCost
		virtual double ComputeCost(double directionSwitchingCost, double reverseCostMultiplier, double forwardCostMultiplier) const override;
	};

	class PathConnectionR2 : public PathConnection<Point2d> {
	public:
		PathConnectionR2() = default;

		/// @copydoc Planner::PathConnection::Connect
		virtual Ref<Path<Point2d>> Connect(const Point2d& from, const Point2d& to) override;
	};

}
