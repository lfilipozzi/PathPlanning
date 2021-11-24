#pragma once

#include "paths/path.h"

namespace Planner {

	class PathR2 : public PathR2Base {
	public:
		PathR2(const Point2d& from, const Point2d& to);

		/// @copydoc Planner::Path::Interpolate
		virtual Point2d Interpolate(double ratio) const override;
		/// @copydoc Planer::Path::Truncate
		virtual void Truncate(double ratio) override;
	};

	class PathConnectionR2 : public PathConnection<Point2d> {
	public:
		PathConnectionR2() = default;

		/// @copydoc Planner::PathConnection::Connect
		virtual Ref<PathR2Base> Connect(const Point2d& from, const Point2d& to) override;
	};

}
