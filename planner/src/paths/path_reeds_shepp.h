#pragma once

#include "paths/path.h"
#include "geometry/reeds_shepp.h"

namespace Planner {
	class PathReedsShepp : public Path<Pose2D<>> {
	public:
		PathReedsShepp(const Pose2D<>& init, const ReedsShepp::PathSegment& pathSegment, double minTurningRadius);

		/// @copydoc Planer::Path::Interpolate
		virtual Pose2D<> Interpolate(double ratio) const override;
		/// @copydoc Planer::Path::Truncate
		virtual void Truncate(double ratio) override;

	private:
		Pose2D<> Straight(const Pose2D<>& start, Direction direction, double length) const;
		Pose2D<> Turn(const Pose2D<>& start, Direction direction, Steer steer, double turnAngle) const;

	private:
		ReedsShepp::PathSegment m_pathSegment;
		double m_minTurningRadius = 1;
	};
}
