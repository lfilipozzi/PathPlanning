#pragma once

#include "paths/path.h"

namespace Planner {

	class PathSE2 : public PathSE2Base {
	public:
		PathSE2(const Pose2d& from, const Pose2d& to);

		/// @copydoc Planner::Path::Interpolate
		virtual Pose2d Interpolate(double ratio) const override;
		/// @copydoc Planer::Path::Truncate
		virtual void Truncate(double ratio) override;
	};

	class PathConnectionSE2 : public PathConnectionSE2Base {
	public:
		PathConnectionSE2() = default;

		/// @copydoc Planner::PathConnection::Connect
		virtual Ref<PathSE2Base> Connect(const Pose2d& from, const Pose2d& to) override;
	};

}
