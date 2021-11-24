#pragma once

#include "paths/path.h"

namespace Planner {

	class PathSE2 : public PlanarPath {
	public:
		PathSE2(const Pose2d& from, const Pose2d& to);

		/// @copydoc Planner::Path::Interpolate
		virtual Pose2d Interpolate(double ratio) const override;
		/// @copydoc Planer::Path::Truncate
		virtual void Truncate(double ratio) override;
	};

	class PathConnectionSE2 : public PlanarPathConnection {
	public:
		PathConnectionSE2() = default;

		/// @copydoc Planner::PathConnection::Connect
		virtual Ref<Path<Pose2d>> Connect(const Pose2d& from, const Pose2d& to) override;
	};

}
