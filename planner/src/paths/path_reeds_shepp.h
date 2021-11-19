#pragma once

#include "paths/path.h"
#include "geometry/reeds_shepp.h"

namespace Planner {
	class PathReedsShepp : public PlanarNonHolonomicPath {
	public:
		PathReedsShepp(const Pose2d& init, const ReedsShepp::PathSegment& pathSegment, double minTurningRadius);

		/// @copydoc Planner::Path::Interpolate
		virtual Pose2d Interpolate(double ratio) const override;
		using PlanarNonHolonomicPath::Interpolate;
		/// @copydoc Planer::Path::Truncate
		virtual void Truncate(double ratio) override;
		/// @copydoc Planer::PlanarNonHolonomicPath::GetCuspPointRatios
		virtual std::set<double> GetCuspPointRatios() const override;
		/// @copydoc Planner::PlanarNonHolonomicPath::GetDirection
		virtual Direction GetDirection(double ratio) const override;

		/// @brief Compute the cost associated to the path.
		/// @details The cost penalizes the distance traveled with a cost
		/// multiplier @forwardCostMultiplier for forward motion, and
		/// @reverseCostMultiplier for reverse motion. An additional cost is
		/// added when switching the direction of motionwith cost
		/// @directionSwitchingCost.
		double ComputeCost(double directionSwitchingCost, double reverseCostMultiplier, double forwardCostMultiplier) const;

		double GetMinTurningRadius() const { return m_minTurningRadius; }

	private:
		Pose2d Straight(const Pose2d& start, Direction direction, double length) const;
		Pose2d Turn(const Pose2d& start, Direction direction, Steer steer, double turnAngle) const;

	private:
		ReedsShepp::PathSegment m_pathSegment;
		double m_minTurningRadius = 1;
	};

	class ReedsSheppConnection : public PlanarPathConnection {
	public:
		ReedsSheppConnection(double minTurningRadius = 1.0, double directionSwitchingCost = 0.0,
			double reverseCostMultiplier = 1.0, double forwardCostMultiplier = 1.0) :
			m_minTurningRadius(minTurningRadius),
			m_directionSwitchingCost(directionSwitchingCost),
			m_reverseCostMultiplier(reverseCostMultiplier), m_forwardCostMultiplier(forwardCostMultiplier) { }

		/// @copydoc Planner::PathConnection::Connect
		virtual Ref<PlanarPath> Connect(const Pose2d& from, const Pose2d& to) override;

	private:
		double m_minTurningRadius;
		double m_directionSwitchingCost, m_reverseCostMultiplier, m_forwardCostMultiplier;
	};
}
