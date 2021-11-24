#pragma once

#include "core/base.h"
#include "geometry/2dplane.h"
#include "state_validator/state_validator.h"

namespace Planner {

	class OccupancyMap;
	class PlanarStateSpace;

	/// @brief State validator based on an occupancy map.
	class StateValidatorOccupancyMap : public PlanarStateValidator {
	public:
		StateValidatorOccupancyMap(const Ref<PlanarStateSpace>& stateSpace, const Ref<OccupancyMap>& map);
		~StateValidatorOccupancyMap() = default;

		/// @copydoc Planner::StateValidator::IsStateValid
		virtual bool IsStateValid(const Pose2d& state) override;

		/// @copydoc Planner::StateValidator::IsPathValid
		virtual bool IsPathValid(const PlanarPath& path, float* last = nullptr) override;

		Ref<OccupancyMap>& GetOccupancyMap() { return m_map; }

	public:
		float minPathInterpolationDistance = 0.1f;
		float minSafeRadius = 1.0f;

	private:
		Ref<OccupancyMap> m_map;
	};
}
