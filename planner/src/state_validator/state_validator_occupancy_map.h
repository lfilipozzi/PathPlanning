#pragma once

#include "core/base.h"
#include "geometry/2dplane.h"
#include "state_validator/state_validator.h"

namespace Planner {

	class OccupancyMap;

	/// @brief State validator based on an occupancy map.
	class StateValidatorOccupancyMap : public PlanarStateValidator {
	public:
		StateValidatorOccupancyMap(const Ref<PlanarStateSpace>& stateSpace);
		~StateValidatorOccupancyMap() = default;

		/// @brief Set the occupancy map to be used;
		void SetOccupancyMap(const Ref<OccupancyMap>& map);

		/// @copydoc Planner::StateValidator::IsStateValid
		virtual bool IsStateValid(const Pose2d& state) override;

		/// @copydoc Planner::StateValidator::IsPathValid
		virtual bool IsPathValid(const PlanarPath& path, float* last = nullptr) override;

		Ref<OccupancyMap>& GetOccupancyMap() { return m_map; }

	public:
		double minPathInterpolationDistance = 0.1;
		double minSafeRadius = 1.0;

	private:
		Ref<OccupancyMap> m_map;
	};
}
