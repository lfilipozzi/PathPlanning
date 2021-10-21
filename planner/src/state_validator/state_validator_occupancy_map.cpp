#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/occupancy_map.h"

namespace Planner {
	StateValidatorOccupancyMap::StateValidatorOccupancyMap(const Ref<OccupancyMap>& map) :
		m_map(map)
	{
	}

	bool StateValidatorOccupancyMap::IsStateValid(const Pose2d& state)
	{
		// TODO
		return true;
	}

	bool StateValidatorOccupancyMap::IsPathValid(const Path<Pose2d>& path, float* last)
	{
		const double pathLength = path.GetLength();

		double length = 0.0;
		while (length < pathLength) {
			Pose2d state = path.Interpolate(length / pathLength);

			// Check current state
			if (!IsStateValid(state)) {
				if (last)
					*last = length / pathLength;
				return false;
			}
			// Update length
			GridCellPosition cell = m_map->WorldToGridPosition(state.position);
			if (cell.IsValid()) {
				float distance = m_map->GetObstacleDistanceMap()->GetDistanceToNearestObstacle(cell);
				length += std::max(distance - minSafeRadius, minPathInterpolationDistance);
			} else {
				length += minPathInterpolationDistance;
			}
		}

		if (last)
			*last = 1.0f;
		return true;
	}
}
