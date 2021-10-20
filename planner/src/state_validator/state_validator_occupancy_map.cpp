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
		Pose2d state = path.GetInitialState();

		double distance = 0.0;
		while (distance < pathLength) {
			// Check current state
			if (!IsStateValid(state)) {
				if (last)
					*last = distance / pathLength;
				return false;
			}
			// Update current state
			GridCellPosition cell = m_map->WorldToGridPosition(state.position);
			distance = m_map->GetObstacleDistanceMap()->GetDistanceToNearestObstacle(cell);
			distance = std::max(distance - minSafeRadius, minPathInterpolationDistance);
			state = path.Interpolate(distance / pathLength);
		}

		if (last)
			*last = 1.0f;
		return true;
	}
}
