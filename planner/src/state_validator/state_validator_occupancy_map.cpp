#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/occupancy_map.h"

namespace Planner {
	StateValidatorOccupancyMap::StateValidatorOccupancyMap(const Ref<StateSpace<Pose2d, 3, double>>& stateSpace) :
		StateValidator<Pose2d, 3, double>(stateSpace) { }

	void StateValidatorOccupancyMap::SetOccupancyMap(const Ref<OccupancyMap>& map)
	{
		m_map = map;
		float width = stateSpace->bounds[1].position.x() - stateSpace->bounds[0].position.x();
		float height = stateSpace->bounds[1].position.y() - stateSpace->bounds[0].position.y();
		m_map->InitializeSize(width, height);
	}

	bool StateValidatorOccupancyMap::IsStateValid(const Pose2d& state)
	{
		// Validate bounds
		Pose2d localState(m_map->WorldPositionToLocalPosition(state.position), state.theta);
		if (!stateSpace->ValidateBounds(localState))
			return false;
		// Check intersection with obstacles
		GridCellPosition cell = m_map->WorldPositionToGridCell(state.position);
		if (cell.IsValid()) {
			float distance = m_map->GetObstacleDistanceMap()->GetDistanceToNearestObstacle(cell);
			return distance >= minSafeRadius;
		}
		return false;
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
			GridCellPosition cell = m_map->WorldPositionToGridCell(state.position);
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
