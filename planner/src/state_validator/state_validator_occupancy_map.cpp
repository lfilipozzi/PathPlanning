#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/occupancy_map.h"
#include "state_space/planar_state_space.h"

namespace Planner {
	StateValidatorOccupancyMap::StateValidatorOccupancyMap(const Ref<PlanarStateSpace>& stateSpace, const Ref<OccupancyMap>& map) :
		PlanarStateValidator(stateSpace)
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
		GridCellPosition cell = m_map->WorldPositionToGridCell(state.position);
		if (!m_stateSpace->ValidateBounds(localState) || !m_map->IsInsideMap(cell))
			return false;
		// Check intersection with obstacles
		// TODO add offset
		float distance = m_map->GetObstacleDistanceMap()->GetDistanceToNearestObstacle(cell);
		return distance >= minSafeRadius;
	}

	bool StateValidatorOccupancyMap::IsPathValid(const PlanarPath& path, float* last)
	{
		PP_PROFILE_FUNCTION();

		const auto& bounds = m_stateSpace->bounds;
		const double pathLength = path.GetLength();
		if (pathLength == 0.0) {
			if (last)
				*last = 1.0f;
			return IsStateValid(path.GetInitialState());
		}

		double lastValidLength = 0.0;
		double length = 0.0;
		while (length < pathLength) {
			Pose2d state = path.Interpolate(length / pathLength);

			// Check current state
			if (!IsStateValid(state)) {
				if (last)
					*last = lastValidLength / pathLength;
				return false;
			}
			lastValidLength = length;

			// Update length
			float distToMapBorder = std::min({
				state.x() - bounds[0].x(),
				bounds[1].x() - state.x(),
				state.y() - bounds[0].y(),
				bounds[1].y() - state.y(),
			});
			// TODO add offset
			GridCellPosition cell = m_map->WorldPositionToGridCell(state.position);
			float distance = m_map->GetObstacleDistanceMap()->GetDistanceToNearestObstacle(cell);
			float deltaLength = distance - minSafeRadius;
			deltaLength = std::min(deltaLength, distToMapBorder);
			length += std::max(deltaLength, minPathInterpolationDistance);
		}

		if (last)
			*last = 1.0f;
		return true;
	}
}
