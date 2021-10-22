#include "core/base.h"
#include "state_validator/obstacle.h"
#include "state_validator/binary_occupancy_map.h"
#include "state_validator/gvd.h"

namespace Planner {

	BinaryOccupancyMap::BinaryOccupancyMap(float width, float height, float resolution) :
		OccupancyMap(width, height, resolution)
	{
	}

	bool BinaryOccupancyMap::AddObstacle(const Ref<Obstacle>& obstacle)
	{
		if (m_obstacles.insert(obstacle).second) {
			int id = m_obstacles.hash_function()(obstacle);
			for (auto& cell : obstacle->GetGridCellPositions(*this)) {
				(*m_occupancyMatrix)[cell.row][cell.col] = id;
				m_obstacleDistanceMap->SetObstacle(cell);
			}
			return true;
		}
		return false;
	}

	bool BinaryOccupancyMap::RemoveObstacle(const Ref<Obstacle>& obstacle)
	{
		if (m_obstacles.erase(obstacle) == 1) {
			for (auto& cell : obstacle->GetGridCellPositions(*this)) {
				(*m_occupancyMatrix)[cell.row][cell.col] = -1;
				m_obstacleDistanceMap->UnsetObstacle(cell);
			}
			return true;
		}
		return false;
	}

	bool BinaryOccupancyMap::IsOccupied(const GridCellPosition& cell)
	{
		return (*m_occupancyMatrix)[cell.row][cell.col] >= 0;
	}
}
