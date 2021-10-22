#include "core/base.h"
#include "state_validator/obstacle.h"
#include "state_validator/binary_occupancy_map.h"
#include "state_validator/gvd.h"

namespace Planner {

	BinaryOccupancyMap::BinaryOccupancyMap(float resolution) :
		OccupancyMap(resolution)
	{
	}

	bool BinaryOccupancyMap::AddObstacle(const Ref<Obstacle>& obstacle)
	{
		PP_ASSERT(m_occupancyMatrix, "Matrix not initialized");

		if (OccupancyMap::AddObstacle(obstacle)) {
			auto id = m_obstacles[obstacle];
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
		PP_ASSERT(m_occupancyMatrix, "Matrix not initialized");

		if (OccupancyMap::RemoveObstacle(obstacle)) {
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
		PP_ASSERT(m_occupancyMatrix, "Matrix not initialized");

		return (*m_occupancyMatrix)[cell.row][cell.col] >= 0;
	}
}
