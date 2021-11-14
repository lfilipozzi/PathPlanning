#include "core/base.h"
#include "state_validator/obstacle.h"
#include "state_validator/obstacle_list_occupancy_map.h"
#include "state_validator/gvd.h"

namespace Planner {

	ObstacleListOccupancyMap::ObstacleListOccupancyMap(float resolution) :
		OccupancyMap(resolution)
	{
	}

	unsigned int FindSmallestIDAvailable(const std::set<unsigned int>& IDs)
	{
		constexpr int smallestAllowedID = 0;
		if (IDs.empty())
			return smallestAllowedID;
		if (*IDs.begin() > smallestAllowedID)
			return smallestAllowedID;
		else {
			auto res = std::adjacent_find(IDs.begin(), IDs.end(), [](unsigned int a, unsigned int b) { return a + 1 != b; });
			if (res == IDs.end())
				return *IDs.rbegin() + 1;
			else
				return *res + 1;
		}
	}

	bool ObstacleListOccupancyMap::AddObstacle(const Ref<Obstacle>& obstacle)
	{
		if (!m_occupancyMatrix)
			throw std::runtime_error("The size of the occupancy matrix has not been initialized");

		auto id = FindSmallestIDAvailable(m_obstacleIDs);
		if (m_obstacles.insert({ obstacle, id }).second) {
			m_obstacleIDs.insert(id);
			for (auto& cell : obstacle->GetBoundaryGridCellPosition(*this)) {
				(*m_occupancyMatrix)[cell.row][cell.col] = id;
				m_obstacleDistanceMap->SetObstacle(cell);
			}
			return true;
		}
		return false;
	}

	bool ObstacleListOccupancyMap::RemoveObstacle(const Ref<Obstacle>& obstacle)
	{
		if (!m_occupancyMatrix)
			throw std::runtime_error("The size of the occupancy matrix has not been initialized");

		auto it = m_obstacles.find(obstacle);
		if (it == m_obstacles.end())
			return false;
		m_obstacleIDs.erase(it->second);
		m_obstacles.erase(it);
		for (auto& cell : obstacle->GetBoundaryGridCellPosition(*this)) {
			(*m_occupancyMatrix)[cell.row][cell.col] = -1;
			m_obstacleDistanceMap->UnsetObstacle(cell);
		}
		return true;
	}

	bool ObstacleListOccupancyMap::IsOccupied(const GridCellPosition& cell)
	{
		if (!m_occupancyMatrix)
			throw std::runtime_error("The size of the occupancy matrix has not been initialized");

		return (*m_occupancyMatrix)[cell.row][cell.col] >= 0;
	}
}
