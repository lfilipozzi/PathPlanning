#include "core/base.h"
#include "state_validator/occupancy_map.h"

namespace Planner {

	void OccupancyMap::InitializeSize(float width, float height)
	{
		m_gridOrigin = { -width / 2.0, -height / 2.0 };
		m_rows = ceil(width / resolution);
		m_columns = ceil(height / resolution);
		m_occupancyMatrix = makeRef<Grid<int>>(m_rows, m_columns, -1);
		m_obstacleDistanceMap = makeRef<GVD::ObstacleDistanceMap>(m_occupancyMatrix, resolution);
	}

	void OccupancyMap::Update()
	{
		m_obstacleDistanceMap->Update();
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

	bool OccupancyMap::AddObstacle(const Ref<Obstacle>& obstacle)
	{
		auto id = FindSmallestIDAvailable(m_obstacleIDs);
		if (m_obstacles.insert({ obstacle, id }).second) {
			m_obstacleIDs.insert(id);
			return true;
		}
		return false;
	}

	bool OccupancyMap::RemoveObstacle(const Ref<Obstacle>& obstacle)
	{
		auto it = m_obstacles.find(obstacle);
		if (it == m_obstacles.end())
			return false;
		m_obstacleIDs.erase(it->second);
		m_obstacles.erase(it);
		return true;
	}

	Point2d OccupancyMap::GridCellToLocalPosition(const GridCellPosition& position) const
	{
		return m_gridOrigin + GridCellToGridPosition(position);
	}

	Point2d OccupancyMap::GridCellToLocalPosition(int row, int column) const
	{
		return GridCellToLocalPosition({ row, column });
	}

	Point2d OccupancyMap::GridCellToWorldPosition(const GridCellPosition& position) const
	{
		return m_localOrigin + m_gridOrigin + GridCellToGridPosition(position);
	}

	Point2d OccupancyMap::GridCellToWorldPosition(int row, int column) const
	{
		return GridCellToWorldPosition({ row, column });
	}

	GridCellPosition OccupancyMap::LocalPositionToGridCell(const Point2d& position, bool bounded) const
	{
		return GridPositionToGridCell(position - m_gridOrigin, bounded);
	}

	GridCellPosition OccupancyMap::LocalPositionToGridCell(double x, double y, bool bounded) const
	{
		return LocalPositionToGridCell({ x, y }, bounded);
	}

	Point2d OccupancyMap::LocalPositionToWorldPosition(const Point2d& position) const
	{
		return m_localOrigin + position;
	}

	Point2d OccupancyMap::LocalPositionToWorldPosition(double x, double y) const
	{
		return LocalPositionToWorldPosition({ x, y });
	}

	GridCellPosition OccupancyMap::WorldPositionToGridCell(const Point2d& position, bool bounded) const
	{
		return GridPositionToGridCell(position - m_gridOrigin - m_localOrigin, bounded);
	}

	GridCellPosition OccupancyMap::WorldPositionToGridCell(double x, double y, bool bounded) const
	{
		return WorldPositionToGridCell({ x, y }, bounded);
	}

	Point2d OccupancyMap::WorldPositionToLocalPosition(const Point2d& position) const
	{
		return position - m_localOrigin;
	}

	Point2d OccupancyMap::WorldPositionToLocalPosition(double x, double y) const
	{
		return WorldPositionToLocalPosition({ x, y });
	}
}
