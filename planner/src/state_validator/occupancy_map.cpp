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

	bool OccupancyMap::IsInsideMap(const Point2d& position) const
	{
		auto cell = WorldPositionToGridCell(position, true);
		return IsInsideMap(cell);
	}

	bool OccupancyMap::IsInsideMap(const GridCellPosition& cell) const
	{
		return cell.row >= 0 && cell.row < m_rows && cell.col >= 0 && cell.col < m_columns;
	}
}
