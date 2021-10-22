#include "core/base.h"
#include "state_validator/occupancy_map.h"

namespace Planner {

	OccupancyMap::OccupancyMap(float width, float height, float resolution) :
		resolution(resolution), rows(ceil(width / resolution)), columns(ceil(height / resolution)),
		width(width), height(height)
	{
		
		m_occupancyMatrix = makeRef<Grid<int>>(rows, columns, -1);
		m_obstacleDistanceMap = makeRef<GVD::ObstacleDistanceMap>(m_occupancyMatrix, resolution);
	}

	void OccupancyMap::Update()
	{
		m_obstacleDistanceMap->Update();
	}

	Point2d OccupancyMap::GridToLocalPosition(const GridCellPosition& position) const
	{
		double x = position.row * resolution;
		double y = position.col * resolution;
		return { x, y };
	}

	Point2d OccupancyMap::GridToLocalPosition(int row, int column) const
	{
		return GridToLocalPosition({ row, column });
	}

	Point2d OccupancyMap::GridToWorldPosition(const GridCellPosition& position) const
	{
		double x = m_origin.x() + position.row * resolution;
		double y = m_origin.y() + position.col * resolution;
		return { x, y };
	}

	Point2d OccupancyMap::GridToWorldPosition(int row, int column) const
	{
		return GridToWorldPosition({ row, column });
	}

	GridCellPosition OccupancyMap::LocalToGridPosition(const Point2d& position, bool bounded) const
	{
		int row = static_cast<int>(position.x() / resolution);
		int col = static_cast<int>(position.y() / resolution);

		if (!bounded || (row >= 0 && row < rows && col >= 0 && col < columns))
			return GridCellPosition(row, col);
		else
			return GridCellPosition(-1, -1);
	}

	GridCellPosition OccupancyMap::LocalToGridPosition(double x, double y, bool bounded) const
	{
		return LocalToGridPosition({ x, y }, bounded);
	}

	Point2d OccupancyMap::LocalToWorldPosition(const Point2d& position) const
	{
		double x = m_origin.x() + position.x();
		double y = m_origin.y() + position.y();
		return { x, y };
	}

	Point2d OccupancyMap::LocalToWorldPosition(double x, double y) const
	{
		return LocalToWorldPosition({ x, y });
	}

	GridCellPosition OccupancyMap::WorldToGridPosition(const Point2d& point, bool bounded) const
	{
		int row = static_cast<int>((point.x() - m_origin.x()) / resolution);
		int col = static_cast<int>((point.y() - m_origin.y()) / resolution);

		if (!bounded || (row >= 0 && row < rows && col >= 0 && col < columns))
			return GridCellPosition(row, col);
		else
			return GridCellPosition(-1, -1);
	}

	GridCellPosition OccupancyMap::WorldToGridPosition(double x, double y, bool bounded) const
	{
		return WorldToGridPosition({ x, y }, bounded);
	}

	Point2d OccupancyMap::WorldToLocalPosition(const Point2d& position) const
	{
		double x = position.x() - m_origin.x();
		double y = position.y() - m_origin.y();
		return { x, y };
	}

	Point2d OccupancyMap::WorldToLocalPosition(double x, double y) const
	{
		return WorldToLocalPosition({ x, y });
	}
}
