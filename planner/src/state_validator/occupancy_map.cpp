#include "core/base.h"
#include "state_validator/occupancy_map.h"

namespace Planner {

	void OccupancyMap::InitializeSize(float width, float height)
	{
		m_localGridOrigin = { -width / 2.0, -height / 2.0 };
		m_worldGridOrigin = m_localOrigin + m_localGridOrigin;
		m_rows = ceil(width / resolution);
		m_columns = ceil(height / resolution);
		m_occupancyMatrix = makeRef<Grid<int>>(m_rows, m_columns, -1);
		m_obstacleDistanceMap = makeRef<GVD::ObstacleDistanceMap>(m_occupancyMatrix, resolution);
	}

	void OccupancyMap::Update()
	{
		m_obstacleDistanceMap->Update();
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
