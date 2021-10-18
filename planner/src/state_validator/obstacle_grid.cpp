#include "core/base.h"
#include "state_validator/obstacle.h"
#include "state_validator/obstacle_grid.h"
#include "state_validator/gvd.h"

namespace Planner {
	bool operator==(const GridCellPosition& lhs, const GridCellPosition& rhs)
	{
		return lhs.row == rhs.row && lhs.col == rhs.col;
	}

	bool operator!=(const GridCellPosition& lhs, const GridCellPosition& rhs)
	{
		return !(lhs == rhs);
	}

	std::vector<GridCellPosition> GridCellPosition::GetNeighbors(int rows, int columns) const
	{
		std::vector<GridCellPosition> neighbors;
		if (!IsValid())
			return neighbors;

		neighbors.reserve(8);
		const int& c = col;
		const int& r = row;
		bool left = c - 1 >= 0;
		bool right = c + 1 < columns;
		bool bottom = r - 1 >= 0;
		bool top = r + 1 < rows;
		if (left) {
			neighbors.push_back({ r, c - 1 });
			if (bottom)
				neighbors.push_back({ r - 1, c - 1 });
			if (top)
				neighbors.push_back({ r + 1, c - 1 });
		}
		if (right) {
			neighbors.push_back({ r, c + 1 });
			if (bottom)
				neighbors.push_back({ r - 1, c + 1 });
			if (top)
				neighbors.push_back({ r + 1, c + 1 });
		}

		if (bottom)
			neighbors.push_back({ r - 1, c });
		if (top)
			neighbors.push_back({ r + 1, c });

		return neighbors;
	}

	bool GridCellPosition::IsAdjacentTo(const GridCellPosition& rhs) const
	{
		if (!IsValid() || !rhs.IsValid())
			return false;

		if (*this == rhs)
			return false;

		int deltaRow = row - rhs.row;
		int deltaCol = col - rhs.col;
		return (deltaRow >= -1 && deltaRow <= 1) && (deltaCol >= -1 && deltaCol <= 1);
	}

	ObstacleGrid::ObstacleGrid(float resolution, float lowerX, float lowerY, float width, float height) :
		resolution(resolution), rows(ceil(width / resolution)), columns(ceil(height / resolution)),
		width(width), height(height),
		lowerX(lowerX), lowerY(lowerY), upperX(lowerX + width), upperY(lowerY + height)
	{
		m_gvd = makeScope<GVD>(*this);
	}

	void ObstacleGrid::AddObstacle(const Ref<Obstacle>& obstacle)
	{
		if (m_obstacles.insert(obstacle).second) {
			int id = m_obstacles.hash_function()(obstacle);
			for (auto& cell : obstacle->GetGridCellPositions(*this)) {
				m_gvd->SetObstacle(cell, id);
			}
		}
	}

	void ObstacleGrid::RemoveObstacle(const Ref<Obstacle>& obstacle)
	{
		if (m_obstacles.erase(obstacle) == 1) {
			for (auto& cell : obstacle->GetGridCellPositions(*this)) {
				m_gvd->UnsetObstacle(cell);
			}
		}
	}

	GridCellPosition ObstacleGrid::PointToCellPosition(const Point2d& point, bool bounded) const
	{
		int row = static_cast<int>((point.x() - lowerX) / resolution);
		int col = static_cast<int>((point.y() - lowerY) / resolution);

		if (!bounded)
			return GridCellPosition(row, col);

		if (row >= 0 && row < rows && col >= 0 && col < columns)
			return GridCellPosition(row, col);
		else
			return GridCellPosition(-1, -1);
	}

	GridCellPosition ObstacleGrid::PointToCellPosition(double x, double y, bool bounded) const
	{
		return PointToCellPosition(Point2d(x, y), bounded);
	}

	Point2d ObstacleGrid::CellPositionToPoint(const GridCellPosition& position)
	{
		double x = lowerX + position.row * resolution;
		double y = lowerY + position.col * resolution;
		return { x, y };
	}

	void ObstacleGrid::Update()
	{
		m_gvd->Update();
	}
	
	GVD& ObstacleGrid::GetGVD() { return *m_gvd; }
	const GVD& ObstacleGrid::GetGVD() const { return *m_gvd; }
}
