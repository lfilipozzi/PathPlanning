#include "state_validator/obstacle.h"
#include "core/base.h"
#include "utils/grid.h"
#include "state_validator/occupancy_map.h"

namespace Planner {
	void Shape::RasterizeLine(const OccupancyMap& map, const Point2d& p0, const Point2d& p1, std::vector<GridCellPosition>& line) const
	{
		// Find cells of the endpoints
		auto p0Cell = map.WorldPositionToGridCell(p0, false);
		auto p1Cell = map.WorldPositionToGridCell(p1, false);

		int x0 = p0Cell.row;
		int y0 = p0Cell.col;
		int x1 = p1Cell.row;
		int y1 = p1Cell.col;

		// Bresenham's Line Algorithm
		bool steep = abs(y1 - y0) > abs(x1 - x0);
		if (steep) {
			// Swap x and y coordinates
			int temp = x0;
			x0 = y0;
			y0 = temp;

			temp = x1;
			x1 = y1;
			y1 = temp;
		}

		if (x0 > x1) {
			// Swap endpoints
			int temp = x0;
			x0 = x1;
			x1 = temp;

			temp = y0;
			y0 = y1;
			y1 = temp;
		}

		int dx = x1 - x0;
		int dy = abs(y1 - y0);
		int err = dx / 2;
		int ystep = y0 < y1 ? 1 : -1;
		int y = y0;

		line.reserve(line.size() + (x1 - x0));
		for (int x = x0; x <= x1; x++) {
			GridCellPosition c = steep ? GridCellPosition(y, x) : GridCellPosition(x, y);

			if (c.row >= 0 && c.row < map.Rows() && c.col >= 0 && c.col < map.Columns())
				line.push_back(c);

			err = err - dy;
			if (err < 0) {
				y += ystep;
				err += dx;
			}
		}
	}

	void CompositeShape::GetGridCellPositions(const OccupancyMap& map, const Pose2d& pose, std::vector<GridCellPosition>& cells)
	{
		for (auto& child : m_children) {
			child->GetGridCellPositions(map, pose, cells);
		}
	}

	void PolygonShape::GetGridCellPositions(const OccupancyMap& map, const Pose2d& pose, std::vector<GridCellPosition>& cells)
	{
		// Compute world position of vertices
		std::vector<Point2d> vertices;
		vertices.reserve(m_vertices.size());
		for (int i = 0; i < m_vertices.size(); i++) {
			vertices[i] = Eigen::Rotation2D(pose.theta) * m_vertices[i] + pose.position;
		}

		for (int i = 0; i < m_vertices.size(); i++)
			RasterizeLine(map, vertices[i % m_vertices.size()], vertices[(i + 1) % m_vertices.size()], cells);
	}

	RegularPolygonShape::RegularPolygonShape(double radius, int count)
	{
		m_vertices.reserve(count);
		for (int i = 0; i < count; i++) {
			m_vertices.push_back({ radius * cos(2 * M_PI * i / (float)count),
				radius * sin(2 * M_PI * i / (float)count) });
		}
	}

	RectangleShape::RectangleShape(double dx, double dy)
	{
		double dx2 = dx / 2.0;
		double dy2 = dy / 2.0;
		m_vertices = { Point2d(dx2, dy2), Point2d(-dx2, dy2), Point2d(-dx2, -dy2), Point2d(dx2, -dy2) };
	}

	CircleShape::CircleShape(double radius, int count)
	{
		// Inflate radius so that the polygon contains the circle
		radius *= 1.0 / cos(M_PI / count);

		m_vertices.reserve(count);
		for (int i = 0; i < count; i++) {
			m_vertices.push_back({ radius * cos(2 * M_PI * i / (float)count),
				radius * sin(2 * M_PI * i / (float)count) });
		}
	}

	std::vector<GridCellPosition> Obstacle::GetGridCellPositions(const OccupancyMap& map)
	{
		std::vector<GridCellPosition> cells;
		m_shape->GetGridCellPositions(map, m_pose, cells);
		return cells;
	}
}
