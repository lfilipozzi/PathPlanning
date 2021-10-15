#pragma once

#include "geometry/2dplane.h"

#include <vector>

namespace Planner {
	class ObstacleGrid;
	struct GridCellPosition;

	class Shape {
	public:
		Shape() = default;
		virtual ~Shape() = default;

		/// @brief Compute grid cells occupied by obstacle
		/// @param[in] grid The grid.
		/// @param[in] pose The obstacle pose.
		/// @param[out] cells The cell positions.
		virtual void GetGridCellPositions(const ObstacleGrid& grid, const Pose2d pose, std::vector<GridCellPosition>& cells) = 0;

	protected:
		void RasterizeLine(const ObstacleGrid& grid, const Point2d& p0, const Point2d& p1, std::vector<GridCellPosition>& line) const;
	};

	class CompositeShape : public Shape {
	public:
		CompositeShape() = default;
		
		/// @copydoc Planner::Shape::GetGridCellPositions
		void Add(const Ref<Shape>& shape) { m_children.push_back(shape); }
		virtual void GetGridCellPositions(const ObstacleGrid& grid, const Pose2d pose, std::vector<GridCellPosition>& cells) override;

	private:
		std::vector<Ref<Shape>> m_children;
	};

	class RectangleShape : public Shape {
	public:
		RectangleShape(double width, double length);

		/// @copydoc Planner::Shape::GetGridCellPositions
		virtual void GetGridCellPositions(const ObstacleGrid& grid, const Pose2d pose, std::vector<GridCellPosition>& cells) override;

	private:
		std::array<Point2d, 4> m_vertices;
	};

	class Obstacle {
	public:
		Obstacle() = default;
		~Obstacle() = default;

		std::vector<GridCellPosition> GetGridCellPositions(const ObstacleGrid& grid);

	private:
		Ref<Shape> m_shape;
		Pose2d m_position;
	};
}
