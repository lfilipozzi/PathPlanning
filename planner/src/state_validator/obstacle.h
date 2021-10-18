#pragma once

#include "core/base.h"
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
		
		void Add(const Ref<Shape>& shape) { m_children.push_back(shape); }
		/// @copydoc Planner::Shape::GetGridCellPositions
		virtual void GetGridCellPositions(const ObstacleGrid& grid, const Pose2d pose, std::vector<GridCellPosition>& cells) override;

	private:
		std::vector<Ref<Shape>> m_children;
	};

	class PolygonShape : public Shape {
	public:
		PolygonShape() = default;

		/// @copydoc Planner::Shape::GetGridCellPositions
		virtual void GetGridCellPositions(const ObstacleGrid& grid, const Pose2d pose, std::vector<GridCellPosition>& cells) override final;

	protected:
		std::vector<Point2d> m_vertices;
	};

	class RectangleShape : public PolygonShape {
	public:
		RectangleShape(double dx, double dy);
};

	class CircleShape : public PolygonShape {
	public:
		CircleShape(double radius, int count);
	};

	class Obstacle {
	public:
		Obstacle() = default;
		~Obstacle() = default;

		void SetShape(const Ref<Shape>& shape) { m_shape = shape; }
		void SetPose(const Pose2d& pose) { m_pose = pose; }
		std::vector<GridCellPosition> GetGridCellPositions(const ObstacleGrid& grid);

	private:
		Ref<Shape> m_shape;
		Pose2d m_pose;
	};
}
