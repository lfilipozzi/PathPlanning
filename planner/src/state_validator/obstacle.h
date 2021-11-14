#pragma once

#include "core/base.h"
#include "geometry/2dplane.h"

#include <vector>

namespace Planner {
	class OccupancyMap;
	struct GridCellPosition;

	class Shape {
	public:
		Shape() = default;
		virtual ~Shape() = default;

		/// @brief Compute grid cells occupied by obstacle
		/// @param[in] map The occupancy map.
		/// @param[in] pose The obstacle pose.
		/// @param[out] cells The cell positions.
		virtual void GetGridCellsPosition(const OccupancyMap& map, const Pose2d& pose, std::vector<GridCellPosition>& cells) = 0;

		/// @brief Compute position occupied by obstacle
		/// @param[in] pose The obstacle pose.
		/// @param[out] points The position of the vertices.
		virtual void GetVerticesPosition(const Pose2d& pose, std::vector<Point2d>& points) = 0;

	protected:
		void RasterizeLine(const OccupancyMap& map, const Point2d& p0, const Point2d& p1, std::vector<GridCellPosition>& line) const;
	};

	class CompositeShape : public Shape {
	public:
		CompositeShape() = default;

		void Add(const Ref<Shape>& shape) { m_children.push_back(shape); }

		/// @copydoc Planner::Shape::GetGridCellsPosition
		virtual void GetGridCellsPosition(const OccupancyMap& map, const Pose2d& pose, std::vector<GridCellPosition>& cells) override;

		/// @copydoc Planner::Shape::GetVerticesPosition
		virtual void GetVerticesPosition(const Pose2d& pose, std::vector<Point2d>& points) override;

	private:
		std::vector<Ref<Shape>> m_children;
	};

	class PolygonShape : public Shape {
	public:
		PolygonShape(const std::vector<Point2d>& vertices) { m_vertices = vertices; }
		PolygonShape(std::vector<Point2d>&& vertices) { m_vertices = vertices; }

		/// @copydoc Planner::Shape::GetGridCellsPosition
		virtual void GetGridCellsPosition(const OccupancyMap& grid, const Pose2d& pose, std::vector<GridCellPosition>& cells) override final;

		/// @copydoc Planner::Shape::GetVerticesPosition
		virtual void GetVerticesPosition(const Pose2d& pose, std::vector<Point2d>& points) override;

	protected:
		PolygonShape() = default;

	protected:
		std::vector<Point2d> m_vertices;
	};

	class RegularPolygonShape : public PolygonShape {
	public:
		/// @param radius The radius of the circle containing the polygon
		/// @param count The number of verticces in the polygon
		RegularPolygonShape(double radius, int count);
	};

	class RectangleShape : public PolygonShape {
	public:
		/// @param dx The width
		/// @param dy The length
		RectangleShape(double dx, double dy);
	};

	class CircleShape : public PolygonShape {
	public:
		/// @details The circle is approximated by a polygon
		/// @param radius The radius of the circle contained in the polygon
		/// @param count The number of vertices used to approximate the circle
		CircleShape(double radius, int count);
	};

	class Obstacle {
	public:
		Obstacle() = default;
		~Obstacle() = default;

		void SetShape(const Ref<Shape>& shape) { m_shape = shape; }
		void SetPose(const Pose2d& pose) { m_pose = pose; }
		std::vector<GridCellPosition> GetBoundaryGridCellPosition(const OccupancyMap& map);
		std::vector<Point2d> GetBoundaryWorldPosition();

	private:
		Ref<Shape> m_shape;
		Pose2d m_pose;
	};
}
