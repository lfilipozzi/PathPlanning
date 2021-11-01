#pragma once

#include "utils/grid.h"
#include "state_validator/gvd.h"
#include "geometry/2dplane.h"

#include <unordered_map>
#include <set>

namespace Planner {
	class Obstacle;

	/// @brief 2D Occupancy map.
	/// @details This map has a grid representation of the workspace. Each cell
	/// of the grid has an integer associated to it. If the value is negative,
	/// the cell is free; if it is positive, the cell is occupied and its value
	/// gives a reference to the obstacle.
	/// Three coordinate frames are defined. The world frame correspond
	/// to the global frame. The local translates the global frame. The grid
	/// frame is translated from the local frame so that the point whose
	/// coordinate is (-width/2, -height/2) becomes the origin, where with and
	/// height are the width and height of the occupancy map.
	class OccupancyMap {
	public:
		OccupancyMap(float resolution) :
			resolution(resolution) { }
		virtual ~OccupancyMap() = default;

		/// @brief Initialize the size of the occupancy grid.
		void InitializeSize(float width, float height);
		/// @brief Return the number of rows in the occupancy grid.
		int Rows() const { return m_rows; }
		/// @brief Return the number of columns in the occupancy grid.
		int Columns() const { return m_columns; }

		/// @brief Update the occupancy map.
		virtual void Update();

		/// @brief Set the position of the bottom left corner of the map.
		void SetPosition(const Point2d& position) { m_localOrigin = position; }
		/// @brief Return the position of the bottom left corner of the map in
		/// the world coordinates.
		const Point2d& GetPosition() const { return m_localOrigin; }

		/// @brief Check if a cell is occupied
		virtual bool IsOccupied(const GridCellPosition& cell) = 0;
		/// @brief Return the value of the occupancy matrix.
		int GetOccupancyValue(const GridCellPosition& cell) { return (*m_occupancyMatrix)[cell.row][cell.col]; }
		/// @overload
		int GetOccupancyValue(int row, int col) { return (*m_occupancyMatrix)[row][col]; }

		/// @brief Return the map of distance to obstacle
		Ref<GVD::ObstacleDistanceMap>& GetObstacleDistanceMap() { return m_obstacleDistanceMap; }
		/// @brief Return the map of distance to obstacle
		const Ref<GVD::ObstacleDistanceMap>& GetObstacleDistanceMap() const { return m_obstacleDistanceMap; }

		/// @brief Convert grid coordinate to local position
		Point2d GridCellToLocalPosition(const GridCellPosition& position) const;
		/// @overload
		Point2d GridCellToLocalPosition(int row, int column) const;
		/// @brief Convert grid cell coordinates to world coordinates
		Point2d GridCellToWorldPosition(const GridCellPosition& position) const;
		/// @overload
		Point2d GridCellToWorldPosition(int row, int column) const;

		/// @brief Convert local coordinate to grid position
		GridCellPosition LocalPositionToGridCell(const Point2d& position, bool bounded = true) const;
		/// @overload
		GridCellPosition LocalPositionToGridCell(double x, double y, bool bounded = true) const;
		/// @brief Convert local position to world coordinate
		Point2d LocalPositionToWorldPosition(const Point2d& position) const;
		/// @overload
		Point2d LocalPositionToWorldPosition(double x, double y) const;

		/// @brief Convert world position  to grid cell coordinate
		GridCellPosition WorldPositionToGridCell(const Point2d& position, bool bounded = true) const;
		/// @overload
		GridCellPosition WorldPositionToGridCell(double x, double y, bool bounded = true) const;
		/// @brief Convert World coordinate to local map coordinate
		Point2d WorldPositionToLocalPosition(const Point2d& position) const;
		/// @overload
		Point2d WorldPositionToLocalPosition(double x, double y) const;

		/// @brief Check if the point is inside the occupancy map
		/// @param point The point in world coordinate
		bool IsInsideMap(const Point2d& position) const;
		/// @overload
		bool IsInsideMap(const GridCellPosition& cell) const;

	private:
		inline Point2d GridCellToGridPosition(const GridCellPosition& position) const
		{
			return { position.row * resolution, position.col * resolution };
		}

		inline GridCellPosition GridPositionToGridCell(const Point2d& position, bool bounded) const
		{
			int row = static_cast<int>(position.x() / resolution);
			int col = static_cast<int>(position.y() / resolution);

			if (!bounded)
				return GridCellPosition(row, col);
			else if (IsInsideMap(GridCellPosition(row, col)))
				return GridCellPosition(row, col);
			else
				return GridCellPosition(-1, -1);
		}

	public:
		const float resolution;

	protected:
		// Map size
		int m_rows = -1, m_columns = -1;
		// Position of the origin of the local frame in the world frame
		Point2d m_localOrigin = { 0.0, 0.0 };
		// Position of the origin of the grid frame in the local frame
		Point2d m_gridOrigin;
		// Matrix of cell occupancy (positive or zero value means occupied)
		Ref<Grid<int>> m_occupancyMatrix;
		// Distance map to nearest obstacle
		Ref<GVD::ObstacleDistanceMap> m_obstacleDistanceMap;
	};
}
