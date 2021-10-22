#pragma once

#include "utils/grid.h"
#include "state_validator/gvd.h"
#include "geometry/2dplane.h"

#include <unordered_map>
#include <set>

namespace Planner {
	class Obstacle;

	/// @brief 2D Occupancy map.
	class OccupancyMap {
	public:
		OccupancyMap(float width, float height, float resolution);
		virtual ~OccupancyMap() = default;

		/// @brief Update the occupancy map. Call this method after
		/// updating (i.e. adding, removing, or modifying) the obstacles or
		/// after moving the map.
		void Update();

		/// @brief Set the position of the bottom left corner of the map.
		void SetOriginPosition(const Point2d& position) { m_origin = position; }
		const Point2d& GetOriginPosition() const { return m_origin; }

		/// @brief Add the obstacle to the grid.
		virtual bool AddObstacle(const Ref<Obstacle>& obstacle);
		/// @brief Remove the obstacle from the grid.
		virtual bool RemoveObstacle(const Ref<Obstacle>& obstacle);
		/// @brief Return the number of obstacle.
		int GetNumObstacles() const { return m_obstacles.size(); }

		/// @brief Check if a cell is occupied
		virtual bool IsOccupied(const GridCellPosition& cell) = 0;
		/// @brief Return the value of the occupancy matrix.
		int GetOccupancyValue(const GridCellPosition& cell) { return (*m_occupancyMatrix)[cell.row][cell.col]; }

		/// @brief Return the map of distance to obstacle
		Ref<GVD::ObstacleDistanceMap> GetObstacleDistanceMap() { return m_obstacleDistanceMap; }
		/// @brief Return the map of distance to obstacle
		const Ref<GVD::ObstacleDistanceMap> GetObstacleDistanceMap() const { return m_obstacleDistanceMap; }

		/// @brief Convert grid coordinate to local position
		Point2d GridToLocalPosition(const GridCellPosition& position) const;
		/// @overload
		Point2d GridToLocalPosition(int row, int column) const;
		/// @brief Convert grid cell coordinates to world coordinates
		Point2d GridToWorldPosition(const GridCellPosition& position) const;
		/// @overload
		Point2d GridToWorldPosition(int row, int column) const;

		/// @brief Convert local coordinate to grid position
		GridCellPosition LocalToGridPosition(const Point2d& position, bool bounded = true) const;
		/// @overload
		GridCellPosition LocalToGridPosition(double x, double y, bool bounded = true) const;
		/// @brief Convert local position to world coordinate
		Point2d LocalToWorldPosition(const Point2d& position) const;
		/// @overload
		Point2d LocalToWorldPosition(double x, double y) const;

		/// @brief Convert world position  to grid cell coordinate
		GridCellPosition WorldToGridPosition(const Point2d& position, bool bounded = true) const;
		/// @overload
		GridCellPosition WorldToGridPosition(double x, double y, bool bounded = true) const;
		/// @brief Convert World coordinate to local map coordinate
		Point2d WorldToLocalPosition(const Point2d& position) const;
		/// @overload
		Point2d WorldToLocalPosition(double x, double y) const;

	public:
		const float resolution;
		const int rows, columns;
		const float width, height;

	protected:
		Point2d m_origin = { 0.0, 0.0 };
		std::unordered_map<Ref<Obstacle>, unsigned int> m_obstacles;
		std::set<unsigned int> m_obstacleIDs;
		Ref<Grid<int>> m_occupancyMatrix;
		Ref<GVD::ObstacleDistanceMap> m_obstacleDistanceMap;
	};
}
