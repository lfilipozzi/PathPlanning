#pragma once

#include "core/base.h"
#include "state_validator/occupancy_map.h"

namespace Planner {
	/// @brief 2D occupancy map from a list of obstacle.
	/// @details The occupancy of the cell is defined by the presence of an
	/// obstacles. Obstacles can be added to the map with the AddObstacle and
	/// RemoveObstacle methods.
	class ObstacleListOccupancyMap : public OccupancyMap {
	public:
		ObstacleListOccupancyMap(float resolution);

		/// @brief Add the obstacle to the grid.
		bool AddObstacle(const Ref<Obstacle>& obstacle);
		/// @brief Remove the obstacle from the grid.
		bool RemoveObstacle(const Ref<Obstacle>& obstacle);
		/// @brief Return the number of obstacle.
		int GetNumObstacles() const { return m_obstacles.size(); }

		/// @copydoc Planner::OccupancyMap::IsOccupied
		virtual bool IsOccupied(const GridCellPosition& cell) override;

	private:
		std::unordered_map<Ref<Obstacle>, unsigned int> m_obstacles;
		std::set<unsigned int> m_obstacleIDs;
	};
}
