#pragma once

#include "core/base.h"
#include "state_validator/occupancy_map.h"

namespace Planner {
	/// @brief 2D binary occupancy map.
	/// @details This map has a grid representation of the workspace. Each cell
	/// of the grid has an integer associated to it. If the value is negative,
	/// the cell is free; if it is positive, the cell is occupied and its value
	/// gives a reference to the obstacle.
	class BinaryOccupancyMap : public OccupancyMap {
	public:
		BinaryOccupancyMap(float resolution);

		/// @copydoc Planner::OccupancyMap::AddObstacle
		virtual bool AddObstacle(const Ref<Obstacle>& obstacle) override;
		/// @copydoc Planner::OccupancyMap::RemoveObstacle
		virtual bool RemoveObstacle(const Ref<Obstacle>& obstacle) override;

		/// @copydoc Planner::OccupancyMap::IsOccupied
		virtual bool IsOccupied(const GridCellPosition& cell) override;
	};
}
