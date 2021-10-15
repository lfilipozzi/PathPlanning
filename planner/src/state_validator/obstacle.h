#pragma once

#include <vector>

namespace Planner {
	class ObstacleGrid;
	struct GridCellPosition;

	class Obstacle {
	public:
		Obstacle() = default;
		virtual ~Obstacle() = default;

		virtual std::vector<GridCellPosition> GetGridCellPositions(const ObstacleGrid& grid) = 0;
	};
}
