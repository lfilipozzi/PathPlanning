#pragma once

#include "core/base.h"
#include "core/hash.h"
#include "state_validator/obstacle.h"

#include <vector>
#include <unordered_set>
#include <cmath>

namespace Planner {

	/// @brief Cell location in a grid.
	struct GridCellPosition {
		GridCellPosition() = default;
		GridCellPosition(int row, int col) :
			row(row), col(col) { }

		int row = -1;
		int col = -1;

		bool IsValid() const { return row >= 0 && col >= 0; }

		std::vector<GridCellPosition> GetNeighbors(int rows, int columns) const;

		bool IsAdjacentTo(const GridCellPosition& rhs) const;
	};

	bool operator==(const GridCellPosition& lhs, const GridCellPosition& rhs)
	{
		return lhs.row == rhs.row && lhs.col == rhs.col;
	}
	bool operator!=(const GridCellPosition& lhs, const GridCellPosition& rhs)
	{
		return !(lhs == rhs);
	}
}

namespace std {
	template <>
	struct hash<Planner::GridCellPosition> {
		std::size_t operator()(const Planner::GridCellPosition& pose) const
		{
			std::size_t seed = 0;
			HashCombine(seed, pose.row);
			HashCombine(seed, pose.col);
			return seed;
		}
	};
}

namespace Planner {
	/// @brief Define a grid cell with a value.
	template <typename T>
	struct GridCell {
		GridCell(const GridCellPosition& position, T value) :
			position(position), value(value) { }

		GridCellPosition position;
		T value;
	};

	template <typename T>
	bool operator<(const GridCell<T>& lhs, const GridCell<T>& rhs)
	{
		return lhs.value < rhs.value;
	}

	template <typename T>
	bool operator>(const GridCell<T>& lhs, const GridCell<T>& rhs)
	{
		return lhs.value > rhs.value;
	}

	class GVD;

	/// @brief
	class ObstacleGrid {
	public:
		ObstacleGrid(float resolution, float lowerX, float lowerY, float width, float height);

		/// @brief Add the obstacle to the grid.
		void AddObstacle(const Ref<Obstacle>& obstacle);

		/// @brief Remove the obstacle from the grid.
		void RemoveObstacle(const Ref<Obstacle>& obstacle);

		/// @brief Return the cell position associated to the position.
		GridCellPosition PointToCellPosition(const Point2d& point, bool bounded = true) const;
		/// @overload
		GridCellPosition PointToCellPosition(double x, double y, bool bounded = true) const;

		/// @brief Return the position associated to the center of the cell position.
		Point2d CellPositionToPoint(const GridCellPosition& position);

	public:
		const float resolution;
		const int rows, columns;
		const float width, height;
		const float lowerX, lowerY;
		const float upperX, upperY;

	private:
		Scope<GVD> m_gvd;
		std::unordered_set<Ref<Obstacle>> m_obstacles;
	};
}
