#include "core/base.h"
#include "utils/grid.h"

namespace Planner {

	bool operator==(const GridCellPosition& lhs, const GridCellPosition& rhs)
	{
		return lhs.row == rhs.row && lhs.col == rhs.col;
	}

	bool operator!=(const GridCellPosition& lhs, const GridCellPosition& rhs)
	{
		return !(lhs == rhs);
	}

	std::vector<GridCellPosition> GridCellPosition::GetNeighbors(int rows, int columns) const
	{
		std::vector<GridCellPosition> neighbors;
		if (!IsValid())
			return neighbors;

		neighbors.reserve(8);
		const int& c = col;
		const int& r = row;
		bool left = c - 1 >= 0;
		bool right = c + 1 < columns;
		bool bottom = r - 1 >= 0;
		bool top = r + 1 < rows;
		if (left) {
			neighbors.push_back({ r, c - 1 });
			if (bottom)
				neighbors.push_back({ r - 1, c - 1 });
			if (top)
				neighbors.push_back({ r + 1, c - 1 });
		}
		if (right) {
			neighbors.push_back({ r, c + 1 });
			if (bottom)
				neighbors.push_back({ r - 1, c + 1 });
			if (top)
				neighbors.push_back({ r + 1, c + 1 });
		}

		if (bottom)
			neighbors.push_back({ r - 1, c });
		if (top)
			neighbors.push_back({ r + 1, c });

		return neighbors;
	}

	bool GridCellPosition::IsAdjacentTo(const GridCellPosition& rhs) const
	{
		PP_ASSERT(IsValid() && rhs.IsValid());

		if (*this == rhs)
			return false;

		int deltaRow = row - rhs.row;
		int deltaCol = col - rhs.col;
		return (deltaRow >= -1 && deltaRow <= 1) && (deltaCol >= -1 && deltaCol <= 1);
	}

	bool GridCellPosition::IsDiagonalTo(const GridCellPosition& rhs) const
	{
		PP_ASSERT(IsValid() && rhs.IsValid());

		return row != rhs.row && col != rhs.col;
	}
}
