#pragma once

#include "core/hash.h"

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

		bool IsDiagonalTo(const GridCellPosition& rhs) const;
	};

	bool operator==(const GridCellPosition& lhs, const GridCellPosition& rhs);
	bool operator!=(const GridCellPosition& lhs, const GridCellPosition& rhs);
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

	/// @brief Template grid
	template <typename T>
	struct Grid {
		Grid(int rows, int columns, const T& val) :
			rows(rows), columns(columns)
		{
			if(!(rows > 0 && columns > 0)) {
				std::string msg("Invalid grid size: received ");
				throw std::invalid_argument(msg + std::to_string(rows) + " x " + std::to_string(columns));
			}

			m_data = new T*[rows];
			for (unsigned int i = 0; i < rows; ++i) {
				m_data[i] = new T[columns];
				for (unsigned int j = 0; j < rows; ++j) {
					m_data[i][j] = val;
				}
			}
		}

		virtual ~Grid()
		{
			for (unsigned int i = 0; i < rows; i++) {
				delete[] m_data[i];
			}
			delete[] m_data;
		}

		Grid(const Grid& other) = delete;
		Grid& operator=(const Grid&) = delete;

		T*& operator[](int i) { return m_data[i]; }
		T* const& operator[](int i) const { return m_data[i]; }

		std::vector<GridCellPosition> GetNeighbors(const GridCellPosition& cell) const
		{
			return cell.GetNeighbors(rows, columns);
		}

	public:
		const int rows, columns;

	protected:
		T** m_data = nullptr;
	};
}
