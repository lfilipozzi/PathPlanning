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
	/// @details Reference-counted
	template <typename T>
	struct Grid {
		Grid(int rows, int columns, const T& val) :
			rows(rows), columns(columns)
		{
			m_count = new int;
			*m_count = 1;

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
			DecreaseCount();
		}

		Grid(const Grid& other) :
			rows(other.rows), columns(other.columns)
		{
			IncreaseCount(other);
		}

		Grid& operator=(const Grid&) = delete;

		T*& operator[](int i) { return m_data[i]; }
		T* const& operator[](int i) const { return m_data[i]; }

		operator bool()
		{
			if (!m_count || !m_data)
				return false;
			return true;
		}

		/// @brief Return the count.
		int GetCount() const { return m_count ? *m_count : 0; }

	private:
		void IncreaseCount(const Grid& other)
		{
			m_data = other.m_data;
			m_count = other.m_count;
			if (m_count)
				(*m_count)++;
		}

		void DecreaseCount()
		{
			if (!m_count)
				return;

			if (*m_count <= 1) {
				for (unsigned int i = 0; i < rows; i++) {
					delete[] m_data[i];
				}
				delete[] m_data;
				delete m_count;
			} else {
				(*m_count)--;
			}
		}

	public:
		const int rows, columns;

	protected:
		T** m_data = nullptr;
		int* m_count = nullptr;
	};
}
