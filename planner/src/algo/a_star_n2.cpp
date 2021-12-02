#include "algo/a_star_n2.h"
#include "state_validator/occupancy_map.h"

namespace Planner {
	class OccupancyMap;

	AStarStatePropagatorFcnN2::AStarStatePropagatorFcnN2(const Ref<OccupancyMap>& map,
		const std::function<double(const GridCellPosition&, const GridCellPosition&)>& pathCostFcn) :
		m_map(map),
		m_pathCostFcn(pathCostFcn) { }

	std::vector<std::tuple<GridCellPosition, NullAction, double>> AStarStatePropagatorFcnN2::GetNeighborStates(const GridCellPosition& cell)
	{
		auto cellNeighbors = cell.GetNeighbors(m_map->Rows(), m_map->Columns());

		std::vector<std::tuple<GridCellPosition, NullAction, double>> neighbors;
		neighbors.reserve(cellNeighbors.size());
		for (auto& n : cellNeighbors) {
			if (m_map->IsOccupied(n))
				continue;
			if (n.IsDiagonalTo(cell))
				if (m_map->IsOccupied({ n.row, cell.col }) && m_map->IsOccupied({ cell.row, n.col }))
					continue;
			double cost = m_pathCostFcn(cell, n);
			neighbors.emplace_back(std::move(n), NullAction(), cost);
		}
		return neighbors;
	}
}
