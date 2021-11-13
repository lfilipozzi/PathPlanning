#include "algo/planar_a_star_grid.h"
#include "state_validator/occupancy_map.h"

namespace Planner {
	class OccupancyMap;

	PlanarAStarStatePropagator::PlanarAStarStatePropagator(const Ref<OccupancyMap>& map,
		const std::function<double(const GridCellPosition&, const GridCellPosition&)>& pathCostFcn) :
		m_map(map),
		m_pathCostFcn(pathCostFcn) { }

	std::vector<std::tuple<GridCellPosition, double>> PlanarAStarStatePropagator::GetNeighborStates(const GridCellPosition& cell)
	{
		auto cellNeighbors = cell.GetNeighbors(m_map->Rows(), m_map->Columns());

		std::vector<std::tuple<GridCellPosition, double>> neighbors;
		neighbors.reserve(cellNeighbors.size());
		for (auto& n : cellNeighbors) {
			if (m_map->IsOccupied(n))
				continue;
			if (n.IsDiagonalTo(cell))
				if (m_map->IsOccupied({ n.row, cell.col }) && m_map->IsOccupied({ cell.row, n.col }))
					continue;
			double cost = m_pathCostFcn(cell, n);
			neighbors.emplace_back(std::move(n), cost);
		}
		return neighbors;
	}

	bool PlanarAStarGrid::Initialize(const Ref<OccupancyMap>& map,
		const std::function<double(const GridCellPosition&, const GridCellPosition&)>& pathCostFcn,
		const std::function<double(const GridCellPosition&, const GridCellPosition&)>& heuristicFcn)
	{
		auto propagator = makeRef<PlanarAStarStatePropagator>(map, pathCostFcn);
		auto heuristic = makeRef<PlanarAStarHeuristic>(heuristicFcn);
		return AStarDeclType::Initialize(propagator, heuristic);
	}

	bool PlanarBidirectionalAStarGrid::Initialize(const Ref<OccupancyMap>& map,
		const std::function<double(const GridCellPosition&, const GridCellPosition&)>& pathCostFcn,
		const std::function<double(const GridCellPosition&, const GridCellPosition&)>& heuristicFcn)
	{
		auto propagator = makeRef<PlanarAStarStatePropagator>(map, pathCostFcn);
		auto heuristic = makeRef<PlanarAStarHeuristic>(heuristicFcn);
		auto [fHeuristic, rHeuristic] = AStarDeclType::GetAverageHeuristicPair(heuristic, heuristic);
		return AStarDeclType::Initialize(propagator, propagator, fHeuristic, rHeuristic);
	}
}
