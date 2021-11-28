#pragma once

#include "algo/path_planner.h"
#include "algo/a_star.h"
#include "algo/bidirectional_a_star.h"
#include "utils/grid.h"
#include <functional>

namespace Planner {
	class OccupancyMap;

	/// @brief State propagator for planar A* grid search.
	class AStarStatePropagatorFcnN2 : public AStarStatePropagator<GridCellPosition> {
	public:
		AStarStatePropagatorFcnN2(const Ref<OccupancyMap>& map,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>& pathCostFcn);

		/// @copydoc Planner::AStarStatePropagator::GetNeighborStates
		virtual std::vector<std::tuple<GridCellPosition, double>> GetNeighborStates(const GridCellPosition& state) override;

	private:
		Ref<OccupancyMap> m_map;
		std::function<double(const GridCellPosition&, const GridCellPosition&)> m_pathCostFcn;
	};

	using AStarHeuristicFcnN2 = AStarConcreteHeuristicFcn<GridCellPosition, std::function<double(const GridCellPosition&, const GridCellPosition&)>>;

	/// @brief Path planner using an A* search over a grid.
	class AStarN2 : public AStar<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>, true> {
		using AStarDeclType = AStar<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>, true>;

	public:
		AStarN2() = default;
	};

	/// @brief Path planner using a bidirectional A* search over a grid.
	class BidirectionalAStarN2 : public BidirectionalAStar<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>, true> {
		using AStarDeclType = BidirectionalAStar<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>, true>;

	public:
		BidirectionalAStarN2() = default;
	};
}
