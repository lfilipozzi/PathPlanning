#pragma once

#include "algo/path_planner.h"
#include "algo/a_star.h"
#include "algo/bidirectional_a_star.h"
#include "utils/grid.h"
#include <functional>

namespace Planner {
	class OccupancyMap;

	/// @brief State propagator for planar A* grid search.
	class PlanarAStarStatePropagator : public AStarStatePropagator<GridCellPosition> {
	public:
		PlanarAStarStatePropagator(const Ref<OccupancyMap>& map,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>& pathCostFcn);

		/// @copydoc Planner::AStarStatePropagator::GetNeighborStates
		virtual std::vector<std::tuple<GridCellPosition, double>> GetNeighborStates(const GridCellPosition& state) override;

	private:
		Ref<OccupancyMap> m_map;
		std::function<double(const GridCellPosition&, const GridCellPosition&)> m_pathCostFcn;
	};

	using PlanarAStarHeuristic = AStarConcreteHeuristicFcn<GridCellPosition, std::function<double(const GridCellPosition&, const GridCellPosition&)>>;

	/// @brief Path planner using an A* search over a grid.
	class PlanarAStarGrid : public AStar<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>, true> {
		using AStarDeclType = AStar<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>, true>;

	public:
		PlanarAStarGrid() = default;

		/// @brief Initialize the search.
		/// @param map The occupancy map.
		/// @param pathCostFcn Function returning the path cost between two
		/// neighbor cells. The first argument of the function is the origin
		/// cell, the second argument is the destination cell.
		/// @param heuristicFcn Function returning the heuristic cost between a
		/// cell (first argument) and the goal (second argument).
		bool Initialize(const Ref<OccupancyMap>& map,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>& pathCostFcn,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>& heuristicFcn);

	protected:
		using AStarDeclType::Initialize;
	};

	/// @brief Path planner using a bidirectional A* search over a grid.
	class PlanarBidirectionalAStarGrid : public BidirectionalAStar<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>, true> {
		using AStarDeclType = BidirectionalAStar<GridCellPosition, std::hash<GridCellPosition>, std::equal_to<GridCellPosition>, true>;

	public:
		PlanarBidirectionalAStarGrid() = default;

		/// @brief Initialize the search.
		/// @param map The occupancy map.
		/// @param pathCostFcn Function returning the path cost between two
		/// neighbor cells. The first argument of the function is the origin
		/// cell, the second argument is the destination cell.
		/// @param heuristicFcn Function returning the heuristic cost between a
		/// cell (first argument) and the goal (second argument).
		bool Initialize(const Ref<OccupancyMap>& map,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>& pathCostFcn,
			const std::function<double(const GridCellPosition&, const GridCellPosition&)>& heuristicFcn);

	protected:
		using AStarDeclType::Initialize;
	};
}
