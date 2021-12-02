#pragma once

#include "core/base.h"
#include "utils/grid.h"
#include "geometry/2dplane.h"

#include <queue>

namespace Planner {

	class OccupancyMap;

	/// @brief Generalized Voronoi Diagram
	/// http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
	class GVD {
	private:
		using PriorityQueue = std::priority_queue<GridCell<int>, std::vector<GridCell<int>>, std::greater<GridCell<int>>>;

	public:
		class VoronoiDistanceMap;

		/// @brief Store map of distance to nearest obstacle.
		/// @details Given an occupancy matrix @occupancy, this class allows to
		/// find the distance to the closest obstacle.
		class ObstacleDistanceMap {
			friend class GVD;

		public:
			ObstacleDistanceMap(const Ref<Grid<int>>& occupancy, float resolution);
			~ObstacleDistanceMap() = default;

			void Update();
			void SetObstacle(const GridCellPosition& s);
			void UnsetObstacle(const GridCellPosition& s);

			inline const GridCellPosition& GetNearestObstacleCell(int row, int col) const { return m_obstacle[row][col]; }
			inline const GridCellPosition& GetNearestObstacleCell(const GridCellPosition& cell) const { return GetNearestObstacleCell(cell.row, cell.col); }
			inline float GetDistanceToNearestObstacle(int row, int col) const { return std::sqrt(m_distance[row][col]) * resolution; }
			inline float GetDistanceToNearestObstacle(const GridCellPosition& s) const { return GetDistanceToNearestObstacle(s.row, s.col); }

			Ref<VoronoiDistanceMap> GetVoronoiDistanceMap();

		private:
			bool CheckVoroConditions(const GridCellPosition& s, const GridCellPosition& n, const GridCellPosition& obstS, const GridCellPosition& obstN);
			void CheckVoro(const GridCellPosition& s, const GridCellPosition& n);
			void RebuildVoronoi();
			bool PatternMatch(const GridCellPosition& s);
			bool IsOccupied(const GridCellPosition& s);

		public:
			const int rows, columns;
			const float resolution;

		private:
			Grid<int> m_distance;
			Grid<GridCellPosition> m_obstacle;
			Grid<bool> m_toRaise;
			Grid<bool> m_toProcess;
			Grid<bool> m_voro;
			PriorityQueue m_open;
			PriorityQueue m_voroQ;

			Ref<Grid<int>> m_occupancy;
			Ref<VoronoiDistanceMap> m_voronoiMap;
		};

		/// @brief Store map of distance to nearest Voronoi edge.
		class VoronoiDistanceMap {
		public:
			~VoronoiDistanceMap() = default;

			void Update();
			void SetEdge(const GridCellPosition& s);
			void UnsetEdge(const GridCellPosition& s);

			inline const GridCellPosition& GetNearestVoronoiEdgeCell(int row, int col) const { return m_edge[row][col]; }
			inline const GridCellPosition& GetNearestVoronoiEdgeCell(const GridCellPosition& cell) const { return GetNearestVoronoiEdgeCell(cell.row, cell.col); }
			inline float GetDistanceToNearestVoronoiEdge(int row, int col) const { return std::sqrt(m_distance[row][col]) * resolution; }
			inline float GetDistanceToNearestVoronoiEdge(const GridCellPosition& s) const { return GetDistanceToNearestVoronoiEdge(s.row, s.col); }

		protected:
			// Constructor should be used only by ObstacleDistanceMap
			VoronoiDistanceMap(unsigned int rows, unsigned int columns, float resolution);

		private:
			bool IsOccupied(const GridCellPosition& s);

		public:
			const int rows, columns;
			const float resolution;

		private:
			Grid<int> m_distance;
			Grid<GridCellPosition> m_edge;
			Grid<bool> m_toRaise;
			Grid<bool> m_toProcess;
			PriorityQueue m_open;
		};

	private:
		/// @brief Store map of cost associated to the Voronoi potential field.
		struct PathCostMap : Grid<float> {
		public:
			PathCostMap(unsigned int rows, unsigned int columns, float alpha, float dMax);
			~PathCostMap() = default;

			void Update(const ObstacleDistanceMap& obstacleMap, const VoronoiDistanceMap& voronoiMap);

		public:
			const float alpha, dMax;
		};

	public:
		GVD(const Ref<OccupancyMap>& map);
		~GVD() = default;

		/// @brief Update the Voronoi potential field
		void Update();

		/// @brief Return the position to the nearest obstacle.
		/// @warning Note that no check is performed on the cell position.
		inline const GridCellPosition& GetNearestObstacleCell(int row, int col) const { return m_obstacleMap->GetNearestObstacleCell(row, col); }
		/// @overload
		inline const GridCellPosition& GetNearestObstacleCell(const GridCellPosition& position) const { return m_obstacleMap->GetNearestObstacleCell(position); }

		/// @brief Return the position to the nearest Voronoi edge.
		/// @warning Note that no check is performed on the cell position.
		inline const GridCellPosition& GetNearestVoronoiEdgeCell(int row, int col) const { return m_voronoiMap->GetNearestVoronoiEdgeCell(row, col); }
		/// @overload
		inline const GridCellPosition& GetNearestVoronoiEdgeCell(const GridCellPosition& position) const { return m_voronoiMap->GetNearestVoronoiEdgeCell(position); }

		/// @brief Return the distance to the nearest obstacle
		/// @warning Note that no check is performed on the cell position.
		inline float GetDistanceToNearestObstacle(int row, int col) const { return m_obstacleMap->GetDistanceToNearestObstacle(row, col); }
		/// @overload
		inline float GetDistanceToNearestObstacle(const GridCellPosition& position) const { return m_obstacleMap->GetDistanceToNearestObstacle(position); }

		/// @brief Return the distance to the nearest Voronoi edge.
		/// @warning Note that no check is performed on the cell position.
		inline float GetDistanceToNearestVoronoiEdge(int row, int col) const { return m_voronoiMap->GetDistanceToNearestVoronoiEdge(row, col); }
		/// @overload
		inline float GetDistanceToNearestVoronoiEdge(const GridCellPosition& position) const { return m_voronoiMap->GetDistanceToNearestVoronoiEdge(position); }

		/// @brief Return the value of the Voronoi potential field.
		/// @warning Note that no check is performed on the cell position.
		inline float GetPathCost(int row, int col) const { return m_pathCostMap[row][col]; }
		/// @overload
		inline float GetPathCost(const GridCellPosition& position) const { return GetPathCost(position.row, position.col); }

		/// @brief Return the position of the nearest obstacle
		/// @param[in] position The world position.
		/// @param[out] obstacle The position of the nearest obstacle.
		/// @return True if position is in the map, false otherwise.
		[[nodiscard]] bool GetNearestObstaclePosition(const Point2d& position, Point2d& obstacle) const;
		/// @brief Return the position of the nearest Voronoi edge
		/// @param[in] position The world position.
		/// @param[out] voronoi The position of the nearest obstacle.
		/// @return True if position is in the map, false otherwise.
		[[nodiscard]] bool GetNearestVoronoiEdgePosition(const Point2d& position, Point2d& voronoi) const;
		/// @brief Return the distance to the nearest obstacle
		/// @param[in] position The world position.
		/// @param[out] distance The distance to the nearest obstacle.
		/// @return True if the position is in the map, false otherwise.
		[[nodiscard]] bool GetDistanceToNearestObstacle(const Point2d& position, float& distance) const;
		/// @brief Return the distance to the nearest Voronoi edge.
		/// @param[in] position The world position.
		/// @param[out] distance The distance to the nearest Voronoi edge.
		/// @return True if the position is in the map, false otherwise.
		[[nodiscard]] bool GetDistanceToNearestVoronoiEdge(const Point2d& position, float& distance) const;
		/// @brief Return the value of the Voronoi potential field.
		/// @param[in] position The world position.
		/// @param[out] cost The path cost.
		/// @return True if the position is in the map, false otherwise.
		[[nodiscard]] bool GetPathCost(const Point2d& position, float& cost) const;

		void Visualize(const std::string& filename) const;

	public:
		const int rows, columns;
		const float resolution;
		const float alpha = 20.0f, dMax = 30.0f;

	private:
		Ref<OccupancyMap> m_occupancyMap;
		Ref<ObstacleDistanceMap> m_obstacleMap;
		Ref<VoronoiDistanceMap> m_voronoiMap;
		PathCostMap m_pathCostMap;
	};
}
