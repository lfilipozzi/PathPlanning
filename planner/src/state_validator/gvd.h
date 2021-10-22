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
		/// @details Given a binary occupancy matrix @occupancy, the class
		/// allows to find the distance to the closest obstacle.
		class ObstacleDistanceMap {
		public:
			ObstacleDistanceMap(const Ref<Grid<int>>& occupancy, float resolution);
			~ObstacleDistanceMap() = default;

			void Update();
			void SetObstacle(const GridCellPosition& s);
			void UnsetObstacle(const GridCellPosition& s);

			float GetDistanceToNearestObstacle(int row, int col) const;
			float GetDistanceToNearestObstacle(const GridCellPosition& s) const;

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
			Grid<int> distance;
			Grid<GridCellPosition> obstacle;
			Grid<bool> toRaise;
			Grid<bool> toProcess;
			Grid<bool> voro;
			PriorityQueue open;
			PriorityQueue voroQ;

			Ref<Grid<int>> occupancy;
			Ref<VoronoiDistanceMap> m_voronoiMap;
		};

		/// @brief Store map of distance to nearest Voronoi edge.
		class VoronoiDistanceMap {
		public:
			~VoronoiDistanceMap() = default;

			void Update();
			void SetEdge(const GridCellPosition& s);
			void UnsetEdge(const GridCellPosition& s);

			float GetDistanceToNearestVoronoiEdge(int row, int col) const;
			float GetDistanceToNearestVoronoiEdge(const GridCellPosition& s) const;

		protected:
			// Constructor should be used only by ObstacleDistanceMap
			VoronoiDistanceMap(unsigned int rows, unsigned int columns, float resolution);

		private:
			bool IsOccupied(const GridCellPosition& s);

		public:
			const int rows, columns;
			const float resolution;

		private:
			Grid<int> distance;
			Grid<GridCellPosition> edge;
			Grid<bool> toRaise;
			Grid<bool> toProcess;
			PriorityQueue open;
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

		/// @brief Return the distance to the nearest obstacle
		inline float GetDistanceToNearestObstacle(int row, int col) const { return m_obstacleMap->GetDistanceToNearestObstacle(row, col); }
		/// @overload
		inline float GetDistanceToNearestObstacle(const GridCellPosition& position) const { return GetDistanceToNearestObstacle(position.row, position.col); }

		/// @brief Return the distance to the nearest Voronoi edge.
		inline float GetDistanceToNearestVoronoiEdge(int row, int col) const { return m_voronoiMap->GetDistanceToNearestVoronoiEdge(row, col); }
		/// @overload
		inline float GetDistanceToNearestVoronoiEdge(const GridCellPosition& position) const { return GetDistanceToNearestVoronoiEdge(position.row, position.col); }

		/// @brief Return the value of the Voronoi potential field.
		inline float GetPathCost(int row, int col) const { return m_pathCostMap[row][col]; }
		/// @overload
		inline float GetPathCost(const GridCellPosition& position) const { return GetPathCost(position.row, position.col); }

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
