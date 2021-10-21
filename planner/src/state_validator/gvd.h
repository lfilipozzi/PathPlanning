#pragma once

#include "core/base.h"
#include "utils/grid.h"

#include <queue>

namespace Planner {

	class OccupancyMap;

	/// @brief Generalized Voronoi Diagram
	/// http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
	class GVD {
	private:
		using PriorityQueue = std::priority_queue<GridCell<int>, std::vector<GridCell<int>>, std::greater<GridCell<int>>>;

	public:
		struct VoronoiDistanceMap;

		/// @brief Store map of distance to nearest obstacle.
		/// @details Given a binary occupancy matrix @occupancy, the class
		/// allows to find the distance to the closest obstacle.
		struct ObstacleDistanceMap {
		public:
			ObstacleDistanceMap(const Grid<int>& occupancy, float resolution);
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

			Grid<int> distance;
			Grid<GridCellPosition> obstacle;
			Grid<bool> toRaise;
			Grid<bool> toProcess;
			Grid<bool> voro;
			PriorityQueue open;
			PriorityQueue voroQ;

			Grid<int> occupancy;

		private:
			Ref<VoronoiDistanceMap> m_voronoiMap;
		};

		/// @brief Store map of distance to nearest Voronoi edge.
		struct VoronoiDistanceMap {
		public:
			VoronoiDistanceMap(unsigned int rows, unsigned int columns, float resolution);
			~VoronoiDistanceMap() = default;

			void Update();
			void SetEdge(const GridCellPosition& s);
			void UnsetEdge(const GridCellPosition& s);

			float GetDistanceToNearestVoronoiEdge(int row, int col) const;
			float GetDistanceToNearestVoronoiEdge(const GridCellPosition& s) const;

		private:
			bool IsOccupied(const GridCellPosition& s);

		public:
			const int rows, columns;
			const float resolution;

			Grid<int> distance;
			Grid<GridCellPosition> edge;
			Grid<bool> toRaise;
			Grid<bool> toProcess;
			PriorityQueue open;
		};

	public:
		/// @brief Store map of cost associated to the Voronoi potential field.
		struct PathCostMap : Grid<float> {
		public:
			PathCostMap(unsigned int rows, unsigned int columns, float alpha, float dMax);
			~PathCostMap() = default;

		public:
			const float alpha, dMax;
		};

	public:
		GVD(const OccupancyMap& map);
		~GVD() = default;

		void SetObstacle(const GridCellPosition& s) { m_obstacleMap->SetObstacle(s); }
		void UnsetObstacle(const GridCellPosition& s) { m_obstacleMap->UnsetObstacle(s); }

		/// @brief Update the Voronoi potential field
		void Update();

		/// @brief Return the position to the nearest obstacle.
		inline const GridCellPosition& GetNearestObstacle(int row, int col) const { return m_obstacleMap->obstacle[row][col]; }
		/// @overload
		inline const GridCellPosition& GetNearestObstacle(const GridCellPosition& position) const { return GetNearestObstacle(position.row, position.col); }

		/// @brief Return the position to the nearest Voronoi edge.
		inline const GridCellPosition& GetNearestVoronoiEdge(int row, int col) const { return m_voronoiMap->edge[row][col]; }
		/// @overload
		inline const GridCellPosition& GetNearestVoronoiEdge(const GridCellPosition& position) const { return GetNearestVoronoiEdge(position.row, position.col); }

		/// @brief Return the distance to the nearest obstacle
		inline float GetDistanceToNearestObstacle(int row, int col) const { return m_obstacleMap->GetDistanceToNearestObstacle(row, col); }
		/// @overload
		inline float GetDistanceToNearestObstacle(const GridCellPosition& position) const { return GetDistanceToNearestObstacle(position.row, position.col); }

		/// @brief Return the distance to the nearest Voronoi edge.
		inline float GetDistanceToNearestVoronoiEdge(int row, int col) const { return m_voronoiMap->GetDistanceToNearestVoronoiEdge(row, col); }
		/// @overload
		inline float GetDistanceToNearestVoronoiEdge(const GridCellPosition& position) const { return GetDistanceToNearestVoronoiEdge(position.row, position.col); }

		/// @brief Return the value of the Voronoi potential field.
		float GetPathCost(int row, int col) const;
		/// @overload
		inline float GetPathCost(const GridCellPosition& position) const { return GetPathCost(position.row, position.col); }

		/// @brief Create a map of the Voronoi potential field.
		Ref<PathCostMap> GeneratePathCostMap() const;

		void Visualize(const std::string& filename) const;

	private:
		const int rows, columns;
		const float resolution;
		Ref<ObstacleDistanceMap> m_obstacleMap;
		Ref<VoronoiDistanceMap> m_voronoiMap;

		float m_alpha = 20.0f, m_dMax = 30.0f;
	};
}
