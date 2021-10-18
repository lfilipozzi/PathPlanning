#pragma once

#include "core/base.h"
#include "state_validator/obstacle_grid.h"

#include <queue>

namespace Planner {

	/// @brief Generalized Voronoi Diagram
	/// http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
	class GVD {
	private:
		struct VoronoiDistanceMap;

		using PriorityQueue = std::priority_queue<GridCell<int>, std::vector<GridCell<int>>, std::greater<GridCell<int>>>;

		// Delete some operators
		struct Map {
			Map() = default;
			~Map() = default;
			Map(const Map&) = delete;
			Map(Map&&) = delete;
			Map& operator=(const Map&) = delete;
			Map& operator=(Map&&) = delete;
		};

		/// @brief Store map of distance to nearest obstacle.
		struct ObstacleDistanceMap : public Map {
			const int rows, columns;
			const float resolution;

			int** distance;
			GridCellPosition** obstacle;
			bool** toRaise;
			bool** toProcess;
			bool** voro;
			int** comp;
			PriorityQueue open;
			PriorityQueue voroQ;

			ObstacleDistanceMap(unsigned int rows, unsigned int columns, float resolution);
			~ObstacleDistanceMap();

			void Update(VoronoiDistanceMap& voronoiMap);
			void SetObstacle(const GridCellPosition& s, int obstacleID);
			void UnsetObstacle(const GridCellPosition& s);

			float GetDistanceToNearestObstacle(int row, int col) const;

		private:
			bool CheckVoroConditions(const GridCellPosition& s, const GridCellPosition& n, const GridCellPosition& obstS, const GridCellPosition& obstN);
			void CheckVoro(VoronoiDistanceMap& voronoiMap, const GridCellPosition& s, const GridCellPosition& n);
			void RebuildVoronoi();
			bool PatternMatch(const GridCellPosition& s);
			bool IsOccupied(const GridCellPosition& s);
		};

		/// @brief Store map of distance to nearest Voronoi edge.
		struct VoronoiDistanceMap : public Map {
			const int rows, columns;
			const float resolution;

			int** distance;
			GridCellPosition** nearest;
			bool** toRaise;
			bool** toProcess;
			PriorityQueue open;

			VoronoiDistanceMap(unsigned int rows, unsigned int columns, float resolution);
			~VoronoiDistanceMap();

			void Update();
			void Set(const GridCellPosition& s);
			void Unset(const GridCellPosition& s);

			float GetDistanceToNearestVoronoiEdge(int row, int col) const;

		private:
			bool IsOccupied(const GridCellPosition& s);
		};

	public:
		/// @brief Store map of cost associated to the Voronoi potential field.
		struct PathCostMap : public Map {
			const int rows, columns;
			const float alpha, dMax;

			float** data;

			PathCostMap(unsigned int rows, unsigned int columns, float alpha, float dMax);
			~PathCostMap();
		};

	public:
		GVD(const ObstacleGrid& grid);
		~GVD() = default;

		void SetObstacle(const GridCellPosition& s, int obstacleID) { m_obstacleMap.SetObstacle(s, obstacleID); }
		void UnsetObstacle(const GridCellPosition& s) { m_obstacleMap.UnsetObstacle(s); }

		/// @brief Update the Voronoi potential field
		void Update();

		/// @brief Return the position to the nearest obstacle.
		inline const GridCellPosition& GetNearestObstacle(int row, int col) const { return m_obstacleMap.obstacle[row][col]; }
		/// @overload
		inline const GridCellPosition& GetNearestObstacle(const GridCellPosition& position) const { return GetNearestObstacle(position.row, position.col); }

		/// @brief Return the position to the nearest Voronoi edge.
		inline const GridCellPosition& GetNearestVoronoiEdge(int row, int col) const { return m_voronoiMap.nearest[row][col]; }
		/// @overload
		inline const GridCellPosition& GetNearestVoronoiEdge(const GridCellPosition& position) const { return GetNearestVoronoiEdge(position.row, position.col); }

		/// @brief Return the distance to the nearest obstacle
		inline float GetDistanceToNearestObstacle(int row, int col) const { return m_obstacleMap.GetDistanceToNearestObstacle(row, col); }
		/// @overload
		inline float GetDistanceToNearestObstacle(const GridCellPosition& position) const { return GetDistanceToNearestObstacle(position.row, position.col); }

		/// @brief Return the distance to the nearest Voronoi edge.
		inline float GetDistanceToNearestVoronoiEdge(int row, int col) const { return m_voronoiMap.GetDistanceToNearestVoronoiEdge(row, col); }
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
		ObstacleDistanceMap m_obstacleMap;
		VoronoiDistanceMap m_voronoiMap;

		float m_alpha = 20.0f, m_dMax = 30.0f;
	};
}
