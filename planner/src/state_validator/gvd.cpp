#include "core/base.h"
#include "state_validator/gvd.h"

#include <limits>

namespace Planner {

	template <typename T>
	T** NewArray(int rows, int columns, const T& val)
	{
		T** values = new T*[rows];
		for (unsigned int i = 0; i < rows; ++i) {
			values[i] = new T[columns];
			for (unsigned int j = 0; j < rows; ++j) {
				values[i][j] = val;
			}
		}
		return values;
	}

	template <typename T>
	void DeleteArray(int rows, T** values)
	{
		for (unsigned int i = 0; i < rows; i++) {
			delete[] values[i];
		}
		delete[] values;
	}

	int SquaredDistance(const GridCellPosition& a, const GridCellPosition& b)
	{
		int dr = a.row - b.row;
		int dc = a.col - b.col;
		return dr * dr + dc * dc;
	}

	GVD::ObstacleDistanceMap::ObstacleDistanceMap(unsigned int rows, unsigned int columns, float resolution) :
		rows(rows), columns(columns), resolution(resolution)
	{
		// clang-format off
		distance  = NewArray(rows, columns, std::numeric_limits<int>::max());
		obstacle  = NewArray(rows, columns, GridCellPosition(-1, -1));
		toRaise   = NewArray(rows, columns, false);
		toProcess = NewArray(rows, columns, false);
		voro      = NewArray(rows, columns, true);
		comp      = NewArray(rows, columns, -1);
		// clang-format on
	}

	GVD::ObstacleDistanceMap::~ObstacleDistanceMap()
	{
		DeleteArray(rows, distance);
		DeleteArray(rows, obstacle);
		DeleteArray(rows, toRaise);
		DeleteArray(rows, toProcess);
		DeleteArray(rows, voro);
		DeleteArray(rows, comp);
	}

	void GVD::ObstacleDistanceMap::Update(VoronoiDistanceMap& voronoiMap)
	{
		while (!open.empty()) {
			const auto s = open.top().position;
			open.pop();
			if (!toProcess[s.row][s.col])
				continue;
			if (toRaise[s.row][s.col]) {
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (obstacle[n.row][n.col].IsValid() && !toRaise[n.row][n.col]) {
						if (!IsOccupied(obstacle[n.row][n.col])) {
							distance[n.row][n.col] = std::numeric_limits<int>::max();
							obstacle[n.row][n.col] = GridCellPosition(-1, -1);
							toRaise[n.row][n.col] = true;
						}
						open.push({ n, distance[n.row][n.col] });
						toProcess[n.row][n.col] = true;
					}
				}
				toRaise[s.row][s.col] = false;
			} else if (IsOccupied(obstacle[s.row][s.col])) {
				voro[s.row][s.col] = false;
				voronoiMap.Unset(s);
				toProcess[s.row][s.col] = false;
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (!toRaise[n.row][n.col]) {
						auto d = SquaredDistance(obstacle[s.row][s.col], n);
						if (d < distance[n.row][n.col]) {
							distance[n.row][n.col] = d;
							obstacle[n.row][n.col] = obstacle[s.row][s.col];
							open.push({ n, d });
							toProcess[n.row][n.col] = true;
						} else {
							CheckVoro(voronoiMap, s, n);
						}
					}
				}
			}
		}
	}

	void GVD::ObstacleDistanceMap::SetObstacle(const GridCellPosition& s, int obstacleID)
	{
		obstacle[s.row][s.col] = s;
		comp[s.row][s.col] = obstacleID;
		distance[s.row][s.col] = 0.0f;
		open.push({ s, 0 });
		toProcess[s.row][s.col] = true;
	}

	void GVD::ObstacleDistanceMap::UnsetObstacle(const GridCellPosition& s)
	{
		distance[s.row][s.col] = std::numeric_limits<int>::max();
		obstacle[s.row][s.col] = GridCellPosition(-1, -1);
		comp[s.row][s.col] = -1;
		toRaise[s.row][s.col] = true;
		open.push({ s, std::numeric_limits<int>::max() });
		toProcess[s.row][s.col] = true;
	}

	float GVD::ObstacleDistanceMap::GetDistanceToNearestObstacle(int row, int col) const
	{
		return std::sqrt(distance[row][col]) * resolution;
	}

	bool GVD::ObstacleDistanceMap::CheckVoroConditions(const GridCellPosition& s, const GridCellPosition& n, const GridCellPosition& obstS, const GridCellPosition& obstN)
	{
		return (distance[s.row][s.col] > 1 || distance[n.row][n.col] > 1) && obstN.IsValid() && obstN != obstS && !obstS.IsAdjacentTo(obstN);
	}

	void GVD::ObstacleDistanceMap::CheckVoro(VoronoiDistanceMap& voronoiMap, const GridCellPosition& s, const GridCellPosition& n)
	{
		auto obstS = obstacle[s.row][s.col];
		auto obstN = obstacle[n.row][n.col];

		if (comp[obstS.row][obstS.col] == comp[obstN.row][obstN.col])
			return;

		if ((distance[s.row][s.col] > 1 || distance[n.row][n.col] > 1) && obstacle[n.row][n.col].IsValid()) {
			if (std::abs(obstS.row - obstN.row) > 1 || std::abs(obstS.col - obstN.col) > 1) {
				auto sObstN = SquaredDistance(s, obstN);
				auto nObstS = SquaredDistance(n, obstS);
				auto sStability = sObstN - distance[s.row][s.col];
				auto nStability = nObstS - distance[n.row][n.col];
				if (sStability < 0 || nStability < 0)
					return;
				if (sStability <= nStability) {
					voro[s.row][s.col] = true;
					voronoiMap.Set(s);
				}
				if (nStability <= sStability) {
					voro[n.row][n.col] = true;
					voronoiMap.Set(n);
				}
			}
		}
	}

	void GVD::ObstacleDistanceMap::RebuildVoronoi()
	{
		// TODO use this function
		while (!voroQ.empty()) {
			const auto s = voroQ.top().position;

			if (PatternMatch(s))
				continue;

			bool stop = false;
			auto obstS = obstacle[s.row][s.row];
			auto neighbors = s.GetNeighbors(rows, columns);
			for (auto& n : neighbors) {
				auto obstN = obstacle[n.row][n.col];
				if (CheckVoroConditions(s, n, obstS, obstN) && comp[obstS.row][obstS.col] != comp[obstN.row][obstN.col]) {
					stop = true;
					break;
				}
			}

			if (stop)
				continue;

			voro[s.row][s.col] = false;
			for (auto& n : neighbors)
				if (voro[n.row][n.col])
					voroQ.push({ n, distance[n.row][n.col] });
		}
	}

	bool GVD::ObstacleDistanceMap::PatternMatch(const GridCellPosition& s)
	{
		if (s.row < 1 || s.row >= rows - 1 || s.col < 1 || s.col >= columns - 1)
			return false;

		if (voro[s.row - 1][s.col - 1] && !voro[s.row][s.col - 1] && !voro[s.row - 1][s.col])
			return true;
		if (voro[s.row + 1][s.col - 1] && !voro[s.row][s.col - 1] && !voro[s.row + 1][s.col])
			return true;
		if (voro[s.row - 1][s.col + 1] && !voro[s.row][s.col + 1] && !voro[s.row - 1][s.col])
			return true;
		if (voro[s.row + 1][s.col + 1] && !voro[s.row][s.col + 1] && !voro[s.row + 1][s.col])
			return true;

		// clang-format off
		if ( voro[s.row][s.col - 1] &&  voro[s.row][s.col + 1] && !voro[s.row - 1][s.col] && !voro[s.row + 1][s.col]) return true;
		if (!voro[s.row][s.col - 1] && !voro[s.row][s.col + 1] &&  voro[s.row - 1][s.col] &&  voro[s.row + 1][s.col]) return true;
		if ( voro[s.row][s.col - 1] &&  voro[s.row][s.col + 1] &&  voro[s.row - 1][s.col] &&  voro[s.row + 1][s.col]) return true;
		// clang-format on

		return false;
	}

	bool GVD::ObstacleDistanceMap::IsOccupied(const GridCellPosition& s)
	{
		return s.IsValid() && obstacle[s.row][s.col] == s;
	}

	GVD::VoronoiDistanceMap::VoronoiDistanceMap(unsigned int rows, unsigned int columns, float resolution) :
		rows(rows), columns(columns), resolution(resolution)
	{
		// clang-format off
		distance  = NewArray(rows, columns, std::numeric_limits<int>::max());
		nearest   = NewArray(rows, columns, GridCellPosition(-1, -1));
		toRaise   = NewArray(rows, columns, false);
		toProcess = NewArray(rows, columns, false);
		// clang-format on
	}

	GVD::VoronoiDistanceMap::~VoronoiDistanceMap()
	{
		DeleteArray(rows, distance);
		DeleteArray(rows, nearest);
		DeleteArray(rows, toRaise);
		DeleteArray(rows, toProcess);
	}

	void GVD::VoronoiDistanceMap::Update()
	{
		while (!open.empty()) {
			const auto s = open.top().position;
			open.pop();
			if (toProcess[s.row][s.col]) {
				if (toRaise[s.row][s.col]) {
					for (auto& n : s.GetNeighbors(rows, columns)) {
						if (nearest[n.row][n.col].IsValid() && !toRaise[n.row][n.col]) {
							if (!IsOccupied(nearest[n.row][n.col])) {
								distance[n.row][n.col] = std::numeric_limits<int>::max();
								nearest[n.row][n.col] = GridCellPosition(-1, -1);
								toRaise[n.row][n.col] = true;
							}
							open.push({ n, distance[n.row][n.col] });
							toProcess[n.row][n.col] = true;
						}
					}
					toRaise[s.row][s.col] = false;
				} else if (IsOccupied(nearest[s.row][s.col])) {
					toProcess[s.row][s.col] = false;
					for (auto& n : s.GetNeighbors(rows, columns)) {
						if (!toRaise[n.row][n.col]) {
							auto d = SquaredDistance(nearest[s.row][s.col], n);
							if (d < distance[n.row][n.col]) {
								distance[n.row][n.col] = d;
								nearest[n.row][n.col] = nearest[s.row][s.col];
								open.push({ n, d });
								toProcess[n.row][n.col] = true;
							}
						}
					}
				}
			}
		}
	}

	void GVD::VoronoiDistanceMap::Set(const GridCellPosition& s)
	{
		nearest[s.row][s.col] = s;
		distance[s.row][s.col] = 0;
		open.push({ s, 0 });
		toProcess[s.row][s.col] = true;
	}

	void GVD::VoronoiDistanceMap::Unset(const GridCellPosition& s)
	{
		distance[s.row][s.col] = std::numeric_limits<int>::max();
		nearest[s.row][s.col] = GridCellPosition(-1, -1);
		toRaise[s.row][s.col] = true;
		open.push({ s, std::numeric_limits<int>::max() });
		toProcess[s.row][s.col] = true;
	}

	float GVD::VoronoiDistanceMap::GetDistanceToNearestVoronoiEdge(int row, int col) const
	{
		return std::sqrt(distance[row][col]) * resolution;
	}

	bool GVD::VoronoiDistanceMap::IsOccupied(const GridCellPosition& s)
	{
		return s.IsValid() && nearest[s.row][s.col] == s;
	}

	GVD::PathCostMap::PathCostMap(unsigned int rows, unsigned int columns, float alpha, float dMax) :
		rows(rows), columns(columns), alpha(alpha), dMax(dMax)
	{
		data = NewArray(rows, columns, 0.0f);
	}

	GVD::PathCostMap::~PathCostMap()
	{
		DeleteArray(rows, data);
	}

	GVD::GVD(const ObstacleGrid& grid) :
		rows(grid.rows), columns(grid.columns), resolution(grid.resolution),
		m_obstacleMap(grid.rows, grid.columns, grid.resolution),
		m_voronoiMap(grid.rows, grid.columns, grid.resolution)
	{
	}

	void GVD::Update()
	{
		m_obstacleMap.Update(m_voronoiMap);
		m_voronoiMap.Update();
	}

	float GVD::GetPathCost(int row, int col) const
	{
		auto obstDist = m_obstacleMap.GetDistanceToNearestObstacle(row, col);
		auto voroDist = m_voronoiMap.GetDistanceToNearestVoronoiEdge(row, col);
		if (obstDist >= m_dMax || voroDist == std::numeric_limits<float>::infinity()) {
			return 0.0f;
		} else {
			return (m_alpha / (m_alpha + obstDist))
				* (voroDist / (obstDist + voroDist))
				* (std::pow(obstDist - m_dMax, 2) / std::pow(m_dMax, 2));
		}
	}

	Ref<GVD::PathCostMap> GVD::GeneratePathCostMap() const
	{
		auto map = makeRef<PathCostMap>(rows, columns, m_alpha, m_dMax);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				map->data[r][c] = GetPathCost(r, c);
			}
		}

		return map;
	}

	void GVD::Visualize(const std::string& filename) const
	{
		FILE* F = fopen(filename.c_str(), "w");
		if (!F) {
			PP_ERROR("Could not open {}!", filename);
			return;
		}

		const auto& sizeX = rows;
		const auto& sizeY = columns;
		fprintf(F, "P6\n#\n");
		fprintf(F, "%d %d\n255\n", sizeX, sizeY);
		for(int y = sizeY - 1; y >= 0; y--){      
			for(int x = 0; x < sizeX; x++){	
				unsigned char c = 0;
				if (m_obstacleMap.comp[x][y] >= 0) {
					// Obstacle
					fputc( 0, F );
					fputc( 0, F );
					fputc( 0, F );
				} else {
					// Path cost map
					float f = (50 + (1.0 - GetPathCost(x, y)) * 150);
					f = std::max(0.0f, std::min(f, 255.0f));
					c = (unsigned char)f;
					fputc( c, F );
					fputc( c, F );
					fputc( c, F );
				}
			}
		}
		fclose(F);
	}
}
