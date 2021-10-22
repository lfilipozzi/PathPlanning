#include "core/base.h"
#include "state_validator/gvd.h"
#include "state_validator/occupancy_map.h"
#include "utils/make_ref_enabler.h"

#include <limits>
#include <cmath>

namespace Planner {

	int SquaredDistance(const GridCellPosition& a, const GridCellPosition& b)
	{
		int dr = a.row - b.row;
		int dc = a.col - b.col;
		return dr * dr + dc * dc;
	}

	GVD::ObstacleDistanceMap::ObstacleDistanceMap(const Ref<Grid<int>>& occupancy, float resolution) :
		rows(occupancy->rows), columns(occupancy->columns), resolution(resolution),
		distance(occupancy->rows, occupancy->columns, std::numeric_limits<int>::max()),
		obstacle(occupancy->rows, occupancy->columns, GridCellPosition(-1, -1)),
		toRaise(occupancy->rows, occupancy->columns, false),
		toProcess(occupancy->rows, occupancy->columns, false),
		voro(occupancy->rows, occupancy->columns, true),
		occupancy(occupancy)
	{
	}

	void GVD::ObstacleDistanceMap::Update()
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
				if (m_voronoiMap)
					m_voronoiMap->UnsetEdge(s);
				toProcess[s.row][s.col] = false;
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (!toRaise[n.row][n.col]) {
						auto d = SquaredDistance(obstacle[s.row][s.col], n);
						if (d < distance[n.row][n.col]) {
							distance[n.row][n.col] = d;
							obstacle[n.row][n.col] = obstacle[s.row][s.col];
							open.push({ n, d });
							toProcess[n.row][n.col] = true;
						} else if (m_voronoiMap) {
							CheckVoro(s, n);
						}
					}
				}
			}
		}
	}

	void GVD::ObstacleDistanceMap::SetObstacle(const GridCellPosition& s)
	{
		obstacle[s.row][s.col] = s;
		distance[s.row][s.col] = 0.0f;
		open.push({ s, 0 });
		toProcess[s.row][s.col] = true;
	}

	void GVD::ObstacleDistanceMap::UnsetObstacle(const GridCellPosition& s)
	{
		distance[s.row][s.col] = std::numeric_limits<int>::max();
		obstacle[s.row][s.col] = GridCellPosition(-1, -1);
		toRaise[s.row][s.col] = true;
		open.push({ s, std::numeric_limits<int>::max() });
		toProcess[s.row][s.col] = true;
	}

	float GVD::ObstacleDistanceMap::GetDistanceToNearestObstacle(int row, int col) const
	{
		return std::sqrt(distance[row][col]) * resolution;
	}

	float GVD::ObstacleDistanceMap::GetDistanceToNearestObstacle(const GridCellPosition& s) const
	{
		return GetDistanceToNearestObstacle(s.row, s.col);
	}

	Ref<GVD::VoronoiDistanceMap> GVD::ObstacleDistanceMap::GetVoronoiDistanceMap()
	{
		if (m_voronoiMap)
			return m_voronoiMap;

		m_voronoiMap = makeRef<MakeRefEnabler<VoronoiDistanceMap>>(rows, columns, resolution);
		return m_voronoiMap;
	}

	bool GVD::ObstacleDistanceMap::CheckVoroConditions(const GridCellPosition& s, const GridCellPosition& n, const GridCellPosition& obstS, const GridCellPosition& obstN)
	{
		return (distance[s.row][s.col] > 1 || distance[n.row][n.col] > 1) && obstN.IsValid() && obstN != obstS && !obstS.IsAdjacentTo(obstN);
	}

	void GVD::ObstacleDistanceMap::CheckVoro(const GridCellPosition& s, const GridCellPosition& n)
	{
		auto obstS = obstacle[s.row][s.col];
		auto obstN = obstacle[n.row][n.col];

		if ((*occupancy)[obstS.row][obstS.col] == (*occupancy)[obstN.row][obstN.col])
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
					m_voronoiMap->SetEdge(s);
				}
				if (nStability <= sStability) {
					voro[n.row][n.col] = true;
					m_voronoiMap->SetEdge(n);
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
				if (CheckVoroConditions(s, n, obstS, obstN) && (*occupancy)[obstS.row][obstS.col] != (*occupancy)[obstN.row][obstN.col]) {
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
		rows(rows), columns(columns), resolution(resolution),
		distance(rows, columns, std::numeric_limits<int>::max()),
		edge(rows, columns, GridCellPosition(-1, -1)),
		toRaise(rows, columns, false),
		toProcess(rows, columns, false)
	{
	}

	void GVD::VoronoiDistanceMap::Update()
	{
		while (!open.empty()) {
			const auto s = open.top().position;
			open.pop();
			if (!toProcess[s.row][s.col])
				continue;
			if (toRaise[s.row][s.col]) {
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (edge[n.row][n.col].IsValid() && !toRaise[n.row][n.col]) {
						if (!IsOccupied(edge[n.row][n.col])) {
							distance[n.row][n.col] = std::numeric_limits<int>::max();
							edge[n.row][n.col] = GridCellPosition(-1, -1);
							toRaise[n.row][n.col] = true;
						}
						open.push({ n, distance[n.row][n.col] });
						toProcess[n.row][n.col] = true;
					}
				}
				toRaise[s.row][s.col] = false;
			} else if (IsOccupied(edge[s.row][s.col])) {
				toProcess[s.row][s.col] = false;
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (!toRaise[n.row][n.col]) {
						auto d = SquaredDistance(edge[s.row][s.col], n);
						if (d < distance[n.row][n.col]) {
							distance[n.row][n.col] = d;
							edge[n.row][n.col] = edge[s.row][s.col];
							open.push({ n, d });
							toProcess[n.row][n.col] = true;
						}
					}
				}
			}
		}
	}

	void GVD::VoronoiDistanceMap::SetEdge(const GridCellPosition& s)
	{
		edge[s.row][s.col] = s;
		distance[s.row][s.col] = 0;
		open.push({ s, 0 });
		toProcess[s.row][s.col] = true;
	}

	void GVD::VoronoiDistanceMap::UnsetEdge(const GridCellPosition& s)
	{
		distance[s.row][s.col] = std::numeric_limits<int>::max();
		edge[s.row][s.col] = GridCellPosition(-1, -1);
		toRaise[s.row][s.col] = true;
		open.push({ s, std::numeric_limits<int>::max() });
		toProcess[s.row][s.col] = true;
	}

	float GVD::VoronoiDistanceMap::GetDistanceToNearestVoronoiEdge(int row, int col) const
	{
		return std::sqrt(distance[row][col]) * resolution;
	}

	float GVD::VoronoiDistanceMap::GetDistanceToNearestVoronoiEdge(const GridCellPosition& s) const
	{
		return GetDistanceToNearestVoronoiEdge(s.row, s.col);
	}

	bool GVD::VoronoiDistanceMap::IsOccupied(const GridCellPosition& s)
	{
		return s.IsValid() && edge[s.row][s.col] == s;
	}

	GVD::PathCostMap::PathCostMap(unsigned int rows, unsigned int columns, float alpha, float dMax) :
		Grid<float>(rows, columns, 0.0f), alpha(alpha), dMax(dMax)
	{
	}

	void GVD::PathCostMap::Update(const ObstacleDistanceMap& obstacleMap, const VoronoiDistanceMap& voronoiMap)
	{
		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				auto obstDist = obstacleMap.GetDistanceToNearestObstacle(r, c);
				auto voroDist = voronoiMap.GetDistanceToNearestVoronoiEdge(r, c);
				if (obstDist >= dMax || voroDist == std::numeric_limits<float>::infinity()) {
					m_data[r][c] =  0.0f;
				} else {
					m_data[r][c] = (alpha / (alpha + obstDist))
						* (voroDist / (obstDist + voroDist))
						* (std::pow(obstDist - dMax, 2) / std::pow(dMax, 2));
				}
			}
		}
	}

	GVD::GVD(const Ref<OccupancyMap>& map) :
		rows(map->rows), columns(map->columns), resolution(map->resolution),
		m_pathCostMap(rows, columns, alpha, dMax)
	{
		m_occupancyMap = map;
		m_obstacleMap = m_occupancyMap->GetObstacleDistanceMap();
		m_voronoiMap = m_obstacleMap->GetVoronoiDistanceMap();
	}

	void GVD::Update()
	{
		m_obstacleMap->Update();
		m_voronoiMap->Update();
		m_pathCostMap.Update(*m_obstacleMap, *m_voronoiMap);
	}

	bool GVD::GetDistanceToNearestObstacle(const Point2d& position, float& distance) const
	{
		auto cell = m_occupancyMap->WorldToGridPosition(position);
		if (!cell.IsValid())
			return false;
		distance = GetDistanceToNearestObstacle(cell);
		return true;
	}

	bool GVD::GetDistanceToNearestVoronoiEdge(const Point2d& position, float& distance) const
	{
		auto cell = m_occupancyMap->WorldToGridPosition(position);
		if (!cell.IsValid())
			return false;
		distance = GetDistanceToNearestVoronoiEdge(cell);
		return true;
	}

	bool GVD::GetPathCost(const Point2d& position, float& cost) const
	{
		auto cell = m_occupancyMap->WorldToGridPosition(position);
		if (!cell.IsValid())
			return false;
		cost = GetPathCost(cell);
		return true;
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
		for (int y = sizeY - 1; y >= 0; y--) {
			for (int x = 0; x < sizeX; x++) {
				unsigned char c = 0;
				if (m_occupancyMap->IsOccupied({ x, y })) {
					// Obstacle
					fputc(0, F);
					fputc(0, F);
					fputc(0, F);
				} else {
					// Path cost map
					float f = (50 + (1.0 - m_pathCostMap[x][y]) * 150);
					f = std::max(0.0f, std::min(f, 255.0f));
					c = (unsigned char)f;
					fputc(c, F);
					fputc(c, F);
					fputc(c, F);
				}
			}
		}
		fclose(F);
	}
}
