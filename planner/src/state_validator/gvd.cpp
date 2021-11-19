#include "core/base.h"
#include "state_validator/gvd.h"
#include "state_validator/occupancy_map.h"
#include "utils/make_ref_enabler.h"

#include <limits>
#include <cmath>
#include <tuple>

namespace Planner {

	int SquaredDistance(const GridCellPosition& a, const GridCellPosition& b)
	{
		int dr = a.row - b.row;
		int dc = a.col - b.col;
		return dr * dr + dc * dc;
	}

	GVD::ObstacleDistanceMap::ObstacleDistanceMap(const Ref<Grid<int>>& occupancy, float resolution) :
		rows(occupancy->rows), columns(occupancy->columns), resolution(resolution),
		m_distance(occupancy->rows, occupancy->columns, std::numeric_limits<int>::max()),
		m_obstacle(occupancy->rows, occupancy->columns, GridCellPosition(-1, -1)),
		m_toRaise(occupancy->rows, occupancy->columns, false),
		m_toProcess(occupancy->rows, occupancy->columns, false),
		m_voro(occupancy->rows, occupancy->columns, true),
		m_occupancy(occupancy)
	{
	}

	void GVD::ObstacleDistanceMap::Update()
	{
		while (!m_open.empty()) {
			const auto s = m_open.top().position;
			m_open.pop();
			if (!m_toProcess[s.row][s.col])
				continue;
			if (m_toRaise[s.row][s.col]) {
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (m_obstacle[n.row][n.col].IsValid() && !m_toRaise[n.row][n.col]) {
						if (!IsOccupied(m_obstacle[n.row][n.col])) {
							m_distance[n.row][n.col] = std::numeric_limits<int>::max();
							m_obstacle[n.row][n.col] = GridCellPosition(-1, -1);
							m_toRaise[n.row][n.col] = true;
						}
						m_open.push({ n, m_distance[n.row][n.col] });
						m_toProcess[n.row][n.col] = true;
					}
				}
				m_toRaise[s.row][s.col] = false;
			} else if (IsOccupied(m_obstacle[s.row][s.col])) {
				m_voro[s.row][s.col] = false;
				if (m_voronoiMap)
					m_voronoiMap->UnsetEdge(s);
				m_toProcess[s.row][s.col] = false;
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (!m_toRaise[n.row][n.col]) {
						auto d = SquaredDistance(m_obstacle[s.row][s.col], n);
						if (d < m_distance[n.row][n.col]) {
							m_distance[n.row][n.col] = d;
							m_obstacle[n.row][n.col] = m_obstacle[s.row][s.col];
							m_open.push({ n, d });
							m_toProcess[n.row][n.col] = true;
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
		m_obstacle[s.row][s.col] = s;
		m_distance[s.row][s.col] = 0.0f;
		m_open.push({ s, 0 });
		m_toProcess[s.row][s.col] = true;
	}

	void GVD::ObstacleDistanceMap::UnsetObstacle(const GridCellPosition& s)
	{
		m_distance[s.row][s.col] = std::numeric_limits<int>::max();
		m_obstacle[s.row][s.col] = GridCellPosition(-1, -1);
		m_toRaise[s.row][s.col] = true;
		m_open.push({ s, std::numeric_limits<int>::max() });
		m_toProcess[s.row][s.col] = true;
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
		return (m_distance[s.row][s.col] > 1 || m_distance[n.row][n.col] > 1) && obstN.IsValid() && obstN != obstS && !obstS.IsAdjacentTo(obstN);
	}

	void GVD::ObstacleDistanceMap::CheckVoro(const GridCellPosition& s, const GridCellPosition& n)
	{
		auto obstS = m_obstacle[s.row][s.col];
		auto obstN = m_obstacle[n.row][n.col];

		if ((*m_occupancy)[obstS.row][obstS.col] == (*m_occupancy)[obstN.row][obstN.col])
			return;

		if ((m_distance[s.row][s.col] > 1 || m_distance[n.row][n.col] > 1) && m_obstacle[n.row][n.col].IsValid()) {
			if (std::abs(obstS.row - obstN.row) > 1 || std::abs(obstS.col - obstN.col) > 1) {
				auto sObstN = SquaredDistance(s, obstN);
				auto nObstS = SquaredDistance(n, obstS);
				auto sStability = sObstN - m_distance[s.row][s.col];
				auto nStability = nObstS - m_distance[n.row][n.col];
				if (sStability < 0 || nStability < 0)
					return;
				if (sStability <= nStability) {
					m_voro[s.row][s.col] = true;
					m_voronoiMap->SetEdge(s);
				}
				if (nStability <= sStability) {
					m_voro[n.row][n.col] = true;
					m_voronoiMap->SetEdge(n);
				}
			}
		}
	}

	void GVD::ObstacleDistanceMap::RebuildVoronoi()
	{
		// TODO use this function
		while (!m_voroQ.empty()) {
			const auto s = m_voroQ.top().position;

			if (PatternMatch(s))
				continue;

			bool stop = false;
			auto obstS = m_obstacle[s.row][s.row];
			auto neighbors = s.GetNeighbors(rows, columns);
			for (auto& n : neighbors) {
				auto obstN = m_obstacle[n.row][n.col];
				if (CheckVoroConditions(s, n, obstS, obstN) && (*m_occupancy)[obstS.row][obstS.col] != (*m_occupancy)[obstN.row][obstN.col]) {
					stop = true;
					break;
				}
			}

			if (stop)
				continue;

			m_voro[s.row][s.col] = false;
			for (auto& n : neighbors)
				if (m_voro[n.row][n.col])
					m_voroQ.push({ n, m_distance[n.row][n.col] });
		}
	}

	bool GVD::ObstacleDistanceMap::PatternMatch(const GridCellPosition& s)
	{
		if (s.row < 1 || s.row >= rows - 1 || s.col < 1 || s.col >= columns - 1)
			return false;

		if (m_voro[s.row - 1][s.col - 1] && !m_voro[s.row][s.col - 1] && !m_voro[s.row - 1][s.col])
			return true;
		if (m_voro[s.row + 1][s.col - 1] && !m_voro[s.row][s.col - 1] && !m_voro[s.row + 1][s.col])
			return true;
		if (m_voro[s.row - 1][s.col + 1] && !m_voro[s.row][s.col + 1] && !m_voro[s.row - 1][s.col])
			return true;
		if (m_voro[s.row + 1][s.col + 1] && !m_voro[s.row][s.col + 1] && !m_voro[s.row + 1][s.col])
			return true;

		// clang-format off
		if ( m_voro[s.row][s.col - 1] &&  m_voro[s.row][s.col + 1] && !m_voro[s.row - 1][s.col] && !m_voro[s.row + 1][s.col]) return true;
		if (!m_voro[s.row][s.col - 1] && !m_voro[s.row][s.col + 1] &&  m_voro[s.row - 1][s.col] &&  m_voro[s.row + 1][s.col]) return true;
		if ( m_voro[s.row][s.col - 1] &&  m_voro[s.row][s.col + 1] &&  m_voro[s.row - 1][s.col] &&  m_voro[s.row + 1][s.col]) return true;
		// clang-format on

		return false;
	}

	bool GVD::ObstacleDistanceMap::IsOccupied(const GridCellPosition& s)
	{
		return s.IsValid() && m_obstacle[s.row][s.col] == s;
	}

	GVD::VoronoiDistanceMap::VoronoiDistanceMap(unsigned int rows, unsigned int columns, float resolution) :
		rows(rows), columns(columns), resolution(resolution),
		m_distance(rows, columns, std::numeric_limits<int>::max()),
		m_edge(rows, columns, GridCellPosition(-1, -1)),
		m_toRaise(rows, columns, false),
		m_toProcess(rows, columns, false)
	{
	}

	void GVD::VoronoiDistanceMap::Update()
	{
		while (!m_open.empty()) {
			const auto s = m_open.top().position;
			m_open.pop();
			if (!m_toProcess[s.row][s.col])
				continue;
			if (m_toRaise[s.row][s.col]) {
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (m_edge[n.row][n.col].IsValid() && !m_toRaise[n.row][n.col]) {
						if (!IsOccupied(m_edge[n.row][n.col])) {
							m_distance[n.row][n.col] = std::numeric_limits<int>::max();
							m_edge[n.row][n.col] = GridCellPosition(-1, -1);
							m_toRaise[n.row][n.col] = true;
						}
						m_open.push({ n, m_distance[n.row][n.col] });
						m_toProcess[n.row][n.col] = true;
					}
				}
				m_toRaise[s.row][s.col] = false;
			} else if (IsOccupied(m_edge[s.row][s.col])) {
				m_toProcess[s.row][s.col] = false;
				for (auto& n : s.GetNeighbors(rows, columns)) {
					if (!m_toRaise[n.row][n.col]) {
						auto d = SquaredDistance(m_edge[s.row][s.col], n);
						if (d < m_distance[n.row][n.col]) {
							m_distance[n.row][n.col] = d;
							m_edge[n.row][n.col] = m_edge[s.row][s.col];
							m_open.push({ n, d });
							m_toProcess[n.row][n.col] = true;
						}
					}
				}
			}
		}
	}

	void GVD::VoronoiDistanceMap::SetEdge(const GridCellPosition& s)
	{
		m_edge[s.row][s.col] = s;
		m_distance[s.row][s.col] = 0;
		m_open.push({ s, 0 });
		m_toProcess[s.row][s.col] = true;
	}

	void GVD::VoronoiDistanceMap::UnsetEdge(const GridCellPosition& s)
	{
		m_distance[s.row][s.col] = std::numeric_limits<int>::max();
		m_edge[s.row][s.col] = GridCellPosition(-1, -1);
		m_toRaise[s.row][s.col] = true;
		m_open.push({ s, std::numeric_limits<int>::max() });
		m_toProcess[s.row][s.col] = true;
	}

	bool GVD::VoronoiDistanceMap::IsOccupied(const GridCellPosition& s)
	{
		return s.IsValid() && m_edge[s.row][s.col] == s;
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
					m_data[r * columns + c] = 0.0f;
				} else {
					m_data[r * columns + c] = (alpha / (alpha + obstDist))
						* (voroDist / (obstDist + voroDist))
						* (std::pow(obstDist - dMax, 2) / std::pow(dMax, 2));
				}
			}
		}
	}

	GVD::GVD(const Ref<OccupancyMap>& map) :
		rows(map->Rows()), columns(map->Columns()), resolution(map->resolution),
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

	bool GVD::GetNearestObstaclePosition(const Point2d& position, Point2d& obstacle) const
	{
		auto cell = m_occupancyMap->WorldPositionToGridCell(position, false);
		if (!m_occupancyMap->IsInsideMap(cell))
			return false;
		obstacle = m_occupancyMap->GridCellToWorldPosition(GetNearestObstacleCell(cell));
		return true;
	}

	bool GVD::GetNearestVoronoiEdgePosition(const Point2d& position, Point2d& voronoi) const
	{
		auto cell = m_occupancyMap->WorldPositionToGridCell(position, false);
		if (!m_occupancyMap->IsInsideMap(cell))
			return false;
		voronoi = m_occupancyMap->GridCellToWorldPosition(GetNearestVoronoiEdgeCell(cell));
		return true;
	}

	bool GVD::GetDistanceToNearestObstacle(const Point2d& position, float& distance) const
	{
		auto cell = m_occupancyMap->WorldPositionToGridCell(position, false);
		if (!m_occupancyMap->IsInsideMap(cell))
			return false;
		distance = GetDistanceToNearestObstacle(cell);
		return true;
	}

	bool GVD::GetDistanceToNearestVoronoiEdge(const Point2d& position, float& distance) const
	{
		auto cell = m_occupancyMap->WorldPositionToGridCell(position, false);
		if (!m_occupancyMap->IsInsideMap(cell))
			return false;
		distance = GetDistanceToNearestVoronoiEdge(cell);
		return true;
	}

	bool GVD::GetPathCost(const Point2d& position, float& cost) const
	{
		auto cell = m_occupancyMap->WorldPositionToGridCell(position, false);
		if (!m_occupancyMap->IsInsideMap(cell))
			return false;
		cost = GetPathCost(cell);
		return true;
	}

	inline std::tuple<int, int, int> HSVToRGB(float H, float S, float V)
	{
		H *= 360;
		S *= 100;
		V *= 100;

		float s = S / 100;
		float v = V / 100;
		float C = s * v;
		float X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
		float m = v - C;
		float r, g, b;

		if (H >= 0 && H < 60)
			r = C, g = X, b = 0;
		else if (H >= 60 && H < 120)
			r = X, g = C, b = 0;
		else if (H >= 120 && H < 180)
			r = 0, g = C, b = X;
		else if (H >= 180 && H < 240)
			r = 0, g = X, b = C;
		else if (H >= 240 && H < 300)
			r = X, g = 0, b = C;
		else
			r = C, g = 0, b = X;

		return { (r + m) * 255, (g + m) * 255, (b + m) * 255 };
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
		int numObstacles = 0;
		for (int x = 0; x < sizeX; x++) {
			for (int y = 0; y < sizeY; y++) {
				numObstacles = std::max(numObstacles, m_occupancyMap->GetOccupancyValue(x, y));
			}
		}

		fprintf(F, "P6\n#\n");
		fprintf(F, "%d %d\n255\n", sizeX, sizeY);
		for (int y = sizeY - 1; y >= 0; y--) {
			for (int x = 0; x < sizeX; x++) {
				// Obstacle
				if (m_occupancyMap->IsOccupied({ x, y })) {
					fputc(0, F);
					fputc(0, F);
					fputc(0, F);
					continue;
				}
				// Path cost map
				auto obstacleCell = m_obstacleMap->m_obstacle[x][y];
				if (!obstacleCell.IsValid()) {
					// Empty map
					fputc(0, F);
					fputc(0, F);
					fputc(0, F);
					continue;
				} else {
					float h = m_occupancyMap->GetOccupancyValue(obstacleCell) / (float)(numObstacles + 1);
					float l = (1.0 - m_pathCostMap[x][y]);
					l = std::max(0.0f, std::min(l, 1.0f));
					auto [r, g, b] = HSVToRGB(h, 1.0f, l);
					fputc((unsigned char)r, F);
					fputc((unsigned char)g, F);
					fputc((unsigned char)b, F);
					continue;
				}
			}
		}
		fclose(F);
	}
}
