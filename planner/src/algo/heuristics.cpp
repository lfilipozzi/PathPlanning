#include "algo/heuristics.h"
#include "state_validator/occupancy_map.h"
#include "utils/make_ref_enabler.h"
#include "geometry/reeds_shepp.h"

namespace Planner {
	NonHolonomicHeuristic::NonHolonomicHeuristic(double spatialResolution, double angularResolution, double minTurningRadius,
		double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost,
		unsigned int numSpatialX, unsigned int numSpatialY) :
		spatialResolution(spatialResolution),
		angularResolution(angularResolution), minTurningRadius(minTurningRadius),
		reverseCostMultiplier(reverseCostMultiplier), forwardCostMultiplier(forwardCostMultiplier), directionSwitchingCost(directionSwitchingCost),
		numSpatialX(numSpatialX), numSpatialY(numSpatialY), numAngular(ceil(2 * M_PI / angularResolution)),
		offsetX(floor(numSpatialX / 2.0) * spatialResolution), offsetY(floor(numSpatialY / 2.0) * spatialResolution)
	{
		m_values = new double**[numSpatialX];
		for (unsigned int i = 0; i < numSpatialX; ++i) {
			m_values[i] = new double*[numSpatialY];
			for (unsigned int j = 0; j < numSpatialY; j++) {
				m_values[i][j] = new double[numAngular];
			}
		}
	}

	NonHolonomicHeuristic::~NonHolonomicHeuristic()
	{
		for (unsigned int i = 0; i < numSpatialX; i++) {
			for (unsigned int j = 0; j < numSpatialY; j++) {
				delete[] m_values[i][j];
			}
			delete[] m_values[i];
		}
		delete[] m_values;
	}

	Ref<NonHolonomicHeuristic> NonHolonomicHeuristic::Build(const std::array<Pose2d, 2>& bounds,
		double spatialResolution, double angularResolution, double minTurningRadius,
		double reverseCostMultiplier, double forwardCostMultiplier, double directionSwitchingCost)
	{
		PP_INFO("Generating non-holonomic heuristic without obstacle.");

		const double spatialSizeX = bounds[1].x() - bounds[0].x();
		const double spatialSizeY = bounds[1].y() - bounds[0].y();

		unsigned int numSpatialGuessX = ceil(spatialSizeX / spatialResolution);
		if (numSpatialGuessX % 2 == 0)
			numSpatialGuessX++;
		unsigned int numSpatialGuessY = ceil(spatialSizeY / spatialResolution);
		if (numSpatialGuessY % 2 == 0)
			numSpatialGuessY++;

		Ref<NonHolonomicHeuristic> heuristic = makeRef<MakeRefEnabler<NonHolonomicHeuristic>>(
			spatialResolution, angularResolution, minTurningRadius,
			reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost,
			numSpatialGuessX, numSpatialGuessY);
		double***& values = heuristic->m_values;
		const double& offsetX = heuristic->offsetX;
		const double& offsetY = heuristic->offsetY;
		const unsigned int& numSpatialX = heuristic->numSpatialX;
		const unsigned int& numSpatialY = heuristic->numSpatialY;
		const unsigned int& numAngular = heuristic->numAngular;

		const Pose2d goal(0.0, 0.0, 0.0);
		for (int i = 0; i < numSpatialX; i++) {
			for (int j = 0; j < numSpatialY; j++) {
				for (int k = 0; k < numAngular; k++) {
					Pose2d pose(i * spatialResolution - offsetX, j * spatialResolution - offsetY, k * angularResolution);
					auto path = ReedsShepp::Solver::GetOptimalPath(pose, goal, minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
					values[i][j][k] = path.ComputeCost(minTurningRadius, reverseCostMultiplier, forwardCostMultiplier, directionSwitchingCost);
				}
			}
		}

		PP_INFO("Non-holonomic heuristic without obstacle generated.");
		return heuristic;
	}

	double NonHolonomicHeuristic::GetHeuristicValue(const Pose2d& from, const Pose2d& to)
	{
		auto delta = to - from;
		delta.theta = delta.WrapTheta();

		int i = (int)round((delta.x() + offsetX) / spatialResolution);
		int j = (int)round((delta.y() + offsetY) / spatialResolution);
		int k = (int)round(delta.theta / angularResolution);
		if (k == numAngular)
			k = 0;

		if (i < 0 || i >= numSpatialX || j < 0 || j >= numSpatialY) {
			double euclideanDistance = delta.position.norm();
			double distanceMultiplier = std::min(reverseCostMultiplier, forwardCostMultiplier);
			return distanceMultiplier * euclideanDistance;
		}

		return m_values[i][j][k];
	}

	ObstaclesHeuristic::ObstaclesHeuristic(const Ref<OccupancyMap>& map, double reverseCostMultiplier, double forwardCostMultiplier) :
		diagonalResolution(std::sqrt(2) * map->resolution),
		costMultiplier(std::min(reverseCostMultiplier, forwardCostMultiplier) * map->resolution),
		m_map(map),
		m_cost(map->Rows(), map->Columns(), std::numeric_limits<float>::infinity()),
		m_explored(map->Rows(), map->Columns(), false)
	{
	}

	void ObstaclesHeuristic::Update(const Pose2d& goal)
	{
		for (int r = 0; r < m_map->Rows(); r++) {
			for (int c = 0; c < m_map->Columns(); c++) {
				m_cost[r][c] = std::numeric_limits<float>::infinity();
				m_explored[r][c] = false;
			}
		}

		GridCellPosition start = m_map->WorldPositionToGridCell(goal.position);
		if (!start.IsValid())
			return;

		Frontier<GridCell<float>, CompareCell, HashCell, EqualCell> frontier;
		frontier.Push({ start, 0.0f });
		m_cost[start.row][start.col] = 0.0f;

		while (!frontier.Empty()) {
			const auto cell = frontier.Pop().position;
			m_explored[cell.row][cell.col] = true;

			for (auto& n : cell.GetNeighbors(m_map->Rows(), m_map->Columns())) {
				if (m_map->IsOccupied(n))
					continue;
				if (n.IsDiagonalTo(cell))
					if (m_map->IsOccupied({ n.row, cell.col }) && m_map->IsOccupied({ cell.row, n.col }))
						continue;

				float transitionCost = cell.row == n.row || cell.col == n.col ? 1.0f : std::sqrt(2.0f);
				float pathCost = transitionCost + m_cost[cell.row][cell.col];

				GridCell<float> const* inFrontier = frontier.Find({ n, 0.0f });
				bool inExplored = m_explored[n.row][n.col];
				if (!inFrontier && !inExplored) {
					frontier.Push({ n, pathCost });
					m_cost[n.row][n.col] = pathCost;
				} else if (inFrontier) {
					// Check if the node in frontier has a higher cost than the
					// current path, and if so replace it by child
					if (inFrontier->value > m_cost[n.row][n.col]) {
						frontier.Remove({ n, 0.0f });
						frontier.Push({ n, pathCost });
						m_cost[n.row][n.col] = pathCost;
					}
				}
			}
		}
	}

	double ObstaclesHeuristic::GetHeuristicValue(const Pose2d& from, const Pose2d& to)
	{
		auto euclidean = (to.position - from.position).norm();
		auto cell = m_map->WorldPositionToGridCell(from.position);
		if (!cell.IsValid())
			return euclidean;
		if (!m_explored[cell.row][cell.col])
			return euclidean;
		double heuristic = m_cost[cell.row][cell.col] * costMultiplier - diagonalResolution;
		return std::max(heuristic, euclidean);
	}

	void ObstaclesHeuristic::Visualize(const std::string& filename) const
	{
		FILE* F = fopen(filename.c_str(), "w");
		if (!F) {
			PP_ERROR("Could not open {}!", filename);
			return;
		}

		const auto& sizeX = m_cost.rows;
		const auto& sizeY = m_cost.columns;
		float maxCost = -std::numeric_limits<float>::infinity();
		for (int x = 0; x < sizeX; x++) {
			for (int y = 0; y < sizeY; y++) {
				if (m_cost[x][y] != std::numeric_limits<float>::infinity())
					maxCost = std::max(maxCost, m_cost[x][y]);
			}
		}
		fprintf(F, "P6\n#\n");
		fprintf(F, "%d %d\n255\n", sizeX, sizeY);
		for (int y = sizeY - 1; y >= 0; y--) {
			for (int x = 0; x < sizeX; x++) {
				unsigned char c = 0;
				if (!m_explored[x][y]) {
					// Obstacle
					fputc(0, F);
					fputc(0, F);
					fputc(0, F);
				} else {
					// Path cost map
					float f = ((maxCost - m_cost[x][y]) / maxCost * 255);
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
