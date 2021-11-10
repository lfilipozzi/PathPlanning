#include "algo/smoother.h"
#include "core/base.h"
#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/occupancy_map.h"
#include "state_validator/gvd.h"
#include "utils/maths.h"

namespace Planner {

	Point2d OrthogonalComplement(const Point2d& a, const Point2d& b)
	{
		return a - (a.x() * b.x() + a.y() * b.y()) * b / b.squaredNorm();
	}

	Smoother::Smoother() :
		m_param(1.0f) { }

	bool Smoother::Initialize(const Ref<StateValidatorOccupancyMap>& validator, const Ref<GVD>& gvd, float maxCurvature)
	{
		if (!validator || !gvd) {
			isInitialized = false;
			return false;
		}

		m_param = Parameters(maxCurvature);
		m_validator = validator;
		m_map = validator->GetOccupancyMap();
		m_gvd = gvd;
		isInitialized = true;
		return true;
	}

	std::vector<Smoother::State> Smoother::Smooth(const std::vector<State>& path)
	{
		if (!isInitialized) {
			PP_ERROR("The algorithm has not been initialized successfully.");
			return std::vector<Smoother::State>();
		}

		return Smooth(path, nullptr);
	}

	std::vector<Smoother::State> Smoother::Smooth(const std::vector<State>& path, Scope<std::unordered_set<int>> unsafeIndices)
	{
		if (path.size() < 5) {
			return path;
		}

		m_currentCount = path.size();
		int num = m_currentCount;

		std::vector<State> currentPath = path;

		const float unsafeRadius = m_validator->minSafeRadius * (1 + m_param.collisionRatio);
		int count = 0;
		float step = m_param.stepTolerance;
		while (step >= m_param.stepTolerance && count < m_param.maxIterations) {
			for (int i = 2; i < num - 2; i++) {
				// Do not modify this point if it is a cust point of if it is adjacent to a cusp point
				if (currentPath[i - 2].direction != currentPath[i - 1].direction)
					continue;
				if (currentPath[i - 1].direction != currentPath[i].direction)
					continue;
				if (currentPath[i].direction != currentPath[i + 1].direction)
					continue;
				if (currentPath[i + 1].direction != currentPath[i + 2].direction)
					continue;

				// Do not modify this point if it is unsafe
				if (unsafeIndices ? unsafeIndices->find(i) != unsafeIndices->end() : false)
					continue;

				const Point2d& curr = currentPath[i].Position();
				Point2d gradient = { 0.0, 0.0 };

				// Original path term
				gradient += m_param.pathWeight * (path[i].Position() - curr);

				if (m_map->IsInsideMap(curr)) {
					// Collision term
					Point2d dirToClosestObs = curr - m_map->GridCellToWorldPosition(m_gvd->GetNearestObstacleCell(m_map->WorldPositionToGridCell(curr)));
					float obstDist = dirToClosestObs.norm();
					if (obstDist < unsafeRadius)
						gradient -= m_param.collisionWeight * (obstDist - unsafeRadius) * dirToClosestObs / obstDist;

					// Voronoi term
					const auto& alpha = m_gvd->alpha;
					const auto& dMax = m_gvd->dMax;
					if (obstDist < dMax && m_param.voronoiWeight > 0.0f) {
						Point2d dirToClosestVoroEdge = curr - m_map->GridCellToWorldPosition(m_gvd->GetNearestVoronoiEdgeCell(m_map->WorldPositionToGridCell(curr)));
						float voroDist = dirToClosestVoroEdge.norm();

						if (voroDist > 0.0f) {
							float alphaPlusObstDist = alpha + obstDist;
							float obstDistMinusDMax = obstDist - dMax;
							float obstDistPlusVoroDist = obstDist + voroDist;
							float dMaxSquared = dMax * dMax;
							float pvdv = (alpha / alphaPlusObstDist) * (obstDistMinusDMax * obstDistMinusDMax / dMaxSquared) * (obstDist / (obstDistPlusVoroDist * obstDistPlusVoroDist));
							float pvdo = (alpha / alphaPlusObstDist) * (voroDist / obstDistPlusVoroDist) * (obstDistMinusDMax / dMaxSquared) * (-obstDistMinusDMax / alphaPlusObstDist - obstDistMinusDMax / obstDistPlusVoroDist + 2);

							gradient -= m_param.voronoiWeight * (pvdo * dirToClosestObs / obstDist + pvdv * dirToClosestVoroEdge / voroDist);
						}
					}
				}

				// Smoothing term
				gradient -= m_param.smoothWeight * (currentPath[i - 2].Position() - 4 * currentPath[i - 1].Position() + 6 * curr - 4 * currentPath[i + 1].Position() + currentPath[i + 2].Position());

				// Curvature term
				gradient -= m_param.curvatureWeight * CalculateCurvatureTerm(currentPath[i - 1].Position(), currentPath[i].Position(), currentPath[i + 1].Position());

				gradient /= m_param.collisionWeight + m_param.smoothWeight + m_param.voronoiWeight + m_param.pathWeight + m_param.curvatureWeight;

				currentPath[i].Position() = curr + m_param.learningRate * gradient;

				step = gradient.norm();
			}

			count++;
		}

		if (!unsafeIndices) {
			Scope<std::unordered_set<int>> unsafe = makeScope<std::unordered_set<int>>(CheckPath(currentPath));
			if (unsafe->size() > 0)
				currentPath = Smooth(currentPath, std::move(unsafe));
		}

		return currentPath;
	}

	Point2d Smoother::CalculateCurvatureTerm(const Point2d& xim1, const Point2d& xi, const Point2d& xip1) const
	{
		Point2d dxi = xi - xim1;
		Point2d dxip1 = xip1 - xi;

		float dphi, ddphi;
		Point2d p1, p2;
		dphi = acos(std::clamp<float>((dxi.x() * dxip1.x() + dxi.y() * dxip1.y()) / (dxi.norm() * dxip1.norm()), -1.0f, 1.0f));
		float k = dphi / dxi.norm();
		if (k <= m_param.maxCurvature)
			return { 0.0, 0.0 };

		ddphi = (float)(-1 / sqrt(1 - pow(cos(dphi), 2)));
		float denom = xi.norm() * xip1.norm();
		p1 = OrthogonalComplement(xi, -xip1) / denom;
		p2 = OrthogonalComplement(-xip1, xi) / denom;

		float coeff1 = -1 / dxi.norm() * ddphi;
		float coeff2 = dphi / dxi.squaredNorm();

		Point2d ki, kim1, kip1;
		ki = coeff1 * (-p1 - p2) - coeff2 * Point2d(1.0, 1.0);
		kim1 = coeff1 * p2 + coeff2 * Point2d(1.0, 1.0);
		kip1 = coeff1 * p1;

		return (k - m_param.maxCurvature) * (0.25f * kim1 + 0.5f * ki + 0.25f * kip1);
	}

	std::unordered_set<int> Smoother::CheckPath(const std::vector<State>& path) const
	{
		std::unordered_set<int> unsafeIndices;

		int count = path.size();

		for (int i = 2; i < count - 2; i++) {
			if (path[i - 2].direction != path[i - 1].direction)
				continue;
			if (path[i - 1].direction != path[i].direction)
				continue;
			if (path[i].direction != path[i + 1].direction)
				continue;
			if (path[i + 1].direction != path[i + 2].direction)
				continue;

			auto& currPos = path[i].Position();
			auto& prevPos = path[i - 1].Position();
			auto displacement = 0.25f * (currPos - prevPos) + 0.75f * (currPos - currPos);
			float pathOrientation = atan2(displacement.y(), displacement.x());

			if (!m_validator->IsStateValid({ currPos, pathOrientation })) {
				unsafeIndices.insert(i);
				unsafeIndices.insert(i - 1);
			}
		}

		return unsafeIndices;
	}
}
