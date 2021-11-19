#include "algo/smoother.h"
#include "core/base.h"
#include "state_validator/state_validator_occupancy_map.h"
#include "state_validator/occupancy_map.h"
#include "state_validator/gvd.h"
#include "utils/maths.h"

namespace Planner {

	Point2d OrthogonalComplement(const Point2d& a, const Point2d& b)
	{
		return a - a.dot(b) * b / b.squaredNorm();
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

	Smoother::Status Smoother::Smooth(const std::vector<Pose2d>& path, const std::unordered_set<int>& cuspIndices)
	{
		Status status = Status::Failure;

		if (!isInitialized) {
			PP_ERROR("The algorithm has not been initialized successfully.");
			return status = Status::Failure;
		}

		if (path.size() < 5) {
			m_currentPath = path;
			if (!IsPathSafe())
				return status = Status::Failure;
			return status = Status::PathSize;
		}

		const int num = path.size();

		// List indices of points to optimize
		std::vector<int> indices;
		indices.reserve(num);
		for (int i = 0; i < num - 4; i++) {
			if (cuspIndices.find(i + 4) != cuspIndices.end()) {
				i += 3;
				continue;
			}
			if (cuspIndices.find(i + 3) != cuspIndices.end()) {
				i += 2;
				continue;
			}
			if (cuspIndices.find(i + 2) != cuspIndices.end()) {
				i += 1;
				continue;
			}
			if (cuspIndices.find(i + 1) != cuspIndices.end()) {
				continue;
			}
			if (cuspIndices.find(i) != cuspIndices.end()) {
				continue;
			}
			indices.push_back(i + 2);
		}

		m_currentPath = path;
		std::vector<Point2d> gradients;
		gradients.resize(num);

		const float unsafeRadius = m_validator->minSafeRadius * (1 + m_param.collisionRatio);
		int count = -1;
		float step = m_param.stepTolerance;
		while (true) {
			count++;
			if (count >= m_param.maxIterations) {
				status = Status::MaxIteration;
				break;
			}
			if (step < m_param.stepTolerance) {
				status = Status::StepTolerance;
				break;
			}

			// Reset gradients
			std::fill(gradients.begin(), gradients.end(), Point2d(0.0, 0.0));

			// Compute gradients
			for (int i : indices) {
				PP_ASSERT(i >= 2 && i < num - 2, "Invalid index");

				const Point2d& curr = m_currentPath[i].position;

				// Original path term
				gradients[i] += m_param.pathWeight * (path[i].position - curr);

				if (m_map->IsInsideMap(curr)) {
					// Collision term
					Point2d dirToClosestObs = curr - m_map->GridCellToWorldPosition(m_gvd->GetNearestObstacleCell(m_map->WorldPositionToGridCell(curr)));
					float obstDist = dirToClosestObs.norm();
					if (obstDist < unsafeRadius)
						gradients[i] += m_param.collisionWeight * (obstDist - unsafeRadius) * dirToClosestObs / obstDist;

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

							gradients[i] += m_param.voronoiWeight * (pvdo * dirToClosestObs / obstDist + pvdv * dirToClosestVoroEdge / voroDist);
						}
					}
				}

				// Smoothing term
				gradients[i] += m_param.smoothWeight * (m_currentPath[i - 2].position - 4 * m_currentPath[i - 1].position + 6 * curr - 4 * m_currentPath[i + 1].position + m_currentPath[i + 2].position);

				// Curvature term
				CalculateCurvatureTerm(m_currentPath[i - 1].position, m_currentPath[i].position, m_currentPath[i + 1].position, gradients[i - 1], gradients[i], gradients[i + 1]);
			}

			// Apply gradients
			step = 0.0;
			for (int i = 0; i < num; i++) {
				m_currentPath[i].position = m_currentPath[i].position - m_param.learningRate * gradients[i];
				step = std::max<float>(step, gradients[i].norm());
			}
		}

		// TODO update value of theta for each pose

		if (!IsPathSafe())
			status = Status::Failure;

		return status;
	}

	void Smoother::CalculateCurvatureTerm(const Point2d& xim1, const Point2d& xi, const Point2d& xip1, Point2d& gim1, Point2d& gi, Point2d& gip1) const
	{
		Point2d deltaXi = xi - xim1;
		Point2d deltaXip1 = xip1 - xi;
		float deltaPhi = acos(std::clamp<float>((deltaXi.normalized().dot(deltaXip1.normalized())), -1.0f, 1.0f));

		// The curvature is approximated by deltaPhi / norm_deltaXi
		float kappa = deltaPhi / deltaXi.norm();
		if (kappa <= m_param.maxCurvature) {
			return;
		}

		// The partial derivative of the curvature with respect to the point X
		// is as follows:
		// Dkappa_DX = 1 / norm_deltaXi * DdeltaPhi_DX - deltaPhi / norm_deltaXi^2 * Dnorm_deltaXi_DX
		// * The first gradient term DdeltaPhi_DX can be computed as follows:
		//   DdeltaPhi_DX = DdeltaPhi_DcosDeltaPhi * DcosDeltaPhi_DX according to the chain rule
		//   * The scalar DdeltaPhi_DcosDeltaPhi is:
		//     DdeltaPhi_DcosDeltaPhi = -1 / sqrt(1 - cos(deltaPhi)^2)
		//   * To compute the gradient term DcosDeltaPhi_DX, we use the chain rule
		//     DcosDeltaPhi_DX = DdeltaXi_DX * DcosDeltaPhi_DdeltaXi + DdeltaXip1_DX * DcosDeltaPhi_DdeltaXip1
		//     where DdeltaXi_DX = -I for X = xim1, I for X = xi, and 0 for X = xip1
		//     and DdeltaXip1_DX = 0 for X = xim1, -I for X = xi, and I for X = xip1
		//     * Using the following equality:
		//     cosDeltaPhi = deltaXi . deltaXip1 / (norm_deltaXi * norm_deltaXip1), we have:
		//     * DcosDeltaPhi_DdeltaXi = 1 / norm_deltaXip1 * (D(deltaXi . deltaXip1)_DdeltaXi * 1/norm_deltaXi
		//                             + (deltaXi . deltaXip1) * D(1/norm_deltaXi)_DdeltaXi)
		//                             = 1 / norm_deltaXip1 * (deltaXip1 / norm_deltaXi
		//                             - (deltaXi . deltaXip1) / norm_deltaXi^3 * deltaXi)
		//                             = 1 / (norm_deltaXi * norm_deltaXip1) * (deltaXip1
		//                             - (deltaXi . deltaXip1) / norm_deltaXi^2 * deltaXi)
		//     * Similarly DcosDeltaPhi_DdeltaXi = 1 / (norm_deltaXi * norm_deltaXip1) * (deltaXi
		//                             - (deltaXi . deltaXip1) / norm_deltaXip1^2 * deltaXip1)
		// * The second gradient Dnorm_deltaXi_DX can be computed as follows:
		//   Dnorm_deltaXi_DX = DdeltaXi_DX * Dnorm_deltaXi_DdeltaXi according to the chain rule
		//   and Dnorm_deltaXi_DdeltaXi = deltaXi / norm_deltaXi
		// clang-format off
		float denominator = deltaXi.norm() * deltaXip1.norm();
		Point2d DcosDeltaPhi_DdeltaXi   = OrthogonalComplement(deltaXip1, deltaXi  ) / denominator;
		Point2d DcosDeltaPhi_DdeltaXip1 = OrthogonalComplement(deltaXi,   deltaXip1) / denominator;
		float DdeltaPhi_DcosDeltaPhi = -1.0f / sqrt(1.0f - std::pow(cos(deltaPhi), 2));
		float coef1 = 1 / deltaXi.norm() * DdeltaPhi_DcosDeltaPhi;
		Point2d coef2 = deltaPhi / deltaXi.squaredNorm() * deltaXi.normalized();
		Point2d DcosDeltaPhi_Dxim1 = -DcosDeltaPhi_DdeltaXi;
		Point2d DcosDeltaPhi_Dxi   =  DcosDeltaPhi_DdeltaXi - DcosDeltaPhi_DdeltaXip1;
		Point2d DcosDeltaPhi_Dxip1 =  DcosDeltaPhi_DdeltaXip1;
		Point2d Dkappa_Dxim1 = coef1 * DcosDeltaPhi_Dxim1 + coef2;
		Point2d Dkappa_Dxi   = coef1 * DcosDeltaPhi_Dxi   - coef2;
		Point2d Dkappa_Dxip1 = coef1 * DcosDeltaPhi_Dxip1;

		gim1 += m_param.curvatureWeight * (kappa - m_param.maxCurvature) * Dkappa_Dxim1;
		gi   += m_param.curvatureWeight * (kappa - m_param.maxCurvature) * Dkappa_Dxi;
		gip1 += m_param.curvatureWeight * (kappa - m_param.maxCurvature) * Dkappa_Dxip1;
		// clang-format on
	}

	bool Smoother::IsPathSafe() const
	{
		for (auto& pose : m_currentPath) {
			if (!m_validator->IsStateValid(pose)) {
				return false;
			}
		}

		return true;
	}
}
