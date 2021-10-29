#pragma once

#include "core/base.h"
#include "geometry/2dplane.h"
#include "paths/path.h"
#include <vector>
#include <unordered_set>

namespace Planner {
	class StateValidatorOccupancyMap;
	class OccupancyMap;
	class GVD;

	/// @brief Smooths a path.
	/// @details This class used iterative gradient descent to optimize the path
	/// with regards to several terms: smoothness, proximity to obstacles,
	/// proximity to the Voronoi edge, and curvature.
	class Smoother {
	public:
		struct Parameters {
			/// @brief Lower bound on the size of a step. If the solver attempts to
			/// take a step that is smaller than StepTolerance, the iterations end.
			float stepTolerance = 1e-3;
			/// @brief Bound on the number of solver iterations.
			int maxIterations = 2000;

			/// @brief Learning rate of the smoother.
			float learningRate = 0.2f;

			/// @brief Weight on the error between the requested path and the
			/// optimal path.
			float pathWeight = 0.0f;
			/// @brief Weight on the smoothing term
			float smoothWeight = 4.0f;
			/// @brief Weight on the proximity to the Voronoi edge
			float voronoiWeight = 0.2f;
			/// @brief Weight on the proximity to obstacle
			float collisionWeight = 0.002f;
			/// @brief Weight on the curvature
			float curvatureWeight = 4.0f;

			/// @brief The points that are within a distance of @minCollisionDist *
			/// (1 + @collisionRatio) will not be modified by the smoother where
			/// @minCollisionDist is the distance below which an obstacle is assumed
			/// to be colliding
			float collisionRatio = 0.2f;
			/// @brief Bound on the curvature
			float maxCurvature;

			Parameters(float maxCurvature) :
				maxCurvature(maxCurvature) { }
		};

		struct State {
			Pose2d pose;
			Direction direction;

			Point2d& Position() { return pose.position; }
			const Point2d& Position() const { return pose.position; }
		};

	public:
		Smoother(const Ref<StateValidatorOccupancyMap>& validator, const Ref<GVD>& gvd, float maxCurvature);

		std::vector<State> Smooth(const std::vector<State>& path);

		Parameters GetParameters() { return m_param; }
		const Parameters& GetParameters() const { return m_param; }
		void SetParameters(const Parameters& param) { m_param = param; }

	private:
		std::vector<State> Smooth(const std::vector<State>& path, Scope<std::unordered_set<int>> unsafeIndices);

		Point2d CalculateCurvatureTerm(const Point2d& xim1, const Point2d& xi, const Point2d& xip1) const;
		std::unordered_set<int> CheckPath(const std::vector<State>& path) const;

	private:
		Parameters m_param;
		Ref<StateValidatorOccupancyMap> m_validator;
		Ref<OccupancyMap> m_map;
		Ref<GVD> m_gvd;

		int m_currentCount;
	};
}
