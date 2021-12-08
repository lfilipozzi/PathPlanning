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
		enum Status {
			MaxIteration = 0,
			StepTolerance,
			PathSize,
			Failure = -1,
			Collision = -2,
		};

		struct Parameters {
			/// @brief Lower bound on the size of a step. If the solver attempts to
			/// take a step that is smaller than StepTolerance, the iterations end.
			float stepTolerance = 1e-3;
			/// @brief Bound on the number of solver iterations.
			int maxIterations = 2000;

			/// @brief Learning rate of the smoother.
			float learningRate = 0.01f;

			/// @brief Weight on the error between the requested path and the
			/// optimal path.
			float pathWeight = 0.0f;
			/// @brief Weight on the smoothing term
			float smoothWeight = 0.4f;
			/// @brief Weight on the proximity to the Voronoi edge
			float voronoiWeight = 0.02f;
			/// @brief Weight on the proximity to obstacle
			float collisionWeight = 0.2f;
			/// @brief Weight on the curvature
			float curvatureWeight = 0.4f;

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

	public:
		Smoother();

		/// @brief Initialize the algorithm.
		bool Initialize(const Ref<StateValidatorOccupancyMap>& validator, const Ref<GVD>& gvd, float maxCurvature);

		/// @brief Smooth the path and return the status of the optimization.
		Status Smooth(const std::vector<Pose2d>& path, const std::unordered_set<int>& cuspIndices = std::unordered_set<int>());
		/// @brief Return the path
		const std::vector<Pose2d>& GetPath() const { return m_currentPath; }

		const Parameters& GetParameters() const { return m_param; }
		void SetParameters(const Parameters& param) { m_param = param; }

	private:
		void CalculateCurvatureTerm(const Point2d& xim1, const Point2d& xi, const Point2d& xip1, Point2d& gim1, Point2d& gi, Point2d& gip1) const;
		bool IsPathSafe() const;

	private:
		bool isInitialized = false;

		Parameters m_param;
		Ref<StateValidatorOccupancyMap> m_validator;
		Ref<OccupancyMap> m_map;
		Ref<GVD> m_gvd;

		std::vector<Pose2d> m_currentPath;
	};
}
