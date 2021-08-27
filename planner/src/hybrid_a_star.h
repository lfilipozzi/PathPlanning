#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include <eigen3/Eigen/Dense>
#include "core/hash.h"
#include "path_planner.h"
#include "a_star.h"

namespace Planner {
	class HybridAStar : public PathPlanner<Eigen::Vector3d> {
	private:
		using Vertex3D = Eigen::Vector3d;
		
		class HybridAStarStateSpace : public AStarStateSpace<Vertex3D> {
		public:
			virtual double ComputeDistance(const Vertex3D& from, const Vertex3D& to) const override
			{
				Vertex3D delta = from - to;
				return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
			}
			
			virtual bool IsTransitionCollisionFree(const Vertex3D& /*from*/, const Vertex3D& /*to*/) override
			{
				return true;
			}
			
			virtual std::tuple<double, bool> SteerExactly(const Vertex3D& source, const Vertex3D& target) override
			{
				Vertex3D delta = source - target;
				double dist = sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
				return { dist, true };
			}

			/**
			 * @brief Update state assuming constant steer angle  and bicycle
			 * kinematic model.
			 */
			Vertex3D ConstantSteer(const Vertex3D& state, double delta, bool reverse = false)
			{
				double L = 2.6; // Wheelbase
				double b = L / 2; // Distance to rear axle
				
				// Compute distance to travel to another cell
				double spatialResolution = 1.0;
				double d = spatialResolution * 1.5;
				if (reverse)
					d = -d;
				
				// States
				double x = state.x();
				double y = state.y();
				double theta = state.z();
				
				// Update states
				double beta = atan(b * tan(delta) / L);
				x = x + d * cos(theta + beta);
				y = y + d * sin(theta + beta);
				theta = theta + d * cos(beta) * tan(delta) / L;
				
				return {x, y, theta};
			}

			virtual std::vector<Vertex3D> GetNeighborStates(Vertex3D state) override
			{
				std::vector<double> deltas = { -10, -7.5, -5, -2.5, -0, 2.5, 5, 7.5, 10 };

				std::vector<Vertex3D> neighbors;
				neighbors.reserve(deltas.size());
				for (double delta : deltas) {
					neighbors.push_back(ConstantSteer(state, delta));
				}
				return neighbors;
			}
		};
	
		/**
		 * @brief Hash to discretize the 3D configuration space.
		 * @details Two vertex are considered equal if they correspond to the 
		 * same"cell", i.e. they have their position difference is less than the
		 * spatial resolution and if their heading difference is less than the 
		 * angular resolution.
		 */
		struct HashVertex3D {
			HashVertex3D(double spatialResolution = 1.0, double angularResolution = 0.0872) : 
				spatialResolution(spatialResolution), angularResolution(angularResolution) {};
			std::size_t operator()(const Vertex3D& state) const
			{
				std::size_t seed = 0;
				HashCombine(seed, static_cast<int>(state.x() / spatialResolution));
				HashCombine(seed, static_cast<int>(state.y() / spatialResolution));
				HashCombine(seed, static_cast<int>(fmod(state.z(), 2 * M_PI) / angularResolution));
				return seed;
			}
			
			const double spatialResolution;
			const double angularResolution;
		};
		
		/**
		 * @brief Heuristic used by the A* algorithm.
		 * @details Use the maximum of two heuristic:
		 *   - Heuristic with constraints without obstacles
		 *   - Heuristic with obstacles without constraints
		 * The heuristic with obstacles but without constraints is implemented 
		 * by running a 2D flow fields algorithm.
		 */
		struct Heuristic {
			double HeuristicConstraintsWithoutObstacles(const Vertex3D& from, const Vertex3D& to)
			{
				// TODO compute arc
				Vertex3D delta = from - to;
				double euclidean = sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
				return euclidean;
			}
			
			double HeuristicObstaclesWithoutConstraints(const Vertex3D& /*from*/, const Vertex3D& /*to*/)
			{
				// TODO use flow-fields algorithm
				return 0.0;
			}
			
			double operator()(const Vertex3D& from, const Vertex3D& to)
			{
				return std::max(
					HeuristicConstraintsWithoutObstacles(from, to), 
					HeuristicObstaclesWithoutConstraints(from, to) 
				);
			}
		};
		
	public:
		HybridAStar(const Ref<AStarStateSpace<Vertex3D>>& stateSpace) :
			m_3DAStarSearch(stateSpace, Heuristic()) {};
		virtual ~HybridAStar() = default;
		
		virtual Status SearchPath() override
		{
			// Run A* on 3D Vertex
			m_3DAStarSearch.SetInitState(this->m_init);
			m_3DAStarSearch.SetGoalState(this->m_goal);
			auto status = m_3DAStarSearch.SearchPath();
			m_path = m_3DAStarSearch.GetPath();

			// Smooth the path
			// TODO 
			
			return status;
		}
		
		virtual std::vector<Vertex3D> GetPath() override
		{
			return m_path;
		}
		
	private:
		AStar<Vertex3D, HashVertex3D> m_3DAStarSearch;
		std::vector<Vertex3D> m_path;
	};
}
