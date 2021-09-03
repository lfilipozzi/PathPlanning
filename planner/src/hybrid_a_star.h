#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include <eigen3/Eigen/Dense>
#include "path_planner.h"
#include "a_star.h"

namespace Planner {
	class HybridAStar : public PathPlanner<Eigen::Vector3d> {
	private:
		using Pose = Eigen::Vector3d;
		
		class HybridAStarStateSpace : public AStarStateSpace<Pose> {
		public:
			HybridAStarStateSpace(double spatialResolution = 1.0, double angularResolution = 0.0872) : 
				spatialResolution(spatialResolution), angularResolution(angularResolution) {}

			virtual Pose DiscretizeState(const Pose& state) override
			{
				return {
					static_cast<int>(state.x() / spatialResolution),
					static_cast<int>(state.y() / spatialResolution),
					static_cast<int>(fmod(state.z(), 2 * M_PI) / angularResolution),
				};
			}

			virtual double ComputeDistance(const Pose& from, const Pose& to) const override
			{
				Pose delta = from - to;
				return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
			}
			
			virtual bool IsTransitionCollisionFree(const Pose& /*from*/, const Pose& /*to*/) override
			{
				return true;
			}
			
			virtual std::tuple<double, bool> SteerExactly(const Pose& source, const Pose& target) override
			{
				Pose delta = source - target;
				double dist = sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
				return { dist, true };
			}

			/**
			 * @brief Update state assuming constant steer angle  and bicycle
			 * kinematic model.
			 */
			Pose ConstantSteer(const Pose& state, double delta, bool reverse = false)
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

			virtual std::vector<Pose> GetNeighborStates(Pose state) override
			{
				std::vector<double> deltas = { -10, -7.5, -5, -2.5, -0, 2.5, 5, 7.5, 10 };

				std::vector<Pose> neighbors;
				neighbors.reserve(deltas.size());
				for (double delta : deltas) {
					neighbors.push_back(ConstantSteer(state, delta));
				}
				return neighbors;
			}
			
			const double spatialResolution;
			const double angularResolution;
		};
	
		/**
		 * @brief Hash to discretize the 3D configuration space.
		 * @details Two vertex are considered equal if they correspond to the 
		 * same"cell", i.e. they have their position difference is less than the
		 * spatial resolution and if their heading difference is less than the 
		 * angular resolution.
		 */
		struct HashPose {
			std::size_t operator()(const Pose& state) const
			{
				std::size_t seed = 0;
				HashCombine(seed, state.x());
				HashCombine(seed, state.y());
				HashCombine(seed, fmod(state.z(), 2 * M_PI));
				return seed;
			}
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
			Heuristic(const Ref<AStarStateSpace<Pose>>& stateSpace) : stateSpace(stateSpace) {}

			double HeuristicConstraintsWithoutObstacles(const Pose& from, const Pose& to)
			{
				// TODO compute arc
				Pose delta = from - to;
				double euclidean = sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
				return euclidean;
			}
			
			double HeuristicObstaclesWithoutConstraints(const Pose& /*from*/, const Pose& /*to*/)
			{
				// TODO use flow-fields algorithm
				return 0.0;
			}
			
			double operator()(const Pose& from, const Pose& to)
			{
				return std::max(
					HeuristicConstraintsWithoutObstacles(from, to), 
					HeuristicObstaclesWithoutConstraints(from, to) 
				);
			}
			
			Ref<AStarStateSpace<Pose>> stateSpace;
		};
		
	public:
		HybridAStar(const Ref<AStarStateSpace<Pose>>& stateSpace) :
			m_AStarPoseSearch(stateSpace, Heuristic(stateSpace)) {};
		virtual ~HybridAStar() = default;
		
		virtual Status SearchPath() override
		{
			// Run A* on 3D Vertex
			m_AStarPoseSearch.SetInitState(this->m_init);
			m_AStarPoseSearch.SetGoalState(this->m_goal);
			auto status = m_AStarPoseSearch.SearchPath();
			m_path = m_AStarPoseSearch.GetPath();

			// Smooth the path
			// TODO 
			
			return status;
		}
		
		virtual std::vector<Pose> GetPath() override
		{
			return m_path;
		}
		
	private:
		AStar<Pose, HashPose> m_AStarPoseSearch;
		std::vector<Pose> m_path;
	};
}
