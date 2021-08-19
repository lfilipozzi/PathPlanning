#pragma once

#include <tuple>

namespace Planner {

	/**
	 * @brief Interface to sample the configuration space as required by the
	 * search algorithm.
	 */
	template <typename Vertex>
	class StateSpace {
	public:
		StateSpace() = default;
		virtual ~StateSpace() = default;

		/**
		* @brief Calculate the distance between two states
		* @param from The start state.
		* @param to The end state.
		* @return The distance between the states
		*/
		virtual double ComputeDistance(const Vertex& from, const Vertex& to) const = 0;

		/**
		 * @brief Generate a random state within the configuration space.
		 * @details The sample must not be inside an obstacle.
		 * @return A random state.
		 */
		virtual Vertex Sample() = 0;

		/**
		 * @brief Construct a new state by moving an incremental distance 
		 * from @source towards @target.
		 * @details The path is not assumed to bring exactly to target but it 
		 * must evolve towards it.
		 * @return The new state.
		 */
		virtual Vertex SteerTowards(const Vertex& source, const Vertex& target) = 0;

		/**
		 * @brief Construct a path that connects the node @source to the node 
		 * @target.
		 * @return A tuple containing the cost to go from the source to the 
		 * target and a boolean indicating if the path is collision-free.
		 */
		virtual std::tuple<double, bool> SteerExactly(const Vertex& source, const Vertex& target) = 0;

		/**
		 * @brief Check if there exist a valid transition between two states.
		 * @param from The initial state.
		 * @param to The destination state.
		 * @return True if the transition is valid, false otherwise.
		 */
		virtual bool IsTransitionCollisionFree(const Vertex& from, const Vertex& to) = 0;
	};
}
