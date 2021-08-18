#pragma once

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
		 * @return A random state.
		 */
		virtual Vertex CreateRandomState() = 0;

		/**
		 * @brief Construct a new state by moving an incremental distance 
		 * @stepSize from @source to @target.
		 * @return The new state.
		 */
		virtual Vertex CreateIncrementalState(const Vertex& source, const Vertex& target, double stepSize) = 0;
	};
}
