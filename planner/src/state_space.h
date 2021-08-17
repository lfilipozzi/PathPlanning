#pragma once

namespace Planner {

	/**
	 * @brief Interface to sample the configuration space as required by the
	 * search algorithm.
	 */
	template <typename T>
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
		virtual double ComputeDistance(const T& from, const T& to) const = 0;

		/**
		 * @brief Generate a random state within the configuration space.
		 * @return A random state.
		 */
		virtual T CreateRandomState() = 0;

		/**
		 * @brief Construct a new state by moving an incremental distance 
		 * @stepSize from @source to @target.
		 * @return The new state.
		 */
		virtual T CreateIncrementalState(const T& source, const T& target, double stepSize) = 0;
	};
}
