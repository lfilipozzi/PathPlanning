#pragma once

namespace Planner {

	enum class Steer {
		Left,
		Straight,
		Right
	};

	enum class Direction {
		Forward,
		Backward,
		NoMotion
	};

	/**
	 * @brief Interface to define a path
	 */
	template <typename State>
	class Path {
	public:
		Path() = default;
		Path(State init, double length = 0.0) :
			m_init(init), m_length(length) { }
		virtual ~Path() = default;

		/// @brief Return the initial state
		const State& GetInitialState() const { return m_init; }
		/// @brief Return the final state
		const State& GetFinalState() const { return m_final; }

		/// @brief Interpolate a state along the path.
		virtual State Interpolate(double ratio) const = 0;

		/// @brief Truncate the path
		virtual void Truncate(double ratio) = 0;

		/// @brief Return the length of the path.
		double GetLength() const { return m_length; }

	protected:
		State m_init, m_final;
		double m_length = 0.0;
	};
}