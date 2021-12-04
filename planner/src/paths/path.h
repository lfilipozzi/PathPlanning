#pragma once

#include "geometry/2dplane.h"
#include "core/base.h"
#include <set>

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

	/// @brief Interface to define a path
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

		/// @overload
		std::vector<State> Interpolate(const std::vector<double>& ratios) const
		{
			std::vector<State> states;
			states.reserve(ratios.size());
			for (double r : ratios)
				states.push_back(Interpolate(r));
			return states;
		}

		/// @brief Truncate the path
		virtual void Truncate(double ratio) = 0;

		/// @brief Time-flip the path to swap the initial and final position.
		virtual void TimeFlipTransform()
		{
			std::swap(m_init, m_final);
		}

		/// @brief Return the length of the path.
		double GetLength() const { return m_length; }

	protected:
		State m_init, m_final;
		double m_length = 0.0;
	};

	/// @brief Interface to generate path connecting different states.
	template <typename State>
	class PathConnection {
	public:
		PathConnection() = default;
		virtual ~PathConnection() = default;

		/// @brief Connect the two states.
		virtual Ref<Path<State>> Connect(const State& from, const State& to) = 0;
	};

	using PathR2Base = Path<Point2d>;
	using PathSE2Base = Path<Pose2d>;
	using PathConnectionR2Base = PathConnection<Point2d>;
	using PathConnectionSE2Base = PathConnection<Pose2d>;

	template <typename State>
	class PathNonHolonomic : public Path<State> {
	public:
		PathNonHolonomic() = default;
		PathNonHolonomic(Pose2d init, double length = 0.0) :
			Path<State>(init, length) { }

		/// @brief Return the direction of the path
		virtual Direction GetDirection(double ratio) const = 0;

		/// @brief Return the ratio of cusp points.
		virtual std::set<double> GetCuspPointRatios() const
		{
			return std::set<double>();
		}
	};

	using PathNonHolonomicSE2Base = PathNonHolonomic<Pose2d>;

}
