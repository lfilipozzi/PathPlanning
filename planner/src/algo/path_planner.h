#pragma once

#include "geometry/2dplane.h"
#include "utils/grid.h"
#include <vector>

namespace Planner {

	enum Status {
		Success = 0,
		Failure = -1,
	};

	/// @brief Solve path planning problems.
	/// @details Templated class where Vertex is the type of the vertices. Its
	/// data must be arranged contiguously in memory. Vertex must be copy
	/// assignable and copy constructible. A specialized template of std::hash
	/// for Vertex must be defined.
	template <typename Vertex>
	class PathPlanner {
	public:
		PathPlanner() { }
		virtual ~PathPlanner() = default;

		/// @brief Search for the optimal collision-free path.
		virtual Status SearchPath() = 0;

		/// @brief Return the optimal collision-free path.
		virtual std::vector<Vertex> GetPath() const = 0;

		void SetInitState(const Vertex& init) { m_init = init; }
		void SetGoalState(const Vertex& goal) { m_goal = goal; }
		const Vertex& GetInitState() const { return m_init; }
		const Vertex& GetGoalState() const { return m_goal; }
		Vertex& GetInitState() { return m_init; }
		Vertex& GetGoalState() { return m_goal; }

	protected:
		Vertex m_init;
		Vertex m_goal;
	};

	using PlanarPathPlanner = PathPlanner<Pose2d>;
	using GridPathPlanner = PathPlanner<GridCellPosition>;
}
