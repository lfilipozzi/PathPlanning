#include "path_composite.h"

namespace Planner {
	std::set<double> PathSE2CompositeNonHolonomic::GetCuspPointRatios() const
	{
		std::set<double> ratios;

		if (m_paths.empty())
			return ratios;

		double length = 0.0;
		Direction prevDirection = m_paths[0].second->GetDirection(0.0);
		for (auto& path : m_paths) {
			// Check if the path starts with same direction as the previous ends
			if (path.second->GetDirection(0.0) != prevDirection)
				ratios.insert(length / this->m_length);

			// Check for cusp point in the path
			auto cusps = path.second->GetCuspPointRatios();
			ratios.insert(cusps.begin(), cusps.end());

			prevDirection = path.second->GetDirection(1.0);
			length += path.second->GetLength();
		}

		return ratios;
	}

	Direction PathSE2CompositeNonHolonomic::GetDirection(double ratio) const
	{
		if (m_paths.empty())
			return Direction::NoMotion;
		auto [it, pathRatio] = FindSegment(ratio);
		if (it == m_paths.end())
			--it;
		return it->second->GetDirection(pathRatio);
	}
}
