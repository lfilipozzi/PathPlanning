#pragma once

#include "paths/path.h"

namespace Planner {

	/// @brief Composite path
	template <typename State, typename BasePath = Path<State>>
	class PathComposite : public BasePath {
		static_assert(std::is_base_of<Path<State>, BasePath>::value);

	private:
		struct PathInfo {
			double initLength = 0;
			double finalLength = 0;
			
			PathInfo(double init, double final) : initLength(init), finalLength(final) { };
		};

		using Paths = std::vector<std::pair<PathInfo, Ref<BasePath>>>;

	public:
		PathComposite() = default;

		/// @brief Increase the capacity of the vector to a value that's greater
		/// or equal to @cap.
		void Reserve(size_t cap) { m_paths.reserve(cap); }

		/// @brief Returns the number of elements.
		size_t Size() const { return m_paths.size(); }

		/// @brief Append the path to the container
		void PushBack(const Ref<BasePath>& path)
		{
			double pathLength = path->GetLength();
			m_paths.push_back({ PathInfo(this->m_length, this->m_length + pathLength), path });
			this->m_final = path->GetFinalState();
			this->m_length += pathLength;
		}

		/// @copydoc Planner::Path::Interpolate
		virtual State Interpolate(double ratio) const override final
		{
			if (m_paths.empty())
				throw std::out_of_range("Empty path");

			auto [it, pathRatio] = FindSegment(ratio);
			if (it == m_paths.end())
				it--;
			return it->second->Interpolate(pathRatio);
		}
		using Path<State>::Interpolate;

		/// @copydoc Planner::Path::Truncate
		virtual void Truncate(double ratio) override final
		{
			if (m_paths.empty())
				return;

			auto [it, pathRatio] = FindSegment(ratio);
			if (it == m_paths.end())
				return;
			it->second->Truncate(pathRatio);
			this->m_final = it->second->GetFinalState();
			this->m_length = it->first.initLength + it->second->GetLength();
			m_paths.erase(std::next(it), m_paths.end());
		}

	protected:
		std::tuple<typename Paths::const_iterator, double> FindSegment(double ratio) const
		{
			double length = ratio * this->m_length;
			auto it = std::upper_bound(m_paths.begin(), m_paths.end(), length,
				[](double length, const std::pair<PathInfo, Ref<BasePath>>& pair){
					return length < pair.first.finalLength;
				});
			
			if (it == m_paths.end())
				return { it, 1.0 };
			double pathLength = it->second->GetLength();
			if (pathLength == 0.0)
				return { it, 0.0 };
			double pathRatio = (length - it->first.initLength) / pathLength;
			return { it, pathRatio };
		}

	protected:
		Paths m_paths;
	};

	class PlanarNonHolonomicCompositePath : public PathComposite<Pose2d, PlanarNonHolonomicPath> {
	public:
		/// @copydoc Planner::Path::GetCuspPointRatios
		virtual std::set<double> GetCuspPointRatios() const override;

		/// @copydoc Planner::PlanarNonHolonomicPath::GetDirection
		virtual Direction GetDirection(double ratio) const override;
	};
}
