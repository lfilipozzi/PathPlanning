#pragma once

#include "core/base.h"
#include <vector>
#include <unordered_set>

namespace Planner {
	namespace FrontierImpl {
		template <typename T, typename HashSet, typename Container, typename Compare>
		inline std::pair<const T*, bool> Push(const T& elmt, HashSet& set, Container& data, const Compare& compare)
		{
			auto [it, success] = set.insert(elmt);
			if (success) {
				auto it = std::upper_bound(data.begin(), data.end(), elmt, compare);
				auto insertIt = data.insert(it, elmt);
				return { &(*insertIt), true };
			}
			return { &(*it), false };
		}

		template <typename T, typename Container, typename Compare, typename KeyEqual>
		inline bool Remove(const T& elmt, Container& data, const Compare& compare, const KeyEqual& keyEqual)
		{
			auto first = std::lower_bound(data.begin(), data.end(), elmt, compare);
			auto last = std::upper_bound(data.begin(), data.end(), elmt, compare);
			auto vecIt = std::find_if(first, last, [&](const T& elmt) { return keyEqual(elmt, elmt); });
			if (vecIt != last) {
				data.erase(vecIt);
				return true;
			}
			return false;
		}

		template <typename T, typename HashSet, typename Container>
		inline T Pop(HashSet& set, Container& data)
		{
			// Access last element
			T elmt = data.back();
			// Remove it
			data.pop_back();
			set.erase(elmt);
			return elmt;
		}
	}

	/// @brief Implement a priority list which allows unique membership. It
	/// combines the capabilities of a priority queue and a hash table.
	/// @details The list is sorted by increasing order. A user-provided Compare
	/// can be supplied to change the ordering, e.g. using std::greater would
	/// cause the smallest element to appear as the top.
	template <typename T, typename Compare = std::less<T>, typename Hash = std::hash<T>, typename KeyEqual = std::equal_to<T>, typename Container = std::vector<T>>
	class Frontier {
	public:
		Frontier() = default;
		Frontier(const Compare& compare) :
			m_compare(compare) { }
		Frontier(const Compare& compare, size_t bucketCount, const Hash& hash, const KeyEqual& equal) :
			m_compare(compare), m_set(bucketCount, hash, equal) { }

		/// @brief Return the number of elements in the container.
		size_t Size() const { return m_vec.size(); }

		/// @brief Check whether the container is empty.
		bool Empty() const { return m_vec.empty(); }

		/// @brief Empty the container.
		void Clear() noexcept
		{
			m_vec.clear();
			m_set.clear();
		}

		/// @brief Insert an element in the container.
		/// @return Returns a pair consisting of a pointer to the inserted
		/// element (or to the element that prevented the insertion) and a bool
		/// value set to true if the insertion took place.
		std::pair<const T*, bool> Push(const T& elmt) { return FrontierImpl::Push(elmt, m_set, m_vec, m_compare); }

		/// @brief Remove an element equal to @elmt from the container (if it exists).
		/// @return Number of elements removed (0 or 1).
		size_t Remove(const T& elmt)
		{
			// Find the elmt in the set
			auto setIt = m_set.find(elmt);
			if (setIt != m_set.end()) {
				bool erased = false;
				// Erase elmt in vector
				erased = FrontierImpl::Remove(*setIt, m_vec, m_compare, m_set.key_eq());
				PP_ASSERT(erased, "Element has not been erased");
				// Erase elmt in set
				m_set.erase(setIt);

				return 1;
			} else
				return 0;
		}

		/// @brief Access the last element of the container.
		const T& Top() const { return m_vec.back(); }

		/// @brief Removes the last element of the container.
		/// @return The removed element.
		T Pop() { return FrontierImpl::Pop<T>(m_set, m_vec); }

		/// @brief Find the element in the container that is equal to @elmt.
		/// @return A pointer to the element, nullptr if the element does not exist.
		const T* Find(const T& elmt) const
		{
			auto search = m_set.find(elmt);
			if (search != m_set.end())
				return &(*search);
			return nullptr;
		}

		auto begin() const { return m_vec.begin(); }
		auto end() const { return m_vec.end(); }

	private:
		Compare m_compare;
		Container m_vec;
		std::unordered_set<T, Hash, KeyEqual> m_set;
	};
}
