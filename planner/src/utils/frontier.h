#pragma once

#include "core/base.h"
#include <vector>
#include <unordered_set>

namespace Planner {
	/// @brief Implement a priority list which allows unique membership and 
	/// random access removal.
	/// @details The list is sorted by increasing order. A user-provided Compare
	/// can be supplied to change the ordering, e.g. using std::greater would
	/// cause the smallest element to appear as the top. 
	template <typename T, typename Compare = std::less<T>, typename Hash = std::hash<T>, typename KeyEqual = std::equal_to<T>>
	class Frontier {
	public:
		Frontier() = default;
		Frontier(const Compare& compare) : m_compare(compare) { }
		Frontier(const Compare& compare, size_t bucketCount, const Hash& hash, const KeyEqual& equal) : 
			m_compare(compare), m_keyEqual(equal), m_set(bucketCount, hash, equal) { }

		/// @brief Return the number of elements in the container.
		size_t Size() const { return m_vec.size(); }

		/// @brief Check whether the container is empty.
		bool Empty() const { return m_vec.empty(); }

		/// @brief Insert an element in the container.
		/// @return Returns a pair consisting of a pointer to the inserted 
		/// element (or to the element that prevented the insertion) and a bool 
		/// value set to true if the insertion took place.
		std::pair<const T*, bool> Push(const T& elmt)
		{
			auto [it, success] = m_set.insert(elmt);
			if (success) {
				auto it = FindFirstCompareIsFalse(elmt);
				auto insertIt = m_vec.insert(it, elmt);
				return { &(*insertIt), true };
			}
			return { &(*it), false };
		}

		/// @brief Remove an element equal to @elmt from the container (if it exists).
		/// @return Number of elements removed (0 or 1).
		size_t Remove(const T& elmt)
		{
			// Find the elmt in the set
			auto setIt = m_set.find(elmt);
			if (setIt != m_set.end()) {
				// Erase elmt in vector
				bool erased = false;
				for (auto vecIt = FindLastCompareIsTrue(*setIt); vecIt != m_vec.end(); vecIt++) {
					if (m_keyEqual(*vecIt, *setIt)) {
						m_vec.erase(vecIt);
						erased = true;
						break;
					}
				}
				PP_ASSERT(erased, "Element has not been erased");
				// Erase elmt in set
				m_set.erase(setIt);
				
				return 1;
			} else
				return 0;
		}

		/// @brief Access the last element of the container.
		T Top() const
		{
			return m_vec.back();
		}

		/// @brief Removes the last element of the container.
		/// @return The removed element.
		T Pop()
		{
			// Access last element
			T elmt = m_vec.back();
			// Remove it
			m_vec.pop_back();
			m_set.erase(elmt);
			return elmt;
		}

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
		/// @brief Return an iterator to the first element @elmt in the 
		/// container for which Compare(@value, @elmt) is false.
		/// @details Implemented using a binary search.
		/// @return An iterator to the element.
		typename std::vector<T>::iterator FindFirstCompareIsFalse(const T& value)
		{
			unsigned int index = 0;
			{
				int begin = 0;
				int end = m_vec.size() - 1;
				while (begin <= end) {
					int mid = begin + (end - begin) / 2;

					bool left = !m_compare(value, m_vec[mid]);
					bool right = mid + 1 < m_vec.size() ? m_compare(value, m_vec[mid + 1]) : true;

					if (left && right) {
						index = mid + 1;
						break;
					} else if (!left)
						end = mid - 1;
					else if (!right)
						begin = mid + 1;
				}
			}

			auto it = m_vec.begin();
			std::advance(it, index);
			return it;
		}

		/// @brief Return an iterator to the last element @elmt in the 
		/// container for which Compare(@elmt, @value) is true.
		/// @details Implemented using a binary search.
		/// @return An iterator to the element.
		typename std::vector<T>::iterator FindLastCompareIsTrue(const T& value)
		{
			unsigned int index = 0;
			{
				int begin = 0;
				int end = m_vec.size() - 1;
				while (begin <= end) {
					int mid = begin + (end - begin) / 2;

					bool left = m_compare(m_vec[mid], value);
					bool right = mid + 1 < m_vec.size() ? !m_compare(m_vec[mid + 1], value) : true;

					if (left && right) {
						index = mid;
						break;
					} else if (!left)
						end = mid - 1;
					else if (!right)
						begin = mid + 1;
				}
			}

			auto it = m_vec.begin();
			std::advance(it, index);
			return it;
		}

	private:
		Compare m_compare;
		KeyEqual m_keyEqual;
		std::vector<T> m_vec;
		std::unordered_set<T, Hash, KeyEqual> m_set;
	};
}
