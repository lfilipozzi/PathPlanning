#pragma once

#include <vector>
#include "core/base.h"
#include "state_space.h"

namespace Planner {

	/**
	 * @brief Flow fields algorithm.
	 */
	template <typename Vertex, typename Dimensions>
	class FlowFields {
	public:
		FlowFields() {};

		/**
		 * @brief Reset the algorithm.
		 */
		void Reset()
		{
			ResetField(); //Set total cost in all cells to 65535
			m_frontier.clear();
		}

		/**
		 * @brief Update the field until the cost at position @targetX, and 
		 * @targetY is known.
		 */
		void CalculateField(Vertex goal)
		{
			m_frontier.reserve(std::pow(m_arrayWidth, Dimensions));
			
			// TODO unsigned int goalID = targetY * m_arrayWidth + targetX;
			unsigned int goalID;

			// Set goal node cost to 0 and add it to the open list
			// TODO setValueAt(targetID, 0);
			m_frontier.push_back(goalID);

			while (!m_frontier.empty()) {

				// Get the next node in the open list
				unsigned currentID = m_frontier.Poll();

// 				unsigned short currentX = currentID % m_arrayWidth;
// 				unsigned short currentY = currentID / m_arrayWidth;

				for (auto neighbor : m_stateSpace->GetNeighbors(currentX, currentY)) {
					// Calculate the new cost of the neighbor node
					// based on the cost of the current node and the weight of the next node
					unsigned int endNodeCost = getValueByIndex(currentID) + m_costField->getCostByIndex(neighbor);

					// If a shorter path has been found, add the node into the open list
					if (endNodeCost < getValueByIndex(neighbor)) {
						// Check if the neighbor cell is already in the list.
						// If it is not then add it to the end of the list.
						if (!checkIfContains(neighbor, m_frontier)) {
							m_frontier.push_back(neighbor);
						}

						// Set the new cost of the neighbor node.
						setValueAt(neighbor, endNodeCost);
					}
				}
			}
		}
		
	private:
		void ResetField()
		{
			
		}
		
	private:
		Ref<AStarStateSpace<Vertex>> m_stateSpace;

		const unsigned int m_arrayWidth = 100;
		std::vector<unsigned int> m_frontier;
		CostField m_costField;
	};
}



