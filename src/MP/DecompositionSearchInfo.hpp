#ifndef ABETARE__DECOMPOSITION_SEARCH_INFO_HPP_
#define ABETARE__DECOMPOSITION_SEARCH_INFO_HPP_

#include "Utils/GraphSearch.hpp"

namespace Abetare
{
    class DecompositionSearchInfo : public GraphSearchInfo<int>
    {
    public:
	DecompositionSearchInfo(void) : GraphSearchInfo<int>(),
					m_idFromRoom(Constants::ID_UNDEFINED),
					m_idToRoom(Constants::ID_UNDEFINED),
					m_idToRegion(Constants::ID_UNDEFINED)
	{
	}
	
	void GetOutEdges(const int u, 
			 std::vector<int> * const edges,
			 std::vector<double> * const costs = NULL) const;
	
	bool IsGoal(const int key) const;
	
	double HeuristicCostToGoal(const int u) const;
	
	int m_idFromRoom;
	int m_idToRoom;
	int m_idToRegion;
    };	
}

#endif


