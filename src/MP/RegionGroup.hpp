#ifndef ABETARE__REGION_GROUP_HPP_
#define ABETARE__REGION_GROUP_HPP_

#include "MP/Action.hpp"
#include "MP/Params.hpp"
#include "Utils/Constants.hpp"
#include "Utils/HashFn.hpp"
#include <functional>
#include <vector>
#include <unordered_map>

namespace Abetare
{
    struct RegionGroupKey
    {
	int m_idRoom;
	int m_idObject;
    };

    static inline 
    bool operator==(const RegionGroupKey& lhs, const RegionGroupKey& rhs)
    {
	return 
	    lhs.m_idRoom == rhs.m_idRoom && 
	    lhs.m_idObject == rhs.m_idObject;
    }
}


namespace std
{
    template<> struct hash<Abetare::RegionGroupKey> 
    {
	size_t operator()(const Abetare::RegionGroupKey &rg) const
	{
	    return Abetare::StringHash((const char*)(&rg), sizeof(rg));
	}
    };
    
}


namespace Abetare
{
    struct RegionGroup
    {
	RegionGroup(void) : m_idRegion(Constants::ID_UNDEFINED)
	{
	    m_curr = m_cache.end();
	}
	
	void   SelectTarget(const Action* const a, double target[]);
	int    SelectVertex(const double target[]);
	void   ReadyForNewAction(const Action* const a);
	double GetCost(void);
	void   OnSelection(void);
	void   OnExpansionFailure(void);
	
	int              m_idRegion;
	std::vector<int> m_idsVertices;
	std::unordered_map<RegionGroupKey, double>           m_cache;
	std::unordered_map<RegionGroupKey, double>::iterator m_curr;
    };
}



#endif


