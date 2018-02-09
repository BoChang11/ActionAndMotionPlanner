#ifndef ABETARE__GROUPS_HPP_
#define ABETARE__GROUPS_HPP_

#include "MP/Group.hpp"
#include "Utils/Misc.hpp"
#include <vector>

namespace Abetare
{
    class Groups
    {
    public:
	Groups(void) 
	{
	}
		
	virtual ~Groups(void)
	{
	    DeleteItems<Group*>(&m_avail);
	}

	Group* Select(int * const pos);
	Group* Find(const int ds[]);
	void AddVertex(Vertex * const v, ActionPlan *ap, const bool adone);
	
	
	std::vector<Group*> m_avail;

	Group* SMAPSelect(void)
	{
	    double tw = 0;
	    for(int i = m_avail.size() - 1; i >= 0; --i)
		if(m_avail[i]->SMAPactions.size() > 0)
		    tw += m_avail[i]->SMAPutil;
	    double r = RandomUniformReal(0, tw);
	    double w = 0.0;
	    int    i = m_avail.size() - 1;
	    	    
	    for(; i >= 0; --i)
	    {
		if(m_avail[i]->SMAPactions.size() > 0)
		    w += m_avail[i]->SMAPutil;
		if(w >= r)
		    break;
	    };
	    return m_avail[i];
	    
	}

	void Print(FILE * out)
	{
	    const int n = m_avail.size();
	    for(int i = 0; i < n; ++i)
	    {
		fprintf(out, "group %d/%d\n", i, n);
		m_avail[i]->Print(out);
	    }
	}
	
	
    };    
}

#endif


