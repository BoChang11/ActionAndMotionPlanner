#ifndef ABETARE__GROUP_HPP_
#define ABETARE__GROUP_HPP_

#include "MP/RegionGroup.hpp"
#include "MP/Action.hpp"
#include "MP/ActionPlan.hpp"
#include "MP/Params.hpp"
#include "MP/Vertex.hpp"
#include "Utils/Misc.hpp"
#include <algorithm>
#include <vector>

namespace Abetare
{
    class Group
    {
    public:
	Group(void) : m_discreteState(NULL),
		      m_nsel(0)
	{
	    SMAPutil = -HUGE_VAL;
	    ONEindexInPlan = -1;
	    
	}
	
	virtual ~Group(void)
	{
	    DeleteItems<RegionGroup*>(&m_avail);
	    if(m_discreteState)
		delete[] m_discreteState;
	}
	
	void         ReadyForNewAction(const Action * const a);
	RegionGroup* SelectRegionGroup(void);
	void         SelectTarget(const Action * const a, double target[]);
	int          SelectTargetAndVertex(const Action * const a, double target[]);
 	void         AddVertex(Vertex * const v, const Action * const a);
	

	double GetCost(void) const
	{
	    if(m_actionPlan.m_plan.size() == 0)
		return PARAM_HCOST_MAX;
	    return pow(m_actionPlan.m_cost, 4) * pow(PARAM_GROUP_SEL_PENALTY, m_nsel);
	}

	void ClearPlan(void)
	{
	    m_actionPlan.m_plan.clear();
	    m_actionPlan.m_cost = PARAM_HCOST_MAX;
	}

	void NewPlan(void)
	{
	    m_actionPlan.Compute(m_discreteState);
	}
	
	void UsePlanFromParent(ActionPlan *ap, const bool adone);
	
			
	void OnSelection(void)
	{
	    ++m_nsel;
	}

	void OnExpansionFailure(void);
	
	void Print(FILE * out);
	
	
	
	int                       *m_discreteState;
	std::vector<RegionGroup*>  m_avail;
	std::vector<int>           m_map;
	ActionPlan                 m_actionPlan;
	std::vector<int>           m_vids;


	std::vector<Action> SMAPactions;
	std::vector<int >   SMAPnplan;
	std::vector<int>    SMAPnsel;
	Action              SMAPaction;
	double              SMAPutil;
	int                 SMAPiaction;

	int ONEindexInPlan;
	

	double SMAPActionUtil(const int i)
	{
	    return 1.0 / (SMAPnplan[i] * SMAPnplan[i] * SMAPnsel[i]);
	}
	
	void SMAPActionSelect(void)
	{
	    double tw = 0;
	    for(int i = SMAPactions.size() - 1; i >= 0; --i)
		tw += SMAPActionUtil(i);
	    double r = RandomUniformReal(0, tw);
	    double w = 0.0;
	    int    i = SMAPactions.size() - 1;
	    
	    for(; i >= 0; --i)
	    {
		w += SMAPActionUtil(i);
		if(w >= r)
		    break;
	    }
	    SMAPiaction= i;
	    SMAPaction = SMAPactions[i];
	    SMAPutil   = SMAPActionUtil(i);

	    ++SMAPnsel[i];
	    
	}

	void SMAPNewActionPlan(void)
	{
	    double umax = -HUGE_VAL;
	    double u;
	    
	    for(int i = SMAPactions.size() - 1; i >= 0; --i)
		if((u = SMAPActionUtil(i)) > umax)
		    umax = u;
	    if(umax < 0.001)
	    {
		ActionPlan ap;
		ap.Compute(m_discreteState);
		if(ap.m_plan.size() > 0)
		{
		    auto pos = std::find(SMAPactions.begin(), SMAPactions.end(), ap.m_plan[0]);
		    if(pos == SMAPactions.end())
		    {
			SMAPactions.push_back(ap.m_plan[0]);
			SMAPnplan.push_back(ap.m_plan.size());
			SMAPnsel.push_back(1);
			SMAPActionSelect();
		    }
		}
	    }
	    
	}
	
	int SMAPSelectTargetAndVertex(double target[]);
	

	int          ONESelectTargetAndVertex(const Action * const a, double target[]);

    protected:
	double m_hcost;
	int    m_nsel;
	
	
    };    
}

#endif


