#include "MP/Groups.hpp"
#include "MP/Planner.hpp"
#include "MP/Params.hpp"

namespace Abetare
{
    Group* Groups::Select(int * const pos) 
    {
	double    cmin = HUGE_VAL;
	int       imin = 0;
	double    c;
	
	for(int i = m_avail.size() - 1; i >= 0; --i)
	    if((c = m_avail[i]->GetCost()) <  cmin)
	    {
		cmin = c;;
		imin = i;
	    }

	if(pos)
	    *pos = imin;
	

	m_avail[imin]->OnSelection();
	return m_avail[imin];
    }
    
    
    Group* Groups::Find(const int ds[])
    {
	DiscreteSpace *discreteSpace = &(Planner::GetInstance()->m_discreteSpace);
	
	const int n = m_avail.size();
	for(int i = 0; i < n; ++i)
	    if(discreteSpace->Same(ds, m_avail[i]->m_discreteState))
		return m_avail[i];
	return NULL;
    }

    void Groups::AddVertex(Vertex * const v, ActionPlan *ap, const bool adone)
    {
	Group *g;
	
	if(v->m_idGroup < 0)
	{
	    int *ds = Planner::GetInstance()->m_discreteSpace.New();
	    Planner::GetInstance()->m_continuousSpace.MapToDiscreteState(v->m_continuousState, v->m_idObjectAttached, ds);
	    g = Find(ds);
	    if(g == NULL)
	    {
		v->m_idGroup = m_avail.size();

		g = new Group();
		g->m_discreteState = ds;
		m_avail.push_back(g);

		if(Planner::GetInstance()->m_discreteSpace.Solved(ds))
		    Planner::GetInstance()->m_vidAtGoal = v->m_idVertex;

		switch(Planner::GetInstance()->m_plannerType)
		{
//		case Planner::PLANNER_NEW:  g->UsePlanCost(adone); break;
		case Planner::PLANNER_SMAP: g->SMAPNewActionPlan(); break;
		case Planner::PLANNER_ONE:  g->ONEindexInPlan = Planner::GetInstance()->m_indexInPlan + adone;
		}
	    }
	    else 
		Planner::GetInstance()->m_discreteSpace.Delete(ds);
	}
	else
	    g = m_avail[v->m_idGroup];
	g->AddVertex(v, ap && ap->m_plan.size() > 0 ? &(ap->m_plan[0]) : NULL);

	if(ap && g->m_actionPlan.m_plan.size() == 0 && Planner::GetInstance()->m_plannerType == Planner::PLANNER_NEW)
	    g->UsePlanFromParent(ap, adone);
	
	    
    }
}


