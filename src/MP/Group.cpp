#include "MP/Group.hpp"
#include "MP/Planner.hpp"
#include "Utils/PseudoRandom.hpp"

namespace Abetare
{
    void Group::Print(FILE *out)
    {
	const int n = Planner::GetInstance()->m_discreteSpace.GetDim();
	fprintf(out, "ds: ");
	for(int i = 0; i < n; ++i)
	    fprintf(out, "%d ", m_discreteState[i]);
	fprintf(out, "\n");
	fprintf(out, "group cost: %f\n", GetCost());
	fprintf(out, "action plan cost: %f\n", m_actionPlan.m_cost);
	fprintf(out, "action plan length: %d\n", m_actionPlan.m_plan.size());
	if(m_actionPlan.m_plan.size() == 0)
	    fprintf(out, "action: -1\n");
	else
	    fprintf(out, "action: t%d fr%d tr%d d%d o%d\n",
		    m_actionPlan.m_plan[0].m_type,
		    m_actionPlan.m_plan[0].m_idFromRoom,
		    m_actionPlan.m_plan[0].m_idToRoom,
		    m_actionPlan.m_plan[0].m_idDoor,
		    m_actionPlan.m_plan[0].m_idObject);
	


    }
    
    int Group::SelectTargetAndVertex(const Action * const a, double target[])
    {
	if(RandomUniformReal() < 0.2)
	{
	    int last = m_vids.size() - 1;
	    int first= std::max(0, last - 50);
	    
	    return m_vids[RandomUniformInteger(first, last)];
	}

	SelectTarget(a, target);
	return SelectRegionGroup()->SelectVertex(target);
    }
    
    
    void Group::SelectTarget(const Action * const a, double target[])
    {
	double *box;

	if(a->m_type == Action::TYPE_PICKUP)
	{
	    if(RandomUniformReal() < 0.1)
		Planner::GetInstance()->m_problem.m_objects[a->m_idObject]->m_poly.GetSomePointInside(target);
	    else
	    {
		 if(RandomUniformReal() < 0.5)
		     box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_depot;
		 else
		     box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_box;
		 target[0] = RandomUniformReal(box[0], box[2]);
		 target[1] = RandomUniformReal(box[1], box[3]);
	    }
	}
	else if(a->m_type == Action::TYPE_RELEASE)
	{
	    if(RandomUniformReal() < 0.5)
		box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_depot;
	    else
		box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_box;
	    target[0] = RandomUniformReal(box[0], box[2]);
	    target[1] = RandomUniformReal(box[1], box[3]);
	}
	else 
	{
	    if(RandomUniformReal() < 0.5)
		box = Planner::GetInstance()->m_problem.m_rooms[a->m_idFromRoom]->m_box;
	    else
		box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_box;
	    target[0] = RandomUniformReal(box[0], box[2]);
	    target[1] = RandomUniformReal(box[1], box[3]);
	}
    }
    
    RegionGroup* Group::SelectRegionGroup(void) 
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

	m_avail[imin]->OnSelection();
	return m_avail[imin];
    }

    void Group::AddVertex(Vertex * const v, const Action * const a)
    {
	RegionGroup *rg;
	
	if(m_map.size() == 0)
	{
	    const int n = Planner::GetInstance()->m_decomposition.m_regions.size();
	    m_map.resize(n);
	    std::fill(m_map.begin(), m_map.end(), -1);
	}

	if(m_map[v->m_idRegion] < 0)
	{
	    rg = new RegionGroup();
	    rg->m_idRegion = v->m_idRegion;
	    v->m_idRegionGroup = m_map[v->m_idRegion] = m_avail.size();
	    rg->m_idsVertices.push_back(v->m_idVertex);
	    m_avail.push_back(rg);
	    rg->ReadyForNewAction(a);
	}
	else
	{
	    v->m_idRegionGroup = m_map[v->m_idRegion];
	    m_avail[v->m_idRegionGroup]->m_idsVertices.push_back(v->m_idVertex);
	}
	
	m_vids.push_back(v->m_idVertex);
	
    }

    void Group::UsePlanFromParent(ActionPlan *ap, const bool adone)
    {
	if(adone)
	{
	    m_actionPlan.m_plan.assign(ap->m_plan.begin() + 1, ap->m_plan.end());
	    m_actionPlan.m_cost = ap->m_cost - Planner::GetInstance()->m_mapActionsToCosts.find(ap->m_plan[0])->second;
	}
	else
	{
	    m_actionPlan.m_plan.assign(ap->m_plan.begin(), ap->m_plan.end());
	    m_actionPlan.m_cost = ap->m_cost;
	}
    }

    
    void  Group::ReadyForNewAction(const Action * const a)
    {
	for(int i = m_avail.size() - 1; i >= 0; --i)
	    m_avail[i]->ReadyForNewAction(a);
    }
    
    void Group::OnExpansionFailure(void)
    {
  	for(int i = m_avail.size() - 1; i >= 0; --i)
	    m_avail[i]->OnExpansionFailure();
    }
    

    
    int Group::SMAPSelectTargetAndVertex(double target[])
    {
	double *box;
	Action *a = &SMAPaction;
	
	if(a->m_type == Action::TYPE_PICKUP)
	{
	    if(RandomUniformReal() < 0.1)
		Planner::GetInstance()->m_problem.m_objects[a->m_idObject]->m_poly.GetSomePointInside(target);
	    else
	    {
		if(RandomUniformReal() < 0.5)
		    box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_depot;
		else
		    box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_box;
		target[0] = RandomUniformReal(box[0], box[2]);
		target[1] = RandomUniformReal(box[1], box[3]);
	    }
	}
	else if(a->m_type == Action::TYPE_RELEASE)
	{
	    if(RandomUniformReal() < 0.5)
		box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_depot;
	    else
		box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_box;
	    target[0] = RandomUniformReal(box[0], box[2]);
	    target[1] = RandomUniformReal(box[1], box[3]);
	}
	else 
	{
	    if(RandomUniformReal() < 0.5)
		box = Planner::GetInstance()->m_problem.m_rooms[a->m_idFromRoom]->m_box;
	    else
		box = Planner::GetInstance()->m_problem.m_rooms[a->m_idToRoom]->m_box;
	    target[0] = RandomUniformReal(box[0], box[2]);
	    target[1] = RandomUniformReal(box[1], box[3]);
	}

	/*if(RandomUniformReal() < 0.2)
	{
	    int last = m_vids.size() - 1;
	    int first= 0;//std::max(0, last - 1000);
	    
	    return m_vids[RandomUniformInteger(first, last)];
	    }*/
	
	
	double   dmin    = HUGE_VAL;
	int      imin    = -1;
	Planner *planner = Planner::GetInstance();
	
	for(int i = m_vids.size() - 1; i >= 0; --i)
	{
	    const double d = Algebra2D::PointDistSquared(target, planner->m_vertices[m_vids[i]]->m_proj);
	    if(d < dmin)
	    {
		dmin = d;
		imin = m_vids[i];
	    }
	}
	
	return imin;
    }
    


    int Group::ONESelectTargetAndVertex(const Action * const a, double target[])
    {
	SelectTarget(a, target);
	
	double   dmin    = HUGE_VAL;
	int      imin    = -1;
	Planner *planner = Planner::GetInstance();
	
	for(int i = m_vids.size() - 1; i >= 0; --i)
	{
	    const double d = Algebra2D::PointDistSquared(target, planner->m_vertices[m_vids[i]]->m_proj);
	    if(d < dmin)
	    {
		dmin = d;
		imin = m_vids[i];
	    }
	}
	
	return imin;
    }
    
    
}


