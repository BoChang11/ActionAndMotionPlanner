#include "MP/RegionGroup.hpp"
#include "MP/Planner.hpp"

namespace Abetare
{
    void RegionGroup::SelectTarget(const Action * const a, double target[])
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

/*	
	Planner::GetInstance()->m_decompSearchInfo.m_idFromRoom = a->m_idFromRoom;
	Planner::GetInstance()->m_decompSearchInfo.m_idToRoom   = a->m_idToRoom;
	Planner::GetInstance()->m_decompSearchInfo.m_idToRegion = Planner::GetInstance()->m_decomposition.LocateRegion(target);

	const bool       breakEarly = true;
	int              idGoalRegion;
	std::vector<int> path;

	if(Planner::GetInstance()->m_decompSearch.AStar(m_idRegion, breakEarly, &idGoalRegion))
	{
	    Planner::GetInstance()->m_decompSearch.GetPathFromStart(idGoalRegion, &path);
	    
	    const int n   = path.size();
	    const int use = RandomUniformInteger(n == 1 ? 0 : 1, std::min<int>(10, n - 1));
	    
	    Planner::GetInstance()->m_decomposition.SamplePointInsideRegion(path[use], target);
	}
*/
    }


    int  RegionGroup::SelectVertex(const double target[])
    {
	double   dmin    = HUGE_VAL;
	int      imin    = -1;
	Planner *planner = Planner::GetInstance();
	
	for(int i = m_idsVertices.size() - 1; i >= 0; --i)
	{
	    const double d = Algebra2D::PointDistSquared(target, planner->m_vertices[m_idsVertices[i]]->m_proj);
	    if(d < dmin)
	    {
		dmin = d;
		imin = m_idsVertices[i];
	    }
	}

	return imin;
    }

    double RegionGroup::GetCost(void)
    {
	if(m_curr == m_cache.end())
	    return PARAM_HCOST_MAX;
	return m_curr->second;
    }
    
    void RegionGroup::OnSelection(void)
    {
	m_curr->second *= PARAM_REGION_GROUP_SEL_PENALTY;
    }

    void RegionGroup::OnExpansionFailure(void)
    {
	m_curr->second *= pow(PARAM_REGION_GROUP_SEL_PENALTY, 10);
    }
       
    void RegionGroup::ReadyForNewAction(const Action * const a)
    {
	if(a == NULL)
	{
	    m_curr = m_cache.end();
	    return;
	}

	RegionGroupKey key;
	key.m_idRoom   = a->m_idToRoom;
	key.m_idObject = a->m_idObject;
	
	if((m_curr = m_cache.find(key)) != m_cache.end())
	    return;
	
	
	Planner::GetInstance()->m_decompSearchInfo.m_idFromRoom = a->m_idFromRoom;
	Planner::GetInstance()->m_decompSearchInfo.m_idToRoom   = a->m_idToRoom;
	Planner::GetInstance()->m_decompSearchInfo.m_idToRegion = Planner::GetInstance()->m_decomposition.GetDepotId(a->m_idToRoom);

	const bool       breakEarly = false;
	int              idGoalRegion;
	double           hcost;
	
	if(Planner::GetInstance()->m_decompSearch.AStar(m_idRegion, breakEarly, &idGoalRegion))
	{
	    hcost = Planner::GetInstance()->m_decompSearch.GetPathCostFromStart(idGoalRegion);
	    if(hcost <= PARAM_HCOST_MIN)
		hcost = PARAM_HCOST_MIN;
	}
	else
	    hcost = PARAM_HCOST_MAX;

	m_cache.insert(std::make_pair(key, hcost));
	m_curr = m_cache.find(key);
	
	//printf("................................hcost for %d <%d %d>: %f\n", m_idRegion, key.m_idRoom, key.m_idObject, hcost);
    }

    

}


