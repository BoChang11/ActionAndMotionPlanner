#include "MP/DecompositionSearchInfo.hpp"
#include "MP/Planner.hpp"

namespace Abetare
{
    void DecompositionSearchInfo::GetOutEdges(const int u, 
					  std::vector<int> * const edges,
					  std::vector<double> * const costs) const
    {
	Decomposition *decomp = &(Planner::GetInstance()->m_decomposition);
	Problem       *prob   = &(Planner::GetInstance()->m_problem);
	
	if(edges)
	    edges->clear();
	if(costs)
	    costs->clear();
	
	const int n = decomp->m_regions[u]->m_neighs.size();
	for(int i = 0; i < n; ++i)
	{
	    const int     neigh = decomp->m_regions[u]->m_neighs[i];		    
	    const Region *r     = decomp->m_regions[neigh];
	    bool          accept;
	    
	    if(r->m_type == Region::TYPE_DOOR)
	    {
		Problem::Door *door = prob->m_doors[r->m_label];
		accept = 
		    ((door->m_idRoom1 == m_idFromRoom && door->m_idRoom2 == m_idToRoom) ||
		     (door->m_idRoom2 == m_idFromRoom && door->m_idRoom1 == m_idToRoom));
	    }
	    else 
		accept = r->m_label == m_idFromRoom || r->m_label == m_idToRoom;
	    
	    
	    if(accept)
	    {
		if(edges)
		    edges->push_back(neigh);
		if(costs)
		    costs->push_back(decomp->m_regions[u]->m_weights[i]);
	    }
	    
	}
    }
    
    bool DecompositionSearchInfo::IsGoal(const int key) const
    {
	if(m_idToRegion >= 0)
	    return key == m_idToRegion;
	
	const Region *r = Planner::GetInstance()->m_decomposition.m_regions[key];
	
	if(r->m_type == Region::TYPE_DEPOT || r->m_type == Region::TYPE_NONE)
	    return r->m_label == m_idToRoom;
	return false;
    }
    
    
    double DecompositionSearchInfo::HeuristicCostToGoal(const int u) const
    {
	Decomposition *decomp = &(Planner::GetInstance()->m_decomposition);

	if(m_idToRegion >= 0)
	    return Algebra2D::PointDist(decomp->m_regions[u]->m_center,
					decomp->m_regions[m_idToRegion]->m_center);
	
	const double *box = Planner::GetInstance()->m_problem.m_rooms[m_idToRoom]->m_box;
	const double  c[] = {0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3])};
		
	return Algebra2D::PointDist(c, decomp->m_regions[u]->m_center);
    }
}


