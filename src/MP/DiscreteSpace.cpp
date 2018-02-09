#include "MP/DiscreteSpace.hpp"
#include "MP/Planner.hpp"

namespace Abetare
{
    void DiscreteSpace::CompleteSetup(void) 
    {
	SetDim(OBJS_ROOMS + Planner::GetInstance()->m_problem.m_objects.size());
    }
    
    bool DiscreteSpace::Same(const int ds1[], const int ds2[]) const
    {
	const int n = GetDim();
	for(int i = 0; i < n; ++i)
	    if(ds1[i] != ds2[i])
		return false;
	return true;
    }

    
    bool DiscreteSpace::Solved(const int ds[]) const
    {
	if(GetAttachedObject(ds) >= 0)
	    return false;
	const std::vector<Problem::Object*> *objs = &(Planner::GetInstance()->m_problem.m_objects);

	for(int i = objs->size() - 1; i >= 0; --i)
	    if(objs->operator[](i)->m_idGoalRoom != GetObjectRoom(ds, i))
		return false;
	return true;
    }
    
}


