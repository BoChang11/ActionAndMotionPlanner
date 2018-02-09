#include "MP/Planner.hpp"

namespace Abetare
{

    void ApplyAction(const int dsFrom[], const Action * const a, int dsTo[])
    {
	DiscreteSpace *dspace = &(Planner::GetInstance()->m_discreteSpace);
	
	dspace->Copy(dsTo, dsFrom);
	
	if(a->m_type == Action::TYPE_MOVE)
	    dspace->SetRobotRoom(dsTo, a->m_idToRoom);
	else if(a->m_type == Action::TYPE_MOVE_WITH_OBJECT)
	{
	    dspace->SetRobotRoom(dsTo, a->m_idToRoom);
	    dspace->SetObjectRoom(dsTo, a->m_idObject, a->m_idToRoom);
	}
	else if(a->m_type == Action::TYPE_PICKUP)
	    dspace->SetAttachedObject(dsTo, a->m_idObject);
	else if(a->m_type == Action::TYPE_RELEASE)
	    dspace->SetAttachedObject(dsTo, Constants::ID_UNDEFINED);
    }
}


