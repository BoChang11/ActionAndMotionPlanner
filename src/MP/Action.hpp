#ifndef ABETARE__ACTION_HPP_
#define ABETARE__ACTION_HPP_

#include "Utils/Constants.hpp"
#include "Utils/HashFn.hpp"
#include <functional>

namespace Abetare
{
    struct Action
    {
	enum Type
	    {
		TYPE_MOVE             = 0,
		TYPE_MOVE_WITH_OBJECT = 1,
		TYPE_PICKUP           = 2,
		TYPE_RELEASE          = 3
	    };
	
	    
	Action(void) : m_type(TYPE_MOVE),
	               m_idFromRoom(Constants::ID_UNDEFINED),
		       m_idToRoom(Constants::ID_UNDEFINED),
		       m_idDoor(Constants::ID_UNDEFINED),
		       m_idObject(Constants::ID_UNDEFINED)
	{
	}
	
	~Action(void)
	{
	}

	Type   m_type;
	int    m_idFromRoom;
	int    m_idToRoom;
	int    m_idDoor;
	int    m_idObject;
    };

    static inline 
    bool operator==(const Action& lhs, const Action& rhs)
    {
	return 
	    lhs.m_type == rhs.m_type &&
	    lhs.m_idFromRoom == rhs.m_idFromRoom &&
	    lhs.m_idToRoom == rhs.m_idToRoom &&
	    lhs.m_idDoor == rhs.m_idDoor &&
	    lhs.m_idObject == rhs.m_idObject;
    }

    void ApplyAction(const int dsFrom[], const Action * const a, int dsTo[]);
    

}

namespace std
{
    template<> struct hash<Abetare::Action> 
    {
	size_t operator()(const Abetare::Action &a) const
	{
	    return Abetare::StringHash((const char*)(&a), sizeof(a));
	}
    };
    
}

#endif


