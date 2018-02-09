#ifndef ABETARE__DISCRETE_SPACE_HPP_
#define ABETARE__DISCRETE_SPACE_HPP_

#include "Utils/Allocator.hpp"

namespace Abetare
{    
    class DiscreteSpace : public Allocator<int>
    {
    public:
	enum
	    {
		ROBOT_ROOM   = 0,
		OBJ_ATTACHED = 1,
		OBJS_ROOMS   = 2
	    };
	
	DiscreteSpace(void) : Allocator<int>()
	{
	}
	
	virtual ~DiscreteSpace(void)
	{
	}

	virtual void CompleteSetup(void);
	
	
	int GetRobotRoom(const int s[]) const
	{
	    return s[ROBOT_ROOM];
	}
	
	int GetAttachedObject(const int s[]) const
	{
	    return s[OBJ_ATTACHED];
	}
	
	int GetObjectRoom(const int s[], const int i) const
	{
	    return s[OBJS_ROOMS + i];
	}

	void SetRobotRoom(int s[], const int idRoom) const
	{
	    s[ROBOT_ROOM] = idRoom;
	}

	void SetAttachedObject(int s[], const int idObject) const
	{
	    s[OBJ_ATTACHED] = idObject;
	}
	
	void SetObjectRoom(int s[], const int i, const int idRoom)
	{
	    s[OBJS_ROOMS + i] = idRoom;
	}
	
	bool Same(const int ds1[], const int ds2[]) const;

	bool Solved(const int ds[]) const;
	
    };    
}

#endif


