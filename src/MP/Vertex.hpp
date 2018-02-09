#ifndef ABETARE__VERTEX_HPP_
#define ABETARE__VERTEX_HPP_

#include "Utils/Constants.hpp"
#include <cstdlib>

namespace Abetare
{
    struct Vertex
    {
	Vertex(void) : m_continuousState(NULL),
		       m_parent(Constants::ID_UNDEFINED),
		       m_idVertex(Constants::ID_UNDEFINED),
		       m_idRoom(Constants::ID_UNDEFINED),
		       m_idRegion(Constants::ID_UNDEFINED),
		       m_idRegionGroup(Constants::ID_UNDEFINED),
		       m_idGroup(Constants::ID_UNDEFINED),
		       m_idObjectAttached(Constants::ID_UNDEFINED)
	{
	}

	virtual ~Vertex(void)
	{
	    if(m_continuousState)
		delete[] m_continuousState;
	}
	
	
	double *m_continuousState;
	int     m_parent;
	int     m_idVertex;
	int     m_idRoom;
	int     m_idRegion;
    	int     m_idRegionGroup;
	int     m_idGroup;
	int     m_idObjectAttached;
	double  m_proj[2];
    };
    
	
}

#endif


