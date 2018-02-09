#ifndef ABETARE__MP_REGION_HPP_
#define ABETARE__MP_REGION_HPP_

#include "Utils/Polygon2D.hpp"
#include "Utils/Constants.hpp"
#include <vector>

namespace Abetare
{
    
    struct Region
    {
	enum Type
	    {
		TYPE_NONE  = 0,
		TYPE_DEPOT = 1,
		TYPE_DOOR  = 2
	    };
	
	    
	Region(void)
	{
	    m_type   = TYPE_NONE;
	    m_label  = Constants::ID_UNDEFINED;
	    m_valid  = true;
	}
	
	virtual ~Region(void)
	{
	}

	double               m_center[2];
	Type                 m_type;
	std::vector<int>     m_neighs;
	std::vector<double>  m_weights;
	int                  m_label;
	Polygon2D            m_shape;
	std::vector<int>     m_cellsInside;
	std::vector<int>     m_cellsIntersect;
	bool                 m_valid;
    };
}

#endif


