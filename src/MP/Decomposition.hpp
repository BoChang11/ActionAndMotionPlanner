#ifndef ABETARE__MP_DECOMPOSITION_HPP_
#define ABETARE__MP_DECOMPOSITION_HPP_

#include "MP/Problem.hpp"
#include "MP/Region.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Grid.hpp"

namespace Abetare
{    
    class Decomposition
    {
    public:
	Decomposition(void)
	{
	}
						
	virtual ~Decomposition(void)
	{
	    DeleteItems<Region*>(&m_regions);
	}

	virtual void CompleteSetup(void);
	
	virtual int LocateRegion(const double p[]);

	virtual bool IsInsideRegion(const int rid, const double p[])
	{
	    return m_regions[rid]->m_shape.IsPointInside(p);
	}
	
	virtual void SamplePointInsideRegion(const int rid, double p[]) const
	{
	    m_regions[rid]->m_shape.SampleRandomPointInside(p);
	}

	virtual int GetDepotId(const int i) const;
	virtual int GetDoorId(const int i) const;
	

	virtual void Draw(void);
	virtual void DrawRegions(void);
	virtual void DrawEdges(void);
	
	std::vector<Region*>  m_regions;
	Grid                  m_covGrid;

    protected:
	virtual void Triangulate(void);
	virtual void LocatorConstruct(void);
	virtual void LocatorUpdateCells(const int rid);
	
	struct InsideIntersectRegions
	{
	    int              m_insideRegion;
	    std::vector<int> m_intersectRegions;
	};
	
	std::vector< InsideIntersectRegions* > m_cellsToRegions;
    };    
}

#endif


