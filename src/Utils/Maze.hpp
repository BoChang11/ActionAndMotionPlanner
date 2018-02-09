#ifndef ABETARE__MAZE_HPP_
#define ABETARE__MAZE_HPP_

#include "Utils/Grid.hpp"
#include "Utils/Polygon2D.hpp"
#include <vector>

namespace Abetare
{
    class Maze
    {
    public:
	struct Border
	{
	    int m_cids[2];
	};
	
	std::vector<Border> m_blocked;
	std::vector<Border> m_empty;

	Maze(void)
	{
	}
	
	virtual ~Maze(void)
	{
	}

	virtual void GenerateKruskal(const int dimsx, const int dimsy);

	virtual void KeepBlocked(const double perc);

	virtual void GetWall(const Grid * const grid,
			     const Border       wall,
			     const double       width,
			     double             min[2],
			     double             max[2]) const;
	
	virtual void AddBlockedWalls(const Grid * const grid,
				     const double       width,
				     std::vector<Polygon2D*> * const polys) const;
    };
}

#endif



