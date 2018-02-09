#ifndef ABETARE__MP_PROBLEM_HPP_
#define ABETARE__MP_PROBLEM_HPP_

#include "Utils/Grid.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/Algebra2D.hpp"
#include "External/PQP/PQPTriMesh.hpp"
#include <vector>

namespace Abetare
{
    class Problem
    {
    public:
	struct Door
	{
	    int    m_idRoom1;
	    int    m_idRoom2;
	    double m_box[4];
	};
	
	struct Room
	{
	    std::vector<int> m_idsDoors;
	    double           m_depot[4];
	    double           m_box[4];
	};
	
	struct Object
	{
	    double     m_cfg[3];
	    Polygon2D  m_poly;
	    PQPTriMesh m_tmesh;
	    int        m_idGoalRoom;
	};
	
	struct Obstacles
	{
	    std::vector<Polygon2D*> m_polys;
	    PQPTriMesh              m_tmesh;
	};

	double               m_initRobotTA[3];
	Obstacles            m_obstacles;
	std::vector<Object*> m_objects;
	std::vector<Room*>   m_rooms;
	std::vector<Door*>   m_doors;
	Grid                 m_grid;
	
	double DRAWZ_DOOR_BOX;
	double DRAWZ_DOOR_ID;
	double DRAWZ_ROOM_DEPOT;
	double DRAWZ_ROOM_BOX;
	double DRAWZ_ROOM_ID;
	double DRAWZ_GOAL_ID;
	
	Problem(void);
	
	virtual ~Problem(void);
	
	virtual void Print(FILE * const out);
	virtual void Read(FILE * const in);

	virtual void CompleteSetup(void)
	{
	    ExtrudeObjects();
	    ExtrudeObstacles();
	}
	
	virtual void DrawDoors(void);
	virtual void DrawRooms(void);
	virtual void DrawObjects(void);
	virtual void DrawObstacles(void);
	virtual void DrawGoals(void);
	virtual void Draw(void);

    protected:
	virtual void ExtrudeObjects(void);
	virtual void ExtrudeObstacles(void);


    };	
}

#endif
