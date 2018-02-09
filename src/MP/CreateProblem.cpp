#include "MP/Problem.hpp"
#include "MP/RobotSCar.hpp"
#include "Utils/Maze.hpp"
#include "Utils/Misc.hpp"

namespace Abetare
{
    class CreateProblemParams
    {
    public:
	CreateProblemParams(void)
	{
//grid params		
	    m_nrCellsX = 3;
	    m_nrCellsY = 3;
	    m_dim      = 120.0;
	    
//depot params
	    m_depotPercBorderClearance = 0.05;
	    m_depotPercMinDim          = 0.3;
	    m_depotPercMaxDim          = 0.4;
	    
//object params
	    m_nrObjects          = 6;
	    m_objPercMinRadius   = 0.3; //0.3
	    m_objPercMaxRadius   = 0.4; //0.4
	    m_objMinNrVertices   = 4;
	    m_objMaxNrVertices   = 8;
	    
//maze params
	    m_mazePercKeepBlocked = 0.9;
	    m_mazeWallThickness   = 1.0;
	    
//door params
	    m_doorPercMinDim  = 0.3; //0.3
	    m_doorPercMaxDim  = 0.4; //0.4

//obs params
	    m_nrRandObs          = 45;
	    m_obsPercMinRadius   = 0.07;
	    m_obsPercMaxRadius   = 0.15;
	    m_obsMinNrVertices   = 4;
	    m_obsMaxNrVertices   = 8;
	    m_obsSep             = 1.0;
	}
	
	void Print(void)
	{
	    printf("CreateProblemParams\n");

	    printf("..nrCellsX = <%d>\n", m_nrCellsX);
	    printf("..nrCellsY = <%d>\n", m_nrCellsY);
	    printf("..dim      = <%f>\n\n", m_dim);
	    
	    printf("..depotPercBorderClearance = <%f>\n", m_depotPercBorderClearance);
	    printf("..depotPercMinDim          = <%f>\n", m_depotPercMinDim);
	    printf("..depotPercMaxDim          = <%f>\n\n", m_depotPercMaxDim);
	    
	    printf("..nrObjects        = <%d>\n", m_nrObjects);
	    printf("..objPercMinRadius = <%f>\n", m_objPercMinRadius);
	    printf("..objPercMaxRadius = <%f>\n", m_objPercMaxRadius);
	    printf("..objMinNrVertices = <%d>\n", m_objMinNrVertices);
	    printf("..objMaxNrVertices = <%d>\n\n", m_objMaxNrVertices);
	    
	    printf("..mazePercKeepBlocked = <%f>\n", m_mazePercKeepBlocked);
	    printf("..mazeWallThickness   = <%f>\n\n", m_mazeWallThickness);
	 
	    printf("..doorPercMinDim = <%f>\n", m_doorPercMinDim);
	    printf("..doorPercMaxDim = <%f>\n\n", m_doorPercMaxDim);
	}

	void Read(FILE * const in)
	{
	    char keyword[100];
	    
	    while(fscanf(in, "%s", keyword) == 1)
	    {
		if(strcmp(keyword, "end") == 0)
		    return;
		else if(strcmp(keyword, "nrCellsX") == 0)
		{
		    if(fscanf(in, "%d", &m_nrCellsX) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "nrCellsY") == 0)
		{
		    if(fscanf(in, "%d", &m_nrCellsY) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "dim") == 0)
		{
		    if(fscanf(in, "%lf", &m_dim) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "depotPercBorderClearance") == 0)
		{
		    if(fscanf(in, "%lf", &m_depotPercBorderClearance) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "depotPercMinDim") == 0)
		{
		    if(fscanf(in, "%lf", &m_depotPercMinDim) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "depotPercMaxDim") == 0)
		{
		    if(fscanf(in, "%lf", &m_depotPercMaxDim) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "nrObjects") == 0)
		{
		    if(fscanf(in, "%d", &m_nrObjects) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "objPercMinRadius") == 0)
		{
		    if(fscanf(in, "%lf", &m_objPercMinRadius) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "objPercMaxRadius") == 0)
		{
		    if(fscanf(in, "%lf", &m_objPercMaxRadius) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "objMinNrVertices") == 0)
		{
		    if(fscanf(in, "%d", &m_objMinNrVertices) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "objMaxNrVertices") == 0)
		{
		    if(fscanf(in, "%d", &m_objMaxNrVertices) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "mazePercKeepBlocked") == 0)
		{
		    if(fscanf(in, "%lf", &m_mazePercKeepBlocked) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "mazeWallThickness") == 0)
		{
		    if(fscanf(in, "%lf", &m_mazeWallThickness) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "doorPercMinDim") == 0)
		{
		    if(fscanf(in, "%lf", &m_doorPercMinDim) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "doorPercMaxDim") == 0)
		{
		    if(fscanf(in, "%lf", &m_doorPercMaxDim) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "nrRandObs") == 0)
		{
		    if(fscanf(in, "%d", &m_nrRandObs) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "obsPercMinRadius") == 0)
		{
		    if(fscanf(in, "%lf", &m_obsPercMinRadius) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "obsPercMaxRadius") == 0)
		{
		    if(fscanf(in, "%lf", &m_obsPercMaxRadius) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "obsMinNrVertices") == 0)
		{
		    if(fscanf(in, "%d", &m_obsMinNrVertices) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "obsMaxNrVertices") == 0)
		{
		    if(fscanf(in, "%d", &m_obsMaxNrVertices) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else if(strcmp(keyword, "obsSep") == 0)
		{
		    if(fscanf(in, "%lf", &m_obsSep) != 1)
			OnInputError(printf("expecting value after %s\n", keyword));
		}
		else
		    printf("..unknown param name <%s>\n", keyword);
	    }
	    
	}
	

//grid params		
	int    m_nrCellsX;
	int    m_nrCellsY;
	double m_dim;

//depot params
	double m_depotPercBorderClearance;
	double m_depotPercMinDim;
	double m_depotPercMaxDim;

//object params
	int    m_nrObjects;
	double m_objPercMinRadius;
	double m_objPercMaxRadius;
	int    m_objMinNrVertices;
	int    m_objMaxNrVertices;


//maze params
	double m_mazePercKeepBlocked;
	double m_mazeWallThickness;

//door params
	double m_doorPercMinDim;
	double m_doorPercMaxDim;

//obstacle params
	int    m_nrRandObs;
	double m_obsPercMinRadius;
	double m_obsPercMaxRadius;
	int    m_obsMinNrVertices;
	int    m_obsMaxNrVertices;
	double m_obsSep;
    };

    class CreateProblem
    {
    public:

	void CreateRooms(void)
	{
	    const int nrCells = m_problem.m_grid.GetNrCells();
	    for(int i = 0; i < nrCells; ++i)
	    {
		Problem::Room *room = new Problem::Room();
		m_problem.m_grid.GetCellFromId(i, room->m_box, &(room->m_box[2]));

		const double clearx = m_params.m_depotPercBorderClearance * (room->m_box[2] - room->m_box[0]);
		const double cleary = m_params.m_depotPercBorderClearance * (room->m_box[3] - room->m_box[1]);
		const double xdim = 
		    (1 - 2 * m_params.m_depotPercBorderClearance) * (room->m_box[2] - room->m_box[0]) *
		    RandomUniformReal(m_params.m_depotPercMinDim, m_params.m_depotPercMaxDim);
		const double ydim = 
		    (1 - 2 * m_params.m_depotPercBorderClearance) * (room->m_box[3] - room->m_box[1]) *
		    RandomUniformReal(m_params.m_depotPercMinDim, m_params.m_depotPercMaxDim);
		const double x = 
		    RandomUniformReal(room->m_box[0] + clearx + 0.5 * xdim,
				      room->m_box[2] - clearx - 0.5 * xdim);
		const double y = 
		    RandomUniformReal(room->m_box[1] + cleary + 0.5 * ydim,
				      room->m_box[3] - cleary - 0.5 * ydim);
		
		room->m_depot[0] = x - 0.5 * xdim; room->m_depot[1] = y - 0.5 * ydim;
		room->m_depot[2] = x + 0.5 * xdim; room->m_depot[3] = y + 0.5 * ydim;

		m_problem.m_rooms.push_back(room);
	    }
	}
	
	void CreateObjects(void)
	{
	    std::vector<int>  perm;
	    Problem::Object  *obj;

	    perm.resize(m_problem.m_grid.GetNrCells());
	    for(int i = perm.size() - 1; i >= 0; --i)
		perm[i] = i;
	    PermuteItems<int>(&perm, m_params.m_nrObjects);
	    
	    for(int i = 0; i < m_params.m_nrObjects; ++i)
	    {
		Problem::Room  *room = m_problem.m_rooms[perm[i]];
		const double dmin = std::min<double>(room->m_depot[2] - room->m_depot[0], 
						     room->m_depot[3] - room->m_depot[1]);
		const double r = 0.5 * dmin * RandomUniformReal(m_params.m_objPercMinRadius, m_params.m_objPercMaxRadius);
		const double x = RandomUniformReal(room->m_depot[0] + r, room->m_depot[2] - r);
		const double y = RandomUniformReal(room->m_depot[1] + r, room->m_depot[3] - r);
		const int    nv= RandomUniformInteger(m_params.m_objMinNrVertices, m_params.m_objMaxNrVertices);
	
		obj = new Problem::Object();
		Algebra2D::IdentityAsTransAngle(obj->m_cfg);
		obj->m_cfg[0] = x;
		obj->m_cfg[1] = y;
		obj->m_poly.m_vertices.resize(2 * nv);
		CircleAsPolygon2D(0, 0, r, nv, &(obj->m_poly.m_vertices[0]));
		obj->m_poly.OnShapeChange();
		
		m_problem.m_objects.push_back(obj);
	    }
	}

	void CreateMaze(void)
	{
	    m_maze.GenerateKruskal(m_problem.m_grid.GetDims()[0], m_problem.m_grid.GetDims()[1]);
	    m_maze.KeepBlocked(m_params.m_mazePercKeepBlocked);
	    m_maze.AddBlockedWalls(&(m_problem.m_grid), m_params.m_mazeWallThickness, &(m_problem.m_obstacles.m_polys));
	}

	void CreateDoors(void)
	{
	    double box[4], obs1[4], obs2[4];
	    int    dir;
	    Problem::Door  *door;
	    double dall;;
	    double ddoor;
	    double dobs;
	    Polygon2D *poly;
	    
	    const int nrEmpty = m_maze.m_empty.size();
	    for(int i = 0; i < nrEmpty; ++i)
	    {
		door            = new Problem::Door();
		door->m_idRoom1 = m_maze.m_empty[i].m_cids[0];
		door->m_idRoom2 = m_maze.m_empty[i].m_cids[1];
		m_problem.m_doors.push_back(door);
		
		m_problem.m_rooms[door->m_idRoom1]->m_idsDoors.push_back(m_problem.m_doors.size() - 1);
		m_problem.m_rooms[door->m_idRoom2]->m_idsDoors.push_back(m_problem.m_doors.size() - 1);

		m_maze.GetWall(&m_problem.m_grid, m_maze.m_empty[i], m_params.m_mazeWallThickness, box, &box[2]);
		dir     = m_maze.m_empty[i].m_cids[0] == (m_maze.m_empty[i].m_cids[1] + 1);
		dall    = box[2 + dir] - box[dir];
		ddoor   = dall * RandomUniformReal(m_params.m_doorPercMinDim, m_params.m_doorPercMaxDim);
		dobs    = 0.5 * (dall - ddoor);

		if(dir == 0) //horizontal wall 
		{
		    printf("horizontal: %f %f %f %f (dall=%f ddoor=%f dobs = %f)\n",
			   box[0], box[1], box[2], box[3], dall, ddoor, dobs);
		    
		    obs1[0] = box[0];        obs1[1] = box[1];
		    obs1[2] = box[0] + dobs; obs1[3] = box[3];

		    obs2[0] = box[2] - dobs; obs2[1] = box[1];
		    obs2[2] = box[2];        obs2[3] = box[3];
		    
		    door->m_box[0] = box[0] + dobs; door->m_box[1] = box[1];
		    door->m_box[2] = box[2] - dobs; door->m_box[3] = box[3];
		}
		else
		{
		    obs1[0] = box[0]; obs1[1] = box[1];
		    obs1[2] = box[2]; obs1[3] = box[1] + dobs;
		    
		    obs2[0] = box[0]; obs2[1] = box[3] - dobs;
		    obs2[2] = box[2]; obs2[3] = box[3];
		    
		    door->m_box[0] = box[0]; door->m_box[1] = box[1] + dobs;
		    door->m_box[2] = box[2]; door->m_box[3] = box[3] - dobs;
		}

		poly = new Polygon2D();
		poly->m_vertices.resize(8);
		AABoxAsPolygon2D(obs1, &obs1[2], &(poly->m_vertices[0]));
		poly->OnShapeChange();
		m_problem.m_obstacles.m_polys.push_back(poly);

		poly = new Polygon2D();
		poly->m_vertices.resize(8);
		AABoxAsPolygon2D(obs2, &obs2[2], &(poly->m_vertices[0]));
		poly->OnShapeChange();
		m_problem.m_obstacles.m_polys.push_back(poly);
	    }
	}
	

	void CreateGoals(void)
	{
	    std::vector<int> perm, orooms;
	    double           p[2];
	    
	    orooms.resize(m_problem.m_objects.size());
	    for(int i = orooms.size() - 1; i >= 0; --i)
	    {
		m_problem.m_objects[i]->m_poly.GetSomePointInside(p);
		Algebra2D::TransAngleMultPoint(m_problem.m_objects[i]->m_cfg, p, p);
		orooms[i] = m_problem.m_grid.GetCellId(p);
	    }
	    
	    perm.resize(m_problem.m_rooms.size());
	    for(int i = perm.size() - 1; i >= 0; --i)
		perm[i] = i;
	    
	    bool reject = false;
	    
	    do
	    {
		PermuteItems<int>(&perm, m_problem.m_objects.size());
		reject = false;
		for(int i = orooms.size() - 1; i >= 0 && !reject; --i)
		    reject = orooms[i] == perm[i];
		
	    }
	    while(reject);
	    
	    for(int i = 0; i < (int) m_problem.m_objects.size(); ++i)
		m_problem.m_objects[i]->m_idGoalRoom = perm[i];
	}


	void CreateBoundaries(void)
	{
	    const double hd = 0.5 * m_params.m_dim;
	    const double w  = m_params.m_mazeWallThickness;
	    Polygon2D *poly;
	    double     min[2];
	    double     max[2];
	
	    min[0] = -hd;  min[1] = -hd - w;
	    max[0] =  hd;  max[1] = -hd;
	    poly = new Polygon2D();
	    poly->m_vertices.resize(8);
	    AABoxAsPolygon2D(min, max, &(poly->m_vertices[0]));
	    poly->OnShapeChange();
	    m_problem.m_obstacles.m_polys.push_back(poly);
	    
	    min[0] = -hd;  min[1] =  hd;
	    max[0] =  hd;  max[1] =  hd + w;
	    poly = new Polygon2D();
	    poly->m_vertices.resize(8);
	    AABoxAsPolygon2D(min, max, &(poly->m_vertices[0]));
	    poly->OnShapeChange();
	    m_problem.m_obstacles.m_polys.push_back(poly);

	    min[0] = -hd - w;  min[1] = -hd - w;
	    max[0] = -hd;      max[1] =  hd + w;
	    poly = new Polygon2D();
	    poly->m_vertices.resize(8);
	    AABoxAsPolygon2D(min, max, &(poly->m_vertices[0]));
	    poly->OnShapeChange();
	    m_problem.m_obstacles.m_polys.push_back(poly);

	    min[0] =  hd;      min[1] = -hd - w;
	    max[0] =  hd + w;   max[1] =  hd + w;
	    poly = new Polygon2D();
	    poly->m_vertices.resize(8);
	    AABoxAsPolygon2D(min, max, &(poly->m_vertices[0]));
	    poly->OnShapeChange();
	    m_problem.m_obstacles.m_polys.push_back(poly);
	}

	void CreateRobot(void)
	{
	    Polygon2D      poly;
	    Problem::Room *room;
	    double         TR2[Algebra2D::TransRot_NR_ENTRIES];
	    
	    
	    poly.m_vertices.resize(8);
	    AABoxAsPolygon2D(m_robot.m_tmeshCollisionWithAttachment.GetBoundingBoxMin(), 
			     m_robot.m_tmeshCollisionWithAttachment.GetBoundingBoxMax(), &(poly.m_vertices[0]));
	    poly.OnShapeChange();
 	    m_robotPoly.m_vertices.resize(8);
	    
	    do
	    {
		room = m_problem.m_rooms[RandomUniformInteger(0, m_problem.m_rooms.size() - 1)];
		TR2[0] = RandomUniformReal(room->m_box[0] + 0.1 * (room->m_box[2] - room->m_box[0]), room->m_box[2] - 0.1 * (room->m_box[2] - room->m_box[0]));
		TR2[1] = RandomUniformReal(room->m_box[1] + 0.1 * (room->m_box[3] - room->m_box[1]), room->m_box[3] - 0.1 * (room->m_box[3] - room->m_box[1]));
		Algebra2D::RotSampleUniform(&TR2[Algebra2D::Trans_NR_ENTRIES]);
		
		ApplyTransRotToPolygon2D(TR2, 4, &(poly.m_vertices[0]), &(m_robotPoly.m_vertices[0]));
		m_robotPoly.OnShapeChange();
	    }   
	    while(IsObstacleClear(&m_robotPoly, room) == false);

	    Algebra2D::TransRotAsTransAngle(TR2, m_problem.m_initRobotTA);
	}
	

	void CreateRandomObstacles(void)
	{
	    Polygon2D     *poly = new Polygon2D();
	    Problem::Room *room;
	    
	    for(int i = 0; i < m_params.m_nrRandObs; ++i)
	    {
		do
		{
		    room = m_problem.m_rooms[RandomUniformInteger(0, m_problem.m_rooms.size() - 1)];
		    const double   dmin = std::min<double>(room->m_box[2] - room->m_box[0], room->m_box[3] - room->m_box[1]);
		    const double   r    = 0.5 * dmin * RandomUniformReal(m_params.m_obsPercMinRadius, m_params.m_obsPercMaxRadius);
		    const double   x    = RandomUniformReal(room->m_box[0] + r, room->m_box[2] - r);
		    const double   y    = RandomUniformReal(room->m_box[1] + r, room->m_box[3] - r);
		    const int      nv   = RandomUniformInteger(m_params.m_obsMinNrVertices, m_params.m_obsMaxNrVertices);
		    
		    poly->m_vertices.resize(2 * nv);
		    CircleAsPolygon2D(x, y, r, nv, &(poly->m_vertices[0]));
		    poly->MakeCCW();
		}   
		while(IsObstacleClear(poly, room) == false);

		m_problem.m_obstacles.m_polys.push_back(poly);
		poly = new Polygon2D();
	    }
	    delete poly;
	}

	bool IsObstacleClear(Polygon2D * const poly, Problem::Room * const room)
	{
	    Polygon2D depot;
	    double    pmin1[2];
	    double    pmin2[2];
	    
	    depot.m_vertices.resize(8);
	    AABoxAsPolygon2D(room->m_depot, &room->m_depot[2], &(depot.m_vertices[0]));
	    depot.OnShapeChange();
	    if(poly->CollisionPolygon(&depot) ||
	       poly->DistSquaredPolygon(&depot, pmin1, pmin2) < m_params.m_obsSep * m_params.m_obsSep)
		return false;

	    AABoxAsPolygon2D(room->m_box, &room->m_box[2], &(depot.m_vertices[0]));
	    depot.OnShapeChange();
	    if(poly->DistSquaredPolygon(&depot, pmin1, pmin2) < m_params.m_obsSep * m_params.m_obsSep)
		return false;
	
	    const int n = m_problem.m_obstacles.m_polys.size();
	    for(int i = 0; i < n; ++i)
		if(poly->CollisionPolygon(m_problem.m_obstacles.m_polys[i]) ||
		   poly->DistSquaredPolygon(m_problem.m_obstacles.m_polys[i], pmin1, pmin2) < m_params.m_obsSep * m_params.m_obsSep)
		    return false;

	    if(poly != &m_robotPoly && m_robotPoly.m_vertices.size() > 0)
	    {
		if(poly->CollisionPolygon(&m_robotPoly) ||
		   poly->DistSquaredPolygon(&m_robotPoly, pmin1, pmin2) < m_params.m_obsSep * m_params.m_obsSep)
		    return false;
	    }
	    
	    return true;
	}
	
	
	

	void Instance(void)
	{
	    m_problem.m_grid.Setup2D(m_params.m_nrCellsX,   m_params.m_nrCellsY,
				    -0.5 * m_params.m_dim, -0.5 * m_params.m_dim,
				     0.5 * m_params.m_dim,  0.5 * m_params.m_dim);
	    CreateBoundaries();
	    CreateRooms();
	    CreateObjects();
	    CreateMaze();
	    CreateDoors();
	    CreateGoals();
	    CreateRobot();
	    CreateRandomObstacles();

//remove boundaries
	    const int n = m_problem.m_obstacles.m_polys.size();
	    for(int i = 0; i < 4; ++i)
	    {
		delete m_problem.m_obstacles.m_polys[i];
		m_problem.m_obstacles.m_polys[i] = m_problem.m_obstacles.m_polys.back();
		m_problem.m_obstacles.m_polys.pop_back();
	    }
	    
	}
	
	
	CreateProblemParams m_params;
	Problem             m_problem;
	Maze                m_maze;
	RobotSCar           m_robot;
	Polygon2D           m_robotPoly;
    };
    
	

}

extern "C" void RunCreateProblem(int argc, char **argv)
{
    Abetare::CreateProblem cp;

    if(argc > 2)
    {
	FILE *in = fopen(argv[2], "r");
	cp.m_params.Read(in);
	fclose(in);
    }
    cp.m_params.Print();
    cp.Instance();

    FILE *out = fopen(argv[1], "w");
    cp.m_problem.Print(out);
    fclose(out);
}


