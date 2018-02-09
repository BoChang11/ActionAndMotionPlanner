#include "MP/Problem.hpp"
#include "MP/Params.hpp"
#include "Utils/Misc.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"

namespace Abetare
{
    Problem::Problem(void)
    {
	Algebra2D::IdentityAsTransAngle(m_initRobotTA);

	DRAWZ_ROOM_BOX       = 0.01;
	DRAWZ_DOOR_BOX       = 0.02;
	DRAWZ_DOOR_ID        = 0.03;
	DRAWZ_ROOM_DEPOT     = 0.02;
	DRAWZ_ROOM_ID        = 0.03;
	DRAWZ_GOAL_ID        = 0.04;
    }
    
    Problem::~Problem(void)
    {
	DeleteItems<Object*>(&m_objects);
	DeleteItems<Room*>(&m_rooms);
	DeleteItems<Door*>(&m_doors);
	DeleteItems<Polygon2D*>(&(m_obstacles.m_polys));
    }
    
/*
    bool Problem::IsSatisfied(const DiscreteState * const ds) const
    {
	const int n = m_objects.size();
	for(int i = 0; i < n; ++i)
	    if(ds->m_objsRooms[i] != m_objects[i]->m_idGoalRoom)
		return false;
	return true;
    }
*/  

    void Problem::ExtrudeObstacles(void)
    {
	m_obstacles.m_tmesh.Clear();
	
	m_obstacles.m_tmesh.AddBoundaries(m_grid.GetMin()[0], m_grid.GetMin()[1], 0,
					  m_grid.GetMax()[0], m_grid.GetMax()[1], 
					  PARAM_PROBLEM_HEIGHT_OBSTACLES, 1.0);
	
	const int nrObs = m_obstacles.m_polys.size();
	for(int i = 0; i < nrObs; ++i)
	    m_obstacles.m_tmesh.AddExtrudedPolygon(m_obstacles.m_polys[i], 0.0, 
						   PARAM_PROBLEM_HEIGHT_OBSTACLES);
    }

    void Problem::ExtrudeObjects(void)
    {
	const int nrObjs = m_objects.size();
	for(int i = 0; i < nrObjs; ++i)
	{
	    m_objects[i]->m_tmesh.Clear();
	    m_objects[i]->m_tmesh.AddExtrudedPolygon(&(m_objects[i]->m_poly), 0.0, 
						     PARAM_PROBLEM_HEIGHT_OBJECTS);
	}
    }
    
    
    void Problem::Print(FILE * const out)
    {
//print grid
	fprintf(out, "Grid2 %lf %lf %lf %lf %d %d\n",
		m_grid.GetMin()[0],
		m_grid.GetMin()[1],
		m_grid.GetMax()[0],
		m_grid.GetMax()[1],
		m_grid.GetDims()[0],
		m_grid.GetDims()[1]);

//print robot init placement
	fprintf(out, "RobotInitCfg %f %f %f\n", m_initRobotTA[0], m_initRobotTA[1], m_initRobotTA[2]);
	
//print rooms
	fprintf(out, "Rooms %d\n", (int) m_rooms.size());
	for(int i = 0; i < (int) m_rooms.size(); ++i)
	{
	    fprintf(out, "%d ", (int) m_rooms[i]->m_idsDoors.size());
	    for(int j = 0; j < m_rooms[i]->m_idsDoors.size(); ++j)
		fprintf(out, "%d ", m_rooms[i]->m_idsDoors[j]);
	    fprintf(out, "%f %f %f %f\n",
		    m_rooms[i]->m_depot[0], m_rooms[i]->m_depot[1],
		    m_rooms[i]->m_depot[2], m_rooms[i]->m_depot[3]);
	}

//print doors
	fprintf(out, "Doors %d\n", (int) m_doors.size());
	for(int i = 0; i < (int) m_doors.size(); ++i)
	    fprintf(out, "%d %d %f %f %f %f\n", 
		    m_doors[i]->m_idRoom1,
		    m_doors[i]->m_idRoom2,
		    m_doors[i]->m_box[0], m_doors[i]->m_box[1],
		    m_doors[i]->m_box[2], m_doors[i]->m_box[3]);

//print objects
	fprintf(out, "Objects %d\n", (int) m_objects.size());
	for(int i = 0; i < (int) m_objects.size(); ++i)
	{
	    fprintf(out, "%d %f %f %f\n", 
		    m_objects[i]->m_idGoalRoom,
		    m_objects[i]->m_cfg[0], m_objects[i]->m_cfg[1], m_objects[i]->m_cfg[2]);
	    m_objects[i]->m_poly.Print(out);
	}

/*
//print discrete state
	fprintf(out, "DiscreteState %d\n", m_ds.m_robotRoom);
	for(int i = 0; i < (int) m_ds.m_objsRooms.size(); ++i)
	    fprintf(out, "%d ", m_ds.m_objsRooms[i]);
	fprintf(out, "\n");
*/	

	

//print obstacles
	fprintf(out, "Obstacles %d\n", (int) m_obstacles.m_polys.size());
	for(int i = 0; i < (int) m_obstacles.m_polys.size(); ++i)
	    m_obstacles.m_polys[i]->Print(out);
//	StandardTriMeshWriter(out, &m_obstacles.m_tmesh);
    }

    void Problem::Read(FILE * const in)
    {
	char keyword[100];
	
//read grid
	double minx, miny, maxx, maxy;
	int    dimsx, dimsy;
	if(fscanf(in, "%s %lf %lf %lf %lf %d %d",
		  keyword, &minx, &miny, &maxx, &maxy, &dimsx, &dimsy) != 7)
	    OnInputError(printf("Grid2 must have 6 params\n"));
	m_grid.Setup2D(dimsx, dimsy, minx, miny, maxx, maxy);

//read robot init placement
	if(fscanf(in, "%s %lf %lf %lf", keyword, &m_initRobotTA[0], &m_initRobotTA[1], &m_initRobotTA[2]) != 4)
	    OnInputError(printf("RobotInitCfg must have x y theta\n"));
	

//read rooms	
	int nrRooms, nrDoorsInRoom;
	if(fscanf(in, "%s %d", keyword, &nrRooms) != 2)
	    OnInputError(printf("..expecting: Rooms nrRooms\n"));
	m_rooms.resize(nrRooms);
	for(int i = 0; i < nrRooms; ++i)
	{
	    m_rooms[i] = new Room();
	    if(fscanf(in, "%d", &nrDoorsInRoom) != 1)
		OnInputError(printf("..expecting nrDoors for room %d\n", i));
	    m_rooms[i]->m_idsDoors.resize(nrDoorsInRoom);
	    for(int j = 0; j < nrDoorsInRoom; ++j)
		if(fscanf(in, "%d", &(m_rooms[i]->m_idsDoors[j])) != 1)
		    OnInputError(printf("..expecting door %d for room %d\n", j, i));
	    if(fscanf(in, "%lf %lf %lf %lf\n",
		      &(m_rooms[i]->m_depot[0]), &(m_rooms[i]->m_depot[1]),
		      &(m_rooms[i]->m_depot[2]), &(m_rooms[i]->m_depot[3])) != 4)
		OnInputError(printf("..expecting depot for room %d\n", i));

	    m_grid.GetCellFromId(i, m_rooms[i]->m_box, &(m_rooms[i]->m_box[2]));
	}

//read doors
	int nrDoors;
	if(fscanf(in, "%s %d", keyword, &nrDoors) != 2)
	    OnInputError(printf("..expecting Doors nrDoors\n"));
	m_doors.resize(nrDoors);
	for(int i = 0; i < nrDoors; ++i)
	{
	    m_doors[i ] = new Door();
	    if(fscanf(in, "%d %d %lf %lf %lf %lf", 
		      &(m_doors[i]->m_idRoom1),
		      &(m_doors[i]->m_idRoom2),
		      &(m_doors[i]->m_box[0]), &(m_doors[i]->m_box[1]),
		      &(m_doors[i]->m_box[2]), &(m_doors[i]->m_box[3])) != 6)
		OnInputError(printf("..expecting 7 params for door %d\n", i));
	}
	

//read objects
	int nrObjs;
	if(fscanf(in, "%s %d", keyword, &nrObjs) != 2)
	    OnInputError(printf("..expecting Objects nrObjs\n"));
	m_objects.resize(nrObjs);
	for(int i = 0; i < nrObjs; ++i)
	{
	    m_objects[i] = new Object();
	    if(fscanf(in, "%d %lf %lf %lf", 
		      &m_objects[i]->m_idGoalRoom,
		      &m_objects[i]->m_cfg[0], 
		      &m_objects[i]->m_cfg[1], 
		      &m_objects[i]->m_cfg[2]) != 4)
		OnInputError(printf("..expecting id, idRoom, TR2 for object %d\n", i));
	    m_objects[i]->m_poly.Read(in);
	}


/*
//read discrete state
	m_ds.m_objsRooms.resize(nrObjs);
	if(fscanf(in, "%s %d", keyword, &m_ds.m_robotRoom) != 2)
	    OnInputError(printf("..expecting robot room location\n"));
	for(int i = 0; i < (int) m_ds.m_objsRooms.size(); ++i)
	    if(fscanf(in, "%d", &m_ds.m_objsRooms[i]) != 1)
		OnInputError(printf("..expecting object %d room location\n", i));
*/	


//read obstacles
	int nrObs;
	if(fscanf(in, "%s %d", keyword, &nrObs) != 2)
	    OnInputError(printf("..expecting Obstacles nrObs\n"));
	m_obstacles.m_polys.resize(nrObs);
	for(int i = 0; i < nrObs; ++i)
	{
	    m_obstacles.m_polys[i] = new Polygon2D();
	    m_obstacles.m_polys[i]->Read(in);
	}
//	StandardTriMeshReader(in, &m_obstacles.m_tmesh);

	ExtrudeObjects();
	ExtrudeObstacles();
    }

    void Problem::Draw(void)
    {
	DrawDoors();
	DrawRooms();
	DrawObjects();
	DrawObstacles();
	DrawGoals();
    }
    
    void Problem::DrawDoors(void)
    {
	char      msg[100];
	GMaterial gmat;
	
	GDraw2D();
	GDrawColor(0.8, 1.0, 0.8);
	GDrawPushTransformation();
	GDrawMultTrans(0, 0, DRAWZ_DOOR_BOX);
	for(int i = (int) m_doors.size() - 1; i >= 0; --i)
	    GDrawAABox2D(m_doors[i]->m_box);
	GDrawPopTransformation();
	
	GDraw3D();
	gmat.SetRuby();
	gmat.SetDiffuse(0.2, 0.2, 0.2);
	GDrawMaterial(&gmat);
	for(int i = (int) m_doors.size() - 1; i >= 0; --i)
	{
	    sprintf(msg, "D%d", i + 1);
	    GDrawString3D(msg, 
			  0.5 * (m_doors[i]->m_box[0] + m_doors[i]->m_box[2]), 
			  0.5 * (m_doors[i]->m_box[1] + m_doors[i]->m_box[3]) - 1.5, 
			  DRAWZ_DOOR_ID, false, 3.0);
	}
	
    }

    void Problem::DrawRooms(void)
    {
	char      msg[100];
	GMaterial gmat;
	
	GDraw2D();

	GDrawColor(1.0, 0.8, 0.8);
	GDrawPushTransformation();
	GDrawMultTrans(0, 0, DRAWZ_ROOM_BOX);
//	for(int i = (int) m_rooms.size() - 1; i >= 0; --i)
//	    GDrawAABox2D(m_rooms[i]->m_box);
	GDrawAABox2D(m_grid.GetMin(), m_grid.GetMax());
	GDrawPopTransformation();

	GDrawColor(0.8, 0.8, 1.0);
	GDrawPushTransformation();
	GDrawMultTrans(0, 0, DRAWZ_ROOM_DEPOT);
	for(int i = (int) m_rooms.size() - 1; i >= 0; --i)
	    GDrawAABox2D(m_rooms[i]->m_depot);
	GDrawPopTransformation();

	
	GDraw3D();
	gmat.SetRuby();
	GDrawMaterial(&gmat);
	for(int i = (int) m_rooms.size() - 1; i >= 0; --i)
	{
	    sprintf(msg, "R%d", i+1);
	    GDrawString3D(msg, 
			  0.1 * m_rooms[i]->m_box[0] + 0.9 * m_rooms[i]->m_box[2], 
			  0.1 * m_rooms[i]->m_box[1] + 0.9 * m_rooms[i]->m_box[3] - 3, 
			  DRAWZ_ROOM_ID, false, 3.0);
	}
	
    }

    void Problem::DrawObjects(void)
    {
	char      msg[100];
	GMaterial gmat;
	double    T[Algebra3D::Trans_NR_ENTRIES];
	double    R[Algebra3D::Rot_NR_ENTRIES];
	double    p[2];
	

	for(int i = (int) m_objects.size() - 1; i >= 0; --i)
	{
//draw shape
	    GDrawPushTransformation();
	    T[0] = m_objects[i]->m_cfg[0];
	    T[1] = m_objects[i]->m_cfg[1];
	    T[2] = 0;
	    Algebra3D::ZAxisAngleAsRot(m_objects[i]->m_cfg[Algebra2D::Trans_NR_ENTRIES], R);
	    GDrawMultTransRot(T, R);
	    gmat.SetGold();
	    GDrawMaterial(&gmat);
	    m_objects[i]->m_tmesh.Draw();

//draw id
	    m_objects[i]->m_poly.GetSomePointInside(p);
	    GDrawMaterial(&gmat);
	    sprintf(msg, "O%d", i + 1);
	    gmat.SetObsidian();
	    GDrawMaterial(&gmat);
	    GDrawString3D(msg, p[0], p[1] - 2.0, PARAM_PROBLEM_HEIGHT_OBJECTS, false, 4.0);

	    GDrawPopTransformation();
	}
	
    }

    void Problem::DrawObstacles(void)
    {
	GMaterial gmat;
	
	gmat.SetTurquoise();
	GDrawMaterial(&gmat);
	m_obstacles.m_tmesh.Draw();
    }

    void Problem::DrawGoals(void)
    {
	char      msg[100];
	GMaterial gmat;

	
	GDraw3D();
	gmat.SetObsidian();
	GDrawMaterial(&gmat);
	for(int i = (int) m_objects.size() - 1; i >= 0; --i)
	{
	    sprintf(msg, "G%d", i+1);
	    const double *box = m_rooms[m_objects[i]->m_idGoalRoom]->m_depot;
	    
	    GDrawString3D(msg, 
			  box[0] + 1.5, box[3] - 3, 
			  DRAWZ_GOAL_ID, false, 3.0);
	}
    }
    
    
}
