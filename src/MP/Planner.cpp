#include "MP/Planner.hpp"
#include "MP/Params.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Timer.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/GMaterial.hpp"
#include <algorithm>
#include <climits>

namespace Abetare
{
    Planner* Planner::m_instance = NULL;


    Planner::Planner(void)
    {
	m_plannerType = PLANNER_NEW;
	
	m_instance = this;
	m_decompSearch.m_info = &m_decompSearchInfo;
	m_vidAtGoal = Constants::ID_UNDEFINED;
	strcpy(m_actionPlanCmd, "../External/FastDownward/src/fast-downward.py --alias seq-opt-fdss-1 --log-level warning PDDLsoko/problem.pddl");

	m_indexInPlan = 0;
    }
	
    
    Planner::~Planner(void)
    {
	DeleteItems<Vertex*>(&m_vertices);
    }
    
    void Planner::CompleteSetup(void)
    {
	Timer::Clock clk;
	
	Timer::Start(&clk);
	
	m_instance = this;
	m_problem.CompleteSetup();
	m_continuousSpace.CompleteSetup();
	m_discreteSpace.CompleteSetup();
	m_decomposition.CompleteSetup();

	InitActionCosts();
	PDDLFixedPrefix();
	PDDLFixedPostfix();

	AddRoot();

	m_stats.m_times[PlannerStats::COMPLETE_SETUP] += Timer::Elapsed(&clk);
    }

    void Planner::InitActionCosts(void)
    {
    	const int nrRooms = m_problem.m_rooms.size();
	const int nrDoors = m_problem.m_doors.size();
	const int nrObjs  = m_problem.m_objects.size();
	Action    a;
	
	a.m_type = Action::TYPE_MOVE;
	for(int i = 0; i < nrDoors; ++i)
	{
	    a.m_idFromRoom = m_problem.m_doors[i]->m_idRoom1;
	    a.m_idToRoom   = m_problem.m_doors[i]->m_idRoom2;
	    a.m_idDoor     = i;
	    a.m_idObject   = Constants::ID_UNDEFINED;
	    m_mapActionsToCosts.insert(std::make_pair(a, PARAM_COST_MOVE));

	    a.m_idFromRoom = m_problem.m_doors[i]->m_idRoom2;
	    a.m_idToRoom   = m_problem.m_doors[i]->m_idRoom1;
	    m_mapActionsToCosts.insert(std::make_pair(a, PARAM_COST_MOVE));
	}

	a.m_type = Action::TYPE_MOVE_WITH_OBJECT;
	for(int j = 0; j < nrObjs; ++j)
	    for(int i = 0; i < nrDoors; ++i)
	    {
		a.m_idFromRoom = m_problem.m_doors[i]->m_idRoom1;
		a.m_idToRoom   = m_problem.m_doors[i]->m_idRoom2;
		a.m_idDoor     = i;
		a.m_idObject   = j;
		m_mapActionsToCosts.insert(std::make_pair(a, PARAM_COST_MOVE_WITH_OBJECT));
		
		a.m_idFromRoom = m_problem.m_doors[i]->m_idRoom2;
		a.m_idToRoom   = m_problem.m_doors[i]->m_idRoom1;
		m_mapActionsToCosts.insert(std::make_pair(a, PARAM_COST_MOVE_WITH_OBJECT));
	    }

	for(int j = 0; j < nrObjs; ++j)
	    for(int i = 0; i < nrRooms; ++i)
	    {
		a.m_idFromRoom = i;
		a.m_idToRoom   = i;
		a.m_idDoor     = Constants::ID_UNDEFINED;
		a.m_idObject   = j;

		a.m_type = Action::TYPE_PICKUP;
		m_mapActionsToCosts.insert(std::make_pair(a, PARAM_COST_PICKUP));

		a.m_type = Action::TYPE_RELEASE;
		m_mapActionsToCosts.insert(std::make_pair(a, PARAM_COST_RELEASE));
	    }
    }
    
    
    
    double Planner::PathCost(const std::vector<int> * const path) const
    {
	double c = 0;
	for(int i = 1; i < path->size(); ++i)
	    c += Algebra2D::PointDist(m_vertices[(*path)[i-1]]->m_proj,
				      m_vertices[(*path)[i]]->m_proj);
	return c;
    }
    
    void Planner::GetSolution(std::vector<int> * const path) const
    {
	path->clear();
	
	int pid = m_vidAtGoal;
	while(pid >= 0)
	{
	    path->push_back(pid);

	    if(pid >= (int) m_vertices.size())
		printf("path is wrong ... pid = %d nv = %d size=%d\n", pid, (int) m_vertices.size(), (int) path->size());
	    
	    pid = m_vertices[pid]->m_parent;
	}
 	ReverseItems<int>(path);
    }

    bool Planner::Collision(const double cs[], const Action * const a)
    {
	const double *s = m_continuousSpace.GetRobotState(cs);
	double T3robot[Algebra3D::Trans_NR_ENTRIES];
	double R3robot[Algebra3D::Rot_NR_ENTRIES];
	double T3obj[Algebra3D::Trans_NR_ENTRIES];
	double R3obj[Algebra3D::Rot_NR_ENTRIES];
	double T3other[Algebra3D::Trans_NR_ENTRIES];
	double R3other[Algebra3D::Rot_NR_ENTRIES];
	
	T3robot[0] = s[RobotSCar::STATE_X];
	T3robot[1] = s[RobotSCar::STATE_Y];
	T3robot[2] = 0;
	Algebra3D::ZAxisAngleAsRot(s[RobotSCar::STATE_THETA], R3robot);
	
	//robot-obstacles
	if(m_robot.m_tmeshCollisionWithAttachment.Collision(T3robot, R3robot, &m_problem.m_obstacles.m_tmesh, NULL, NULL))
	    return true;


	//robot-other objects
	const int n = m_problem.m_objects.size();
	for(int i = 0; i < n; ++i)
	{
	    if(i == a->m_idObject && (a->m_type == Action::TYPE_MOVE_WITH_OBJECT || a->m_type == Action::TYPE_RELEASE))
		continue;
	    m_continuousSpace.GetObjectTR3(cs, i, T3other, R3other);
	    if(m_problem.m_objects[i]->m_tmesh.Collision(T3other, R3other, &m_robot.m_tmeshCollisionNoAttachment, T3robot, R3robot))
		return true;
	}

	//attachedObject -- obstacles
	if(a->m_type == Action::TYPE_MOVE_WITH_OBJECT ||
	   a->m_type == Action::TYPE_RELEASE)
	{
	    m_continuousSpace.GetObjectTR3WhenAttached(cs, a->m_idObject, T3obj, R3obj);
	    if(m_problem.m_objects[a->m_idObject]->m_tmesh.Collision(T3obj, R3obj, &m_problem.m_obstacles.m_tmesh, NULL, NULL))
		return true;
	}

	return false;
    }



    void Planner::ONERun(const double tmax)
    {
	Timer::Clock clk;
	
	double target[3];
	
	const int nrIters = RandomUniformInteger(PARAM_PLANNER_MIN_NR_ITERS_EXPAND_ACTION, 
						 PARAM_PLANNER_MAX_NR_ITERS_EXPAND_ACTION);
	
	Group  *g;
	Action *a;
	bool    adone;
	
	Timer::Start(&clk);

	if(m_actionPlan.m_plan.size() == 0)
	    m_actionPlan.Compute(m_groups.m_avail[0]->m_discreteState);
	
	while(IsSolved() == false && Timer::Elapsed(&clk) < tmax)
	{ 
	    g = m_groups.m_avail[RandomUniformInteger(0, m_groups.m_avail.size() - 1)];
	    if(g->ONEindexInPlan < 0)
		g->ONEindexInPlan = 0;
	    m_indexInPlan = g->ONEindexInPlan;
	    a     = &m_actionPlan.m_plan[g->ONEindexInPlan];
	    adone = false;

	    for(int i = 0; i < 200 && adone == false && IsSolved() == false; ++i)
	    {
		const int vid = g->ONESelectTargetAndVertex(a, target);
		adone = ExpandFrom(vid, target, a);
		
	    }
	}
	

	m_stats.m_times[PlannerStats::RUN] += Timer::Elapsed(&clk);
     
    }
    
    void Planner::SMAPRun(const double tmax)
    {
	Timer::Clock clk;
	
	double target[3];
	
	const int nrIters = RandomUniformInteger(PARAM_PLANNER_MIN_NR_ITERS_EXPAND_ACTION, 
						 PARAM_PLANNER_MAX_NR_ITERS_EXPAND_ACTION);

	Timer::Start(&clk);
	while(IsSolved() == false && Timer::Elapsed(&clk) < tmax)
	{ 
	    Group *g = m_groups.SMAPSelect();

	    printf("SMAP selected group %p (actions = %d)\n", g, g->SMAPactions.size());
	    
	    g->SMAPNewActionPlan();
	    g->SMAPActionSelect();
	    g->ReadyForNewAction(&(g->SMAPaction));
	    
	    
	    printf("SMAP exploring new action (groups %d) \n", m_groups.m_avail.size());
	    
	    bool adone = false;
	    for(int i = 0; i < nrIters && adone == false && IsSolved() == false; ++i)
	    {
		const int vid = g->SMAPSelectTargetAndVertex(target);
		adone = ExpandFrom(vid, target, &(g->SMAPaction));
		
	    }
	    
	    //if(adone == false)
	    {
		auto cur = m_mapActionsToCosts.find(g->SMAPaction);
		if(cur->second < INT_MAX / PARAM_COST_PENALTY)
		    cur->second *= PARAM_COST_PENALTY;
	       
//	    group->m_actionPlan.m_plan.clear();
	    }

	  
	}
	

	m_stats.m_times[PlannerStats::RUN] += Timer::Elapsed(&clk);
    }

/*    void Planner::SyclopRun(const double tmax)
    {
	Timer::Clock clk;
	
	Timer::Start(&clk);

	int *dsPrev = NULL;
	int *ds     = m_discreteSpace.New();
	
	std::vector<Group *> use;
	
	while(IsSolved() == false && Timer::Elapsed(&clk) < tmax)
	{
	    m_actionPlan.Compute(m_groups.m_avail[0]->m_discreteState);
	    const int n = m_actionPlan.m_plan.size();
	    dsPrev = m_groups.m_avail[0]->m_discreteState;
	    for(int i = 0; i < n; ++i)
	    {
		ApplyAction(dsPrev, &(m_actionPlan.m_plan[i]), ds);
		
	    }
	    
	}

	m_discreteSpace.Delete(ds);
	
	m_stats.m_times[PlannerStats::RUN] += Timer::Elapsed(&clk);
   
    }
*/
    
    void Planner::Run(const double tmax)
    {
	Timer::Clock clk;
	
	Timer::Start(&clk);
	while(IsSolved() == false && Timer::Elapsed(&clk) < tmax)
	    Expand();

	m_stats.m_times[PlannerStats::RUN] += Timer::Elapsed(&clk);
    }
    
    void Planner::Expand(void)
    {
	Group       *group;
	RegionGroup *regionGroup;
	double       target[2];
	int          vid;
	bool         adone = false;
	int          pos;
	Action      *a;
	Timer::Clock clk;

	if(m_groups.m_avail.size() == 1 &&
	   m_groups.m_avail[0]->m_actionPlan.m_plan.size() == 0)
	    m_groups.m_avail[0]->NewPlan();
	
	group = m_groups.Select(&pos);
	if(group->m_actionPlan.m_plan.size() == 0)
	    OnInputError(printf("selected group has no plan. smth wrong\n"));
	
	Timer::Start(&clk);

	m_actionPlan.m_plan.assign(group->m_actionPlan.m_plan.begin(), group->m_actionPlan.m_plan.end());
	m_actionPlan.m_cost = group->m_actionPlan.m_cost;

	

	a = &(group->m_actionPlan.m_plan[0]);
	group->ReadyForNewAction(a);


	m_groups.Print(stdout);
	printf("selected group %d\n", pos);
//	printf("press enter to continue\n");
//	getchar();
	
	int       nrStart = group->m_avail.size();
	bool      failed  = false;
	const int nrIters = RandomUniformInteger(PARAM_PLANNER_MIN_NR_ITERS_EXPAND_ACTION, 
						 PARAM_PLANNER_MAX_NR_ITERS_EXPAND_ACTION);
	for(int i = 0; i < nrIters && adone == false && IsSolved() == false; ++i)
	{
	    const int vid = group->SelectTargetAndVertex(a, target);
	    adone = ExpandFrom(vid, target, a);

	    if(i > 10 && group->m_avail.size() == nrStart)
	    {
		group->OnExpansionFailure();
		failed = true;
		break;
	    }
	}

	if(adone == false)
	{
	    auto cur = m_mapActionsToCosts.find(*a);
	    if(cur == m_mapActionsToCosts.end())
		OnInputError(printf("action t%d fr%d tr%d d%d o%d  not in map",
				    a->m_type,
				    a->m_idFromRoom,
				    a->m_idToRoom,
				    a->m_idDoor,
				    a->m_idObject));
	    if(cur->second < INT_MAX / PARAM_COST_PENALTY)
	    {
	  	cur->second *= PARAM_COST_PENALTY;

		for(int i = m_groups.m_avail.size() - 1; i >= 0; --i)
		    m_groups.m_avail[i]->m_actionPlan.RecomputeCost();
	    }
	    
	    group->ClearPlan();
	    
	    if(failed == false)
		group->NewPlan();

	    printf("new plan for group %d\n", pos);
//	printf("press enter to continue\n");
//	getchar();
	}


	printf("done = %d [plan length = %d] [group %p %f avail = %d]\n", adone, 
	       group->m_actionPlan.m_plan.size(), group, group->GetCost(), group->m_avail.size());
	
	if(failed && m_groups.m_avail.size() > 1)
	{
	    auto cur = std::find(m_groups.m_avail.begin(), m_groups.m_avail.end(), group);
	    m_groups.m_avail.erase(cur);
	    delete group;
	    }
	
	m_stats.m_times[PlannerStats::EXPAND] += Timer::Elapsed(&clk);
  }

    
    bool Planner::ExpandFrom(int vid, const double target[], const Action * const a)
    {
	
	bool                steer;

	if(m_plannerType == PLANNER_NEW)
	    steer = RandomUniformReal() < PARAM_PLANNER_PROB_STEER;
	else if(m_plannerType == PLANNER_ONE)
	    steer = RandomUniformReal() < 0.9;
	else
	    steer = RandomUniformReal() < 0.1;
	
	const int           nrSteps = RandomUniformInteger(PARAM_PLANNER_MIN_NR_STEPS, PARAM_PLANNER_MAX_NR_STEPS);
	const double       *s;
	std::vector<double> u;
	std::vector<double> snew;
	double              p[2];
	bool                adone = false;
	Vertex             *vnew;
	double              pstart[2];
	Timer::Clock        clk;
	
	
	u.resize(m_robot.m_controlAllocator.GetDim());
	snew.resize(m_continuousSpace.GetDim());

	if(steer)
	    m_robot.StartSteerToPosition(target);
	else
	    m_robot.SampleControl(&u[0]);

//	pstart[0] = m_vertices[vid]->m_continuousState[0];
//	pstart[1] = m_vertices[vid]->m_continuousState[1];
	
	for(int i = 0; i < nrSteps && IsSolved() == false; ++i)
	{
	    Timer::Start(&clk);	    
	    s = m_vertices[vid]->m_continuousState;
	    m_continuousSpace.Copy(&snew[0], s);
	    if(steer)
		m_robot.SteerToPosition(m_continuousSpace.GetRobotState(s), target, &u[0]);
	    m_robot.SimulateOneStep(m_continuousSpace.GetRobotState(s), &u[0], m_continuousSpace.GetRobotState(&snew[0]));
	    adone = adone || ActionComplete(&snew[0], a);
	    m_stats.m_times[PlannerStats::SIMULATE] += Timer::Elapsed(&clk);
	    Timer::Start(&clk);
	    if(Collision(&snew[0], a))
	    {
		m_stats.m_times[PlannerStats::COLLISION] += Timer::Elapsed(&clk);
		return false;
	    }
	    m_stats.m_times[PlannerStats::COLLISION] += Timer::Elapsed(&clk);
	    Timer::Start(&clk);
	    vnew = AddVertex(&snew[0], vid, a, adone);
	    m_stats.m_times[PlannerStats::ADD_VERTEX] += Timer::Elapsed(&clk);
	    if(vnew == NULL)
		return false;
	    if(adone)
		return true;
	    vid = vnew->m_idVertex;

//	    if(i > 30 && Algebra2D::PointDist(pstart, &snew[0]) < 8.0)
//		return false;
	    
	}

	return adone;
	
    }

    void Planner::AddRoot(void)
    {
	std::vector<double> s;
	
	s.resize(m_continuousSpace.GetDim());
	m_continuousSpace.GetStateFromProblem(&s[0]);
	AddVertex(&s[0], Constants::ID_UNDEFINED, NULL, false);
    }
    

    Vertex* Planner::AddVertex(const double cs[], const int parent, const Action * const a, const bool adone)
    {
	double p[2];
	int    rid;
	int    idRoom;
	
	
	m_robot.PickupPoint(m_continuousSpace.GetRobotState(cs), p);
	rid = m_decomposition.LocateRegion(p);
	if(rid < 0)
	    return NULL;
	idRoom = m_problem.m_grid.GetCellId(p);
	if(a != NULL && (idRoom != a->m_idFromRoom && idRoom != a->m_idToRoom))
	    return NULL;
		
	Vertex *vnew            = new Vertex();
	vnew->m_idVertex        = m_vertices.size();
	vnew->m_parent          = parent;
	vnew->m_proj[0]         = p[0];
	vnew->m_proj[1]         = p[1];
	vnew->m_continuousState = m_continuousSpace.Copy(cs);
	vnew->m_idRegion        = rid;
	vnew->m_idRoom          = idRoom;
	vnew->m_idGroup         = Constants::ID_UNDEFINED;
	
	//if(a != NULL && (adone == false && parent >= 0 && vnew->m_idRoom == m_vertices[parent]->m_idRoom))
	//    vnew->m_idGroup = m_vertices[parent]->m_idGroup;
	if(a != NULL && (a->m_type == Action::TYPE_MOVE_WITH_OBJECT ||
			 (a->m_type == Action::TYPE_PICKUP && adone == true) ||
			 (a->m_type == Action::TYPE_RELEASE && adone == false)))
	    vnew->m_idObjectAttached = a->m_idObject;
	m_vertices.push_back(vnew);
	m_groups.AddVertex(vnew, a == NULL ? NULL : &m_actionPlan, adone);


	return vnew;
    }
    
    bool Planner::ActionComplete(double cs[], const Action * const a)
    {
	double        p[2];
	const double *depot;
	double        box[4];
	
	if(a->m_type == Action::TYPE_PICKUP)
	{
	    m_continuousSpace.GetObjectPoint(cs, a->m_idObject, p);
	    if(m_robot.CanBePickedUp(m_continuousSpace.GetRobotState(cs), p))
	    {
		m_continuousSpace.UpdateObjectStateWhenPickedUp(cs, a->m_idObject);
		return true;
	    }		
	}
	else if(a->m_type == Action::TYPE_MOVE || a->m_type == Action::TYPE_MOVE_WITH_OBJECT)
	{
   	     m_robot.PickupPoint(m_continuousSpace.GetRobotState(cs), p);
	     return m_problem.m_grid.GetCellId(p)  == a->m_idToRoom;
	}
	else if(a->m_type == Action::TYPE_RELEASE)
	{
	    m_robot.PickupPoint(m_continuousSpace.GetRobotState(cs), p);
	    depot = m_problem.m_rooms[a->m_idToRoom]->m_depot;
	    const double c = PARAM_PROBLEM_DEPOT_RELEASE_FRAC;
	    if(p[0] >= ((1+c)*depot[0]/2 + (1-c)*depot[2]/2) &&
	       p[0] <= ((1-c)*depot[0]/2 + (1+c)*depot[2]/2) &&
	       p[1] >= ((1+c)*depot[1]/2 + (1-c)*depot[3]/2) &&
	       p[1] <= ((1-c)*depot[1]/2 + (1+c)*depot[3]/2))
 /*
minx = (1 + a) * 0.5 * d0 + (1 - a) * 0.5 * d2
maxx = (1 - a) * 0.5 * d0 + (1 + a) * 0.5 * d2
*/
//	    if(IsPointInsideAABox2D(p, depot, &depot[2]))
	    {
		m_continuousSpace.UpdateObjectStateWhenReleased(cs, a->m_idObject);
		return true;
	    }
	}

	return false;
    }
   
    
    void Planner::PDDLFixedPrefix(void)
    {
	m_pddlMsgFixedPrefix.clear();
	PDDLPreamble(&m_pddlMsgFixedPrefix);
	
	m_pddlMsgFixedPrefix.append("(:init\n");
	PDDLConnects(&m_pddlMsgFixedPrefix);
	
    }
    
    void Planner::PDDLFixedPostfix(void)
    {

	m_pddlMsgFixedPostfix.append("   (= (total-cost) 0)\n");
	m_pddlMsgFixedPostfix.append(")\n"); //end init

	PDDLGoals(&m_pddlMsgFixedPostfix);
	PDDLMetric(&m_pddlMsgFixedPostfix);
	m_pddlMsgFixedPostfix.append(")\n");

    }
    

    void Planner::PDDLProb(const int ds[], std::string * const msg)
    {
	const int nrRooms = m_problem.m_rooms.size();
	const int nrDoors = m_problem.m_doors.size();
	const int nrObjs  = m_problem.m_objects.size();
	char      tmp[100];
	std::vector<bool> empty;
	
	msg->clear();
/*	PDDLPreamble(msg);
	
	msg->append("(:init\n");
	PDDLConnects(msg);
*/
	msg->append(m_pddlMsgFixedPrefix.c_str());
	
	sprintf(tmp, "   (robotInRoom R%d)\n", m_discreteSpace.GetRobotRoom(ds));
	msg->append(tmp);
	
	empty.resize(nrRooms);
	std::fill(empty.begin(), empty.end(), true);
	for(int i = 0; i < nrObjs; ++i)
	{
	    sprintf(tmp, "   (objInRoom o%d R%d)\n", i, m_discreteSpace.GetObjectRoom(ds, i));
	    msg->append(tmp);
	    empty[m_discreteSpace.GetObjectRoom(ds, i)] = false;
	}
	for(int i = 0; i < nrRooms; ++i)
	    if(empty[i])
	    {
		sprintf(tmp, "   (empty R%d)\n", i);
		msg->append(tmp);
	    }

	if(m_discreteSpace.GetAttachedObject(ds) >= 0)
	{
	    sprintf(tmp, "   (carry o%d)\n", m_discreteSpace.GetAttachedObject(ds));
	    msg->append(tmp);
	}
	else
	    msg->append("   (robotEmpty)\n");
	PDDLCosts(msg);
/*	msg->append(")\n");

	PDDLGoals(msg);
	PDDLMetric(msg);
	msg->append(")\n");
*/
	msg->append(m_pddlMsgFixedPostfix.c_str());
	
	
    }
    
    void Planner::PDDLPreamble(std::string * const msg)
    {
	const int nrRooms = m_problem.m_rooms.size();
	const int nrDoors = m_problem.m_doors.size();
	const int nrObjs  = m_problem.m_objects.size();
	char      tmp[100];
	
	msg->append("(define (problem SOKO-QUERY1) (:domain SOKO)\n (:objects\n");
	for(int i = 0; i < nrRooms; ++i)
	{
	    sprintf(tmp, "   R%d - room\n", i);
	    msg->append(tmp);
	}
	for(int i = 0; i < nrDoors; ++i)
	{
	    sprintf(tmp, "   d%d - door\n", i);
	    msg->append(tmp);
	}
	for(int i = 0; i < nrObjs; ++i)
	{
	    sprintf(tmp, "   o%d - movable\n", i);
	    msg->append(tmp);
	}
	msg->append(")\n");
    }

    
    void Planner::PDDLConnects(std::string * const msg)
    {
	const int n = m_problem.m_doors.size();
	char tmp[100];
	
	for(int i = 0; i < n; ++i)
	{
	    sprintf(tmp, "   (connects R%d R%d d%d) (connects R%d R%d d%d)\n", 
		    m_problem.m_doors[i]->m_idRoom1,
		    m_problem.m_doors[i]->m_idRoom2, i,
		    m_problem.m_doors[i]->m_idRoom2,
		    m_problem.m_doors[i]->m_idRoom1, i);
	    msg->append(tmp);
	}	    
    }

    void Planner::PDDLCosts(std::string * const msg)
    {
	Action a;
	int    c;
	char   tmp[200];
	
	tmp[0] = '\0';
	for(auto it = m_mapActionsToCosts.begin(); it != m_mapActionsToCosts.end(); ++it)
	{
	    a = it->first;
	    c = (int) (it->second);
	   
	    switch(a.m_type)
	    {
	    case Action::TYPE_MOVE: 
		sprintf(tmp, "   (= (moveCost R%d R%d d%d) %d)\n", a.m_idFromRoom, a.m_idToRoom, a.m_idDoor, c);
		break;
	    case Action::TYPE_MOVE_WITH_OBJECT: 
		sprintf(tmp, "   (= (moveWithObjectCost R%d R%d d%d o%d) %d)\n", a.m_idFromRoom, a.m_idToRoom, a.m_idDoor, a.m_idObject, c);
		break;
	    case Action::TYPE_PICKUP: 
		sprintf(tmp, "   (= (pickupObjectCost R%d o%d) %d)\n", a.m_idFromRoom, a.m_idObject, c);
		break;
	    case Action::TYPE_RELEASE: 
		sprintf(tmp, "   (= (releaseObjectCost R%d o%d) %d)\n", a.m_idFromRoom, a.m_idObject, c);
		break;
	    }
	    msg->append(tmp);
	}

	//msg->append("   (= (total-cost) 0)\n");
	 
	
    }
    
    
    void Planner::PDDLGoals(std::string * const msg)
    {
	const int n = m_problem.m_objects.size();
	char tmp[100];

	msg->append("(:goal (and (robotEmpty)\n");
	for(int i = 0; i < n; ++i)
	{
	    sprintf(tmp, "  (objInRoom o%d R%d)\n", i, m_problem.m_objects[i]->m_idGoalRoom);
	    msg->append(tmp);
	}
	msg->append("))\n");
    }

    void Planner::PDDLMetric(std::string * const msg)
    {
	msg->append("(:metric minimize (total-cost))\n");
    }

    void Planner::DrawVertices(void)
    {
	const bool is2D = GDrawIs2D();
	
	GDrawPushTransformation();
	GDrawMultTrans(0, 0, 0.3);
	GDraw2D();
	GDrawColor(0, 0, 1);
	for(int i = m_vertices.size() - 1; i >= 0; --i)
	    GDrawCircle2D(m_vertices[i]->m_proj[0],
			  m_vertices[i]->m_proj[1], 0.3);
	GDrawPopTransformation();
	
	if(!is2D)
	    GDraw3D();
	
    }
    
    void Planner::DrawEdges(void)
    {
	const bool is2D = GDrawIs2D();
	int        pid;
	
	GDraw2D();
	GDrawColor(1, 0, 0);
	GDrawLineWidth(1.0);
	for(int i = m_vertices.size() - 1; i >= 0; --i)
	    if((pid = m_vertices[i]->m_parent) >= 0)
		GDrawSegment2D(m_vertices[pid]->m_proj, m_vertices[i]->m_proj);
	if(!is2D)
	    GDraw3D();
    }
    
    void Planner::DrawRobotAndObjects(const double s[], const int attached) 
    {	
	m_robot.Draw(m_continuousSpace.GetRobotState(s));
	
	char      msg[100];
	GMaterial gmat;
	double    T[Algebra3D::Trans_NR_ENTRIES];
	double    R[Algebra3D::Rot_NR_ENTRIES];
	double    p[2];
	
	for(int i = (int) m_problem.m_objects.size() - 1; i >= 0; --i)
	{
	    if(i == attached)
		m_continuousSpace.GetObjectTR3WhenAttached(s, i, T, R);
	    else
		m_continuousSpace.GetObjectTR3(s, i, T, R);
	    
//draw shape
	    GDrawPushTransformation();
	    GDrawMultTransRot(T, R);
	    gmat.SetGold();
	    GDrawMaterial(&gmat);
	    m_problem.m_objects[i]->m_tmesh.Draw();

//draw id
	    m_problem.m_objects[i]->m_poly.GetSomePointInside(p);
	    GDrawMaterial(&gmat);
	    sprintf(msg, "%d", i + 1);
	    gmat.SetObsidian();
	    GDrawMaterial(&gmat);
	    GDrawString3D(msg, p[0], p[1] - 2.0, PARAM_PROBLEM_HEIGHT_OBJECTS, false, 4.0);

	    GDrawPopTransformation();
	}

    }

}


/*
  |Pickup(R, o)|  = nrRooms * nrObjs
  |Release(R, o)| = nrRooms * nrObjs
  |Go(Ri, Rj, d)| = |nrEdges|
  |MoveWithObject(Ri, Rj, d, o)| = |nrEdges| * nrObjs
 */
