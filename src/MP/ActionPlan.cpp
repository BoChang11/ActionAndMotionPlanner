#include "MP/ActionPlan.hpp"
#include "MP/Planner.hpp"
#include "Utils/PrintMsg.hpp"
#include "Utils/Timer.hpp"
#include <string>
#include <cstdlib>
#include <cstdio>

namespace Abetare
{
    void ActionPlan::RecomputeCost(void)
    {
	m_cost = 0;
	
	const int n = m_plan.size();
	for(int i = 0; i < n; ++i)
	    m_cost += Planner::GetInstance()->m_mapActionsToCosts.find(m_plan[i])->second;
	
    }
    
    bool ActionPlan::Compute(const int ds[])
    {
	std::string  msg;
	char         cmd[300];
	Timer::Clock clk;
	
	Timer::Start(&clk);
	
	Planner::GetInstance()->PDDLProb(ds, &msg);
	
	FILE *out = fopen("PDDLsoko/problem.pddl", "w");
	fprintf(out, "%s\n", msg.c_str());
	fclose(out);
	
//	system("../External/FastDownward/src/fast-downward.py --alias seq-opt-fdss-1 --log-level warning PDDLsoko/problem.pddl");
//	system("../External/FastDownward/src/fast-downward.py --alias seq-opt-merge-and-shrink --log-level warning PDDLsoko/problem.pddl");

//	system("../External/FastDownward/src/fast-downward.py --alias seq-sat-lama-2011 --log-level warning PDDLsoko/problem.pddl");

	system(Planner::GetInstance()->m_actionPlanCmd);
	Read("sas_plan");	
	
	Planner::GetInstance()->m_stats.m_times[PlannerStats::AI_TIME]   += Timer::Elapsed(&clk);
	Planner::GetInstance()->m_stats.m_times[PlannerStats::AI_CALLS]  += 1;
	Planner::GetInstance()->m_stats.m_times[PlannerStats::AI_LENGTH] += m_plan.size();
	

	return m_plan.size() > 0;
	
    }
    
    void ActionPlan::Read(const char fname[])
    {
	FILE   *in = fopen(fname, "r");
	Action  a;
	char    tmp[100];
	
	m_plan.clear();
	
	while(fscanf(in, "%s", tmp) == 1)
	{
	    if(strcmp(tmp, "(move") == 0)
	    {
		a.m_type     = Action::TYPE_MOVE;
		a.m_idObject = Constants::ID_UNDEFINED;
		if(fscanf(in, " r%d r%d d%d)", 
			  &a.m_idFromRoom, 
			  &a.m_idToRoom,
			  &a.m_idDoor) != 3)
		    OnInputError(printf("expecting Ri Rj dk after move\n"));
		m_plan.push_back(a);
	    }
	    else if(strcmp(tmp, "(movewithobject") == 0)
	    {
		a.m_type = Action::TYPE_MOVE_WITH_OBJECT;
		if(fscanf(in, " r%d r%d d%d o%d)", 
			  &a.m_idFromRoom, 
			  &a.m_idToRoom,
			  &a.m_idDoor,
			  &a.m_idObject) != 4)
		    OnInputError(printf("expecting Ri Rj dk ol after move\n"));
		m_plan.push_back(a);
	    }
	    else if(strcmp(tmp, "(pickupobject") == 0)
	    {
		a.m_type = Action::TYPE_PICKUP;
		a.m_idDoor = Constants::ID_UNDEFINED;
		if(fscanf(in, " r%d o%d)", 
			  &a.m_idFromRoom, 
			  &a.m_idObject) != 2)
		    OnInputError(printf("expecting Ri oj after pickup\n"));
		a.m_idToRoom = a.m_idFromRoom;
		m_plan.push_back(a);
	    }
	    else if(strcmp(tmp, "(releaseobject") == 0)
	    {
		a.m_type = Action::TYPE_RELEASE;
		a.m_idDoor = Constants::ID_UNDEFINED;
		if(fscanf(in, " r%d o%d)", 
			  &a.m_idFromRoom, 
			  &a.m_idObject) != 2)
		    OnInputError(printf("expecting Ri oj after release\n"));
		a.m_idToRoom = a.m_idFromRoom;
		m_plan.push_back(a);
	    }
	    else if(strcmp(tmp, ";") == 0)
	    {
		if(fscanf(in, " cost = %lf", &m_cost) != 1)
		    OnInputError(printf("expecting cost value\n"));
	    }
	}
	
	
	fclose(in);
/*	for(int i = 0; i < m_plan.size(); ++i)
	    printf("%d %d %d %d %d\n", m_plan[i].m_type, m_plan[i].m_idFromRoom, m_plan[i].m_idToRoom, m_plan[i].m_idDoor, m_plan[i].m_idObject);
	printf("cost = %f\n", m_cost);
*/
//	system("more sas_plan");
//	printf("press enter to continue\n");
//	getchar();
	
    }
    
}
