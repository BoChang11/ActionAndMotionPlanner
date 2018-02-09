#include "MP/Planner.hpp"
#include "MP/ActionPlan.hpp"
#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"


namespace Abetare
{	
    class GRunPlannerManager : public GManager
    {
    public:
	enum
	    {
		FLAG_DRAW_TREE = 1,
		FLAG_PAUSE = 2,
		FLAG_STEER = 4,
		FLAG_RUN  = 8
	    };
	
	
	GRunPlannerManager(void) : GManager()
	{
	    m_timer   = 5;
	    m_flags   = FLAG_DRAW_TREE;
	    m_dstate  = NULL;
	    m_cstate  = NULL;
	   
	    GlobalCamera();

	    m_attach = -1;
	}

	
	virtual ~GRunPlannerManager(void)
	{
	    m_planner.m_continuousSpace.Delete(m_cstate);
	    m_planner.m_discreteSpace.Delete(m_dstate);
	}
	

	virtual void HandleEventOnDisplay(void)
	{
	    GManager::HandleEventOnDisplay();

	    
	    m_planner.m_problem.DrawDoors();
	    m_planner.m_problem.DrawRooms();
	    m_planner.m_problem.DrawObstacles();
	    m_planner.m_problem.DrawGoals();

	    m_planner.m_decomposition.Draw();
 

	    m_planner.DrawRobotAndObjects(m_cstate, m_attach);

	    if(HasFlag(m_flags, FLAG_DRAW_TREE))
	    {
		m_planner.DrawVertices();
		m_planner.DrawEdges();
	    }
	    
	    if(HasFlag(m_flags, FLAG_STEER))
	    {
		GDrawColor(1.0, 0.0, 0.0);
		GDrawSphere3D(m_target, 1.0);
	    }
	    	    
	}	
	
	virtual bool HandleEventOnNormalKeyPress(const int key)
	{
	    
	    if(key == 'p')
		m_flags = FlipFlag(m_flags, FLAG_PAUSE);
	    else if(key == 's')
		m_flags = FlipFlag(m_flags, FLAG_STEER);
	    else if(key == 't')
		m_flags = FlipFlag(m_flags, FLAG_DRAW_TREE);
	    else if(key == 'r')
		m_flags = FlipFlag(m_flags, FLAG_RUN);
	    else if(key >= '1' && key <= '9')
	    {
		m_planner.m_plannerType = Planner::PLANNER_NEW;

		m_planner.Run(key - '0');
		if(m_planner.IsSolved())
		{
		    m_pos = 0;
		    m_planner.GetSolution(&m_path);
		}
		else
		{
		    m_planner.m_continuousSpace.Copy(m_cstate, m_planner.m_vertices.back()->m_continuousState);
		    m_attach = m_planner.m_vertices.back()->m_idObjectAttached;
		}

		printf("....total time = %f solved = %d\n", m_planner.m_stats.m_times[PlannerStats::RUN], m_planner.IsSolved());
		m_planner.m_stats.Print(stdout);
		
		
	    }
	       else if(key == '-')
	    {
		if(m_attach >= 0)
		    m_planner.m_continuousSpace.UpdateObjectStateWhenReleased(m_cstate, m_attach);
		m_attach = -1;
	    }
	    else if(key == 'a')
	    {
		m_planner.m_continuousSpace.MapToDiscreteState(m_cstate, m_attach, m_dstate);
		std::string msg;
		m_planner.PDDLProb(m_dstate, &msg);
		FILE *out = fopen("PDDLsoko/problem.pddl", "w");
		fprintf(out, "%s\n", msg.c_str());
		fclose(out);
		
	    }
	    else if(key == 'b')
	    {
		ActionPlan ap;
		m_planner.m_continuousSpace.MapToDiscreteState(m_cstate, m_attach, m_dstate);
		ap.Compute(m_dstate);
		
		//	ap.Read("sas_plan");

		printf("plan length %d and cost %f\n", 
		       ap.m_plan.size(),
		       ap.m_cost);
		
	    }
	    
	    
	    
	    return GManager::HandleEventOnNormalKeyPress(key);
	}
	
	virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
	{
	    MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
	    if(HasFlag(m_flags, FLAG_STEER))
	    {
		m_planner.m_robot.StartSteerToPosition(m_target);
	    }
	    
	    return true;
	}
	
	virtual void HandleEventOnTimer(void)
	{   
/*
	    if(HasFlag(m_flags, FLAG_STEER) && 
	       m_planner.m_robot.ReachedPosition(m_cstate, m_target) == false &&
	       m_planner.m_robot.IsStateWithinBounds(m_cstate) == true)
	    {
		std::vector<double> s, u;
		s.resize(m_planner.m_robot.m_stateAllocator.GetDim());
		u.resize(m_planner.m_robot.m_controlAllocator.GetDim());
		m_planner.m_robot.SteerToPosition(m_cstate, m_target, &u[0]);
		m_planner.m_robot.SimulateOneStep(m_cstate, &u[0], &s[0]);
		if(m_planner.m_robot.IsStateWithinBounds(&s[0]) == true) 
		    m_planner.m_robot.m_stateAllocator.Copy(m_cstate, &s[0]);
	    }
*/
	    if(HasFlag(m_flags, FLAG_PAUSE))
		return;
	    if(HasFlag(m_flags, FLAG_RUN) && m_planner.IsSolved() == false)
	    {
		printf("running it\n");
		m_planner.Run(0.1);
	    }
	    
	    if(m_planner.IsSolved())
		m_flags = RemoveFlag(m_flags, FLAG_RUN);
	    
	    
	    if(m_path.size() > 0)
	    {
		++m_pos;
		if(m_pos >= m_path.size())
		    m_pos = m_path.size() - 1;
		m_planner.m_continuousSpace.Copy(m_cstate, m_planner.m_vertices[m_path[m_pos]]->m_continuousState);
		m_attach = m_planner.m_vertices[m_path[m_pos]]->m_idObjectAttached;
	    }
	    

	    
	    GManager::HandleEventOnTimer();
	}

	void GlobalCamera(void)
	{
const double eye[] = {0.000000, -196.828386, 176.025100};
const double center[] = {0.000000, 0.000000, 0.000000};
const double right[] = {1.000000, -0.000000, 0.000000};
const double forward[] = {0.000000, 0.731354, -0.681998};	    
	    m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);
	}
	
	Planner m_planner;
	double *m_cstate;
        int    *m_dstate;
	Flags   m_flags;
	double  m_target[3];
	int     m_attach;
	std::vector<int> m_path;
	int              m_pos;
	
	
   };
};


extern "C" int GRunPlanner(int argc, char **argv)
{
    Abetare::GRunPlannerManager gManager;

    FILE *in = argc > 1 ? fopen(argv[1], "r") : NULL;    
    if(in)
    {
	gManager.m_planner.m_problem.Read(in);
	gManager.m_planner.CompleteSetup();
	gManager.m_dstate = gManager.m_planner.m_discreteSpace.New();
	gManager.m_cstate = gManager.m_planner.m_continuousSpace.New();
	gManager.m_planner.m_continuousSpace.GetStateFromProblem(gManager.m_cstate);
	 
	
	fclose(in);
    }
    
   gManager.MainLoop("GRunPlanner", 1280, 720);
    
    return 0;
}
