#include "MP/Problem.hpp"
#include "Utils/GManager.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Flags.hpp"
#include "MP/RobotSCar.hpp"

namespace Abetare
{	
    class GRunProblemManager : public GManager
    {
    public:
	enum
	    {
		FLAG_STEER = 1
	    };
	
	
	GRunProblemManager(void) : GManager()
	{
	    m_timer   = 5;
	    m_flags   = 0;

	    GlobalCamera();

	    for(int i = 0; i < m_robot.m_stateAllocator.GetDim(); ++i)
		m_state[i] = 0;
	    m_target[0] = m_target[1] = m_target[2];
	}

	
	virtual ~GRunProblemManager(void)
	{
	}
	
	virtual void HandleEventOnDisplay(void)
	{
	    GManager::HandleEventOnDisplay();

	    GDraw2D();
	    GDrawColor(1.0, 0.8, 0.8);
	    GDrawAABox2D(m_problem.m_grid.GetMin(), m_problem.m_grid.GetMax());
	    GDraw3D();
	    
	    m_robot.Draw(m_state);
	    m_problem.Draw();

	    if(HasFlag(m_flags, FLAG_STEER))
	    {
		GDrawColor(1.0, 0.0, 0.0);
		GDrawSphere3D(m_target, 1.0);
	    }
	    
	}	
	
	virtual bool HandleEventOnNormalKeyPress(const int key)
	{
	    if(key == 's')
		m_flags = FlipFlag(m_flags, FLAG_STEER);

	    printf("flags %d\n", m_flags);
	    
	    
	    return GManager::HandleEventOnNormalKeyPress(key);
	}
	
	virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
	{
		MousePosFromScreenToWorld(x, y, &m_target[0], &m_target[1], &m_target[2]);
	    if(HasFlag(m_flags, FLAG_STEER))
	    {
		m_robot.StartSteerToPosition(m_target);
	    }
	    
	    return true;
	}
	
	virtual void HandleEventOnTimer(void)
	{   
	    if(HasFlag(m_flags, FLAG_STEER) && 
	       m_robot.ReachedPosition(m_state, m_target) == false &&
	       m_robot.IsStateWithinBounds(m_state) == true)
	    {
		std::vector<double> s, u;
		s.resize(m_robot.m_stateAllocator.GetDim());
		u.resize(m_robot.m_controlAllocator.GetDim());
		m_robot.SteerToPosition(m_state, m_target, &u[0]);
		m_robot.SimulateOneStep(m_state, &u[0], &s[0]);
		if(m_robot.IsStateWithinBounds(&s[0]) == true) 
		    m_robot.m_stateAllocator.Copy(m_state, &s[0]);
	    }
	    
	    GManager::HandleEventOnTimer();
	}

	void GlobalCamera(void)
	{
	    const double eye[] = {0.000000, -194.782392, 178.219162};
	    const double center[] = {0.000000, 0.000000, 0.000000};
	    const double right[] = {1.000000, -0.000000, 0.000000};
	    const double forward[] = {0.000000, 0.731354, -0.681998};
	    
	    m_gCamera.SetEyeCenterRightForward(eye, center, right, forward);
	}
	
	Problem   m_problem;
	Flags     m_flags;
	RobotSCar m_robot;
	double    m_state[100];
	double    m_target[3];
    };
};


extern "C" int GRunProblem(int argc, char **argv)
{
    Abetare::GRunProblemManager gManager;

    FILE *in = argc > 1 ? fopen(argv[1], "r") : NULL;    
    if(in)
    {
	gManager.m_problem.Read(in);
	gManager.m_state[Abetare::RobotSCar::STATE_X] = gManager.m_problem.m_initRobotTA[0];
	gManager.m_state[Abetare::RobotSCar::STATE_Y] = gManager.m_problem.m_initRobotTA[1];
	gManager.m_state[Abetare::RobotSCar::STATE_THETA] = gManager.m_problem.m_initRobotTA[2];
	fclose(in);
    }
    
   gManager.MainLoop("GRunProblem", 1280, 720);
    
    return 0;
}
