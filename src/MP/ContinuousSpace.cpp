#include "MP/ContinuousSpace.hpp"
#include "MP/Planner.hpp"
#include "MP/Params.hpp"
#include "Utils/Algebra3D.hpp"

namespace Abetare
{
   void ContinuousSpace::CompleteSetup(void) 
   {
       SetDim(Planner::GetInstance()->m_robot.m_stateAllocator.GetDim() + 
	      3 * Planner::GetInstance()->m_problem.m_objects.size());
   }
    
    double* ContinuousSpace::GetObjectState(double s[], const int i) const
    {
	return &s[Planner::GetInstance()->m_robot.m_stateAllocator.GetDim() + 3 * i];
    }
    
    const double* ContinuousSpace::GetObjectState(const double s[], const int i) const
    {
	return &s[Planner::GetInstance()->m_robot.m_stateAllocator.GetDim() + 3 * i];
    }

    void ContinuousSpace::GetObjectPoint(const double s[], const int i, double p[]) const
    {
	double porig[2];
	const double *cfg = GetObjectState(s, i);
	Planner::GetInstance()->m_problem.m_objects[i]->m_poly.GetSomePointInside(porig);
	Algebra2D::TransAngleMultPoint(cfg, porig, p);
/*
	printf("object %d cfg %f %f %f\n", i, cfg[0], cfg[1], cfg[2]);
	for(int i = 0; i < GetDim(); ++i)
	    printf("%f ", s[i]);
	printf("\n");
*/	
	
    }
    
    void ContinuousSpace::GetObjectStateWhenAttached(const double s[], const int i, double cfg[]) const
    {
	const double *rs = GetRobotState(s);
	const double *os = GetObjectState(s, i);
	
	Algebra2D::TransAngleMultTransAngleAsTransAngle(rs, os, cfg);
    }


    void ContinuousSpace::UpdateObjectStateWhenPickedUp(double s[], const int i) const
    {
	const double *rs = GetRobotState(s);
	double       *os = GetObjectState(s, i);
	double        cfg[3];
	double        irs[3];
	
	
	Planner::GetInstance()->m_robot.PickupPoint(rs, cfg);
	cfg[2] = os[2];
	
	Algebra2D::InvTransAngleAsTransAngle(rs, irs);
	Algebra2D::TransAngleMultTransAngleAsTransAngle(irs, cfg, os);
    }

    void ContinuousSpace::UpdateObjectStateWhenReleased(double s[], const int i) const
    {
	double cfg[3];
	GetObjectStateWhenAttached(s, i, cfg);
	
	double *os = GetObjectState(s, i);
	os[0] = cfg[0];
	os[1] = cfg[1];
	os[2] = cfg[2];
    }

    void ContinuousSpace::GetObjectTR3(const double s[], const int i, double T[], double R[]) const
    {
	const double *cfg = GetObjectState(s, i);

	T[0] = cfg[0];
	T[1] = cfg[1];
	T[2] = 0;
	Algebra3D::ZAxisAngleAsRot(cfg[2], R);
    }


    void ContinuousSpace::GetObjectTR3WhenAttached(const double s[], const int i, double T[], double R[]) const
    {
	double cfga[3];
	
	GetObjectStateWhenAttached(s, i, cfga);
	T[0] = cfga[0];
	T[1] = cfga[1];
	T[2] = Planner::GetInstance()->m_robot.GetPickupHeight() - PARAM_PROBLEM_HEIGHT_OBJECTS;
	Algebra3D::ZAxisAngleAsRot(cfga[2], R);
    }

    void ContinuousSpace::GetStateFromProblem(double s[]) const
    {
	double *cfg = GetRobotState(s);
	
	cfg[RobotSCar::STATE_X] = Planner::GetInstance()->m_problem.m_initRobotTA[0];
	cfg[RobotSCar::STATE_Y] = Planner::GetInstance()->m_problem.m_initRobotTA[1];
	cfg[RobotSCar::STATE_THETA] = Planner::GetInstance()->m_problem.m_initRobotTA[2];

	const int n = Planner::GetInstance()->m_problem.m_objects.size();
	for(int i = 0; i < n; ++i)
	{
	    cfg = GetObjectState(s, i);
	    cfg[0] = Planner::GetInstance()->m_problem.m_objects[i]->m_cfg[0];
	    cfg[1] = Planner::GetInstance()->m_problem.m_objects[i]->m_cfg[1];
	    cfg[2] = Planner::GetInstance()->m_problem.m_objects[i]->m_cfg[2];
	}
    }

    void ContinuousSpace::MapToDiscreteState(const double cs[], const int attached, int ds[]) const
    {
	double p[2];
	Planner::GetInstance()->m_robot.PickupPoint(GetRobotState(cs), p);
	
	Planner::GetInstance()->m_discreteSpace.SetRobotRoom(ds, Planner::GetInstance()->m_problem.m_grid.GetCellId(p));

	const int n = Planner::GetInstance()->m_problem.m_objects.size();
	for(int i = 0; i < n; ++i)
	{
	    if(i == attached)
		Planner::GetInstance()->m_discreteSpace.SetObjectRoom(ds, i, Planner::GetInstance()->m_discreteSpace.GetRobotRoom(ds));
	    else
	    {
		GetObjectPoint(cs, i, p);
		Planner::GetInstance()->m_discreteSpace.SetObjectRoom(ds, i, Planner::GetInstance()->m_problem.m_grid.GetCellId(p));
	    }
	    
	    //    printf("object %d has point %f %f which is in room %d\n", i, p[0], p[1],
	    //	   Planner::GetInstance()->m_discreteSpace.GetObjectRoom(ds, i));
	}

	Planner::GetInstance()->m_discreteSpace.SetAttachedObject(ds, attached);
	
    }

    void ContinuousSpace::Print(const double cs[], FILE * const out) const
    {
	for(int i = 0; i < GetDim(); ++i)
	    fprintf(out, "%6.4f ", cs[i]);
	fprintf(out, "\n");
    }
    

}


