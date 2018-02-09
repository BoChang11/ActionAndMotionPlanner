#include "MP/Robot.hpp"
#include "MP/Params.hpp"
#include "Utils/Constants.hpp"

namespace Abetare
{
    void Robot::SimulateOneStep(const double s[],
				const double u[], 
				double snew[])
    {
	const double d  = RandomUniformReal(PARAM_ROBOT_MIN_DIST_ONE_STEP, PARAM_ROBOT_MAX_DIST_ONE_STEP);
        double       dt = fabs(TimeStepConstantAcceleration(GetVelocity(s, u),
							    GetAcceleration(s, u),
							    d));

	if(dt > PARAM_ROBOT_DT)
	    dt = PARAM_ROBOT_DT;

	const double  hhalf    = 0.5 * dt;
	const double  hthird   = dt / 3.0;
	const double  hsixth   = dt / 6.0;
	const int     dimState = m_stateAllocator.GetDim();

	std::vector<double> waux;
	waux.resize(2 * dimState);
			
	double *wa   = &waux[0];
	double *wb   = &waux[dimState];
		
	MotionEqs(s, 0, u, wa); 
	for(int i = 0; i < dimState; ++i)
	{
	    snew[i] = s[i] + hsixth * wa[i];
	    wa[i]   = s[i] + hhalf  * wa[i];	
	}
	MotionEqs(wa, hhalf, u, wb);
	for(int i = 0; i < dimState; ++i)
	{
	    snew[i] += hthird * wb[i];
	    wb[i]    = s[i] + hhalf * wb[i];	
	}
	MotionEqs(wb, hhalf, u, wa);
	for(int i = 0; i < dimState; ++i)
	{
	    snew[i] += hthird * wa[i];
	    wa[i]    = s[i] + dt * wa[i];	
	}
	MotionEqs(wa, dt, u, wb);
	for(int i = 0; i < dimState; ++i)
	    snew[i] += hsixth * wb[i];    

	ClampState(snew);
    }   

    	
    double Robot::TimeStepConstantVelocity(const double v, const double d) const
    {
	const double absv = fabs(v);
	
	return absv < Constants::EPSILON ? (d / Constants::EPSILON) : (d / absv);	    
    }
    
    double Robot::TimeStepConstantAcceleration(const double v, 
					       const double a, 
					       const double d) const
    {
	if(fabs(a) < Constants::EPSILON)
	    return TimeStepConstantVelocity(v, d);
	
	if(a >= 0.0 && v >= 0.0)
	    return fabs((-v + sqrt(v * v + 2 * a * d)) / a);
	else if(a <= 0.0 && v <= 0.0)
	    return fabs((v + sqrt(fabs(v * v - 2 * a * d))) / (-a));    
	else if(a <= 0.0 && v >= 0.0)
	{
	    const double b2_4ac = v * v + 2 * a * d;
	    if(b2_4ac >= 0.0)
		return fabs((-v + sqrt(b2_4ac)) / a);
	    const double dist_remaining = d - v * v / (2*a);
	    //traveled from speed zero in reverse
	    return sqrt(2 * dist_remaining / fabs(a));
	}
	else if(a >= 0 && v <= 0.0)
	{
	    const double b2_4ac = v * v - 2 * a * d;
	    if(b2_4ac >= 0.0)
		return fabs((-v - sqrt(b2_4ac)) / a);	
	    const double dist_remaining = d + v * v / (2 * a);
	    return sqrt(2 * dist_remaining / a);	
	}
	
	return Constants::EPSILON;    
    }   

}


