#ifndef ABETARE__ROBOT_HPP_
#define ABETARE__ROBOT_HPP_

#include "Utils/Allocator.hpp"

namespace Abetare
{
    class Robot
    {
    public:
	Robot(void) 
	{
	}
			
	virtual ~Robot(void)
	{
	}

//state utils
	virtual bool IsStateWithinBounds(const double s[]) const = 0;
	virtual void ClampState(double s[]) const = 0;
	
//picked up
	virtual double GetPickupHeight(void) const = 0;
	virtual bool CanBePickedUp(const double s[], const double p[]) const = 0;
	virtual void PickupPoint(const double s[], double p[]) const = 0;	

//steering
	virtual void StartSteerToPosition(const double target[]) = 0;
	virtual void SteerToPosition(const double s[], const double target[], double u[]) = 0;
	virtual bool ReachedPosition(const double s[], const double target[]) const = 0;
	virtual void SampleControl(double u[]) const = 0;
	

//motion equations
	virtual void SimulateOneStep(const double s[], const double u[], double snew[]);
	virtual void MotionEqs(const double s[], const double t, const double u[], double ds[]) = 0;
	virtual double TimeStepConstantVelocity(const double v, const double d) const;
	virtual double TimeStepConstantAcceleration(const double v, const double a, const double d) const;
	virtual double GetVelocity(const double s[], const double u[]) = 0;
	virtual double GetAcceleration(const double s[], const double u[]) = 0;
	
//drawing
	virtual void Draw(const double s[]) = 0;

//allocators
	Allocator<double> m_stateAllocator;
	Allocator<double> m_controlAllocator;
	Allocator<double> m_cfgAllocator;
    };    
}

#endif


