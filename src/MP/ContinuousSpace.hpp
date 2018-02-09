#ifndef ABETARE__CONTINUOUS_SPACE_HPP_
#define ABETARE__CONTINUOUS_SPACE_HPP_

#include "Utils/Allocator.hpp"
#include <cstdio>

namespace Abetare
{
    class ContinuousSpace : public Allocator<double>
    {
    public:
	ContinuousSpace(void) : Allocator<double>()
	{
	}
	
	virtual ~ContinuousSpace(void)
	{
	}

	virtual void CompleteSetup(void);
	
	double* GetRobotState(double s[]) const
	{
	    return s;
	}
	
	const double* GetRobotState(const double s[]) const
	{
	    return s;
	}
	
	double* GetObjectState(double s[], const int i) const;
	
	const double* GetObjectState(const double s[], const int i) const;

	void GetObjectPoint(const double s[], const int i, double p[]) const;
	
	void GetObjectStateWhenAttached(const double s[], const int i, double cfg[]) const;

	void UpdateObjectStateWhenPickedUp(double s[], const int i) const;

	void UpdateObjectStateWhenReleased(double s[], const int i) const;

	void GetObjectTR3(const double s[], const int i, double T[], double R[]) const;
	
	void GetObjectTR3WhenAttached(const double s[], const int i, double T[], double R[]) const;

	void GetStateFromProblem(double s[]) const;

	void MapToDiscreteState(const double cs[], const int attached, int ds[]) const;

	void Print(const double cs[], FILE * const out) const;
	
    };    
}

#endif


