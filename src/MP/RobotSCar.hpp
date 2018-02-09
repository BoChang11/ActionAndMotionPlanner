#ifndef ABETARE__ROBOT_SCAR_HPP_
#define ABETARE__ROBOT_SCAR_HPP_

#include "MP/Robot.hpp"
#include "Utils/PIDController.hpp"
#include "Utils/TriMesh.hpp"
#include "External/PQP/PQPTriMesh.hpp"

namespace Abetare
{
    class RobotSCar : public Robot
    {
    public:
	enum
	    {
		STATE_X           = 0,
		STATE_Y           = 1,
		STATE_THETA       = 2,
		STATE_VEL         = 3,
		STATE_STEER_ANGLE = 4,
		CONTROL_ACC       = 0,
		CONTROL_STEER_VEL = 1
	    };
		    
	RobotSCar(void);
			
	virtual ~RobotSCar(void)
	{
	}

	virtual bool IsStateWithinBounds(const double s[]) const;

	virtual void ClampState(double s[]) const;

	virtual void MotionEqs(const double s[], const double t, const double u[], double ds[]);
		
	virtual double GetVelocity(const double s[], const double u[])
	{
	    return s[STATE_VEL];
	    
	}
	
	virtual double GetAcceleration(const double s[], const double u[])
	{
	    return u[CONTROL_ACC];
	}
	
	virtual double GetPickupHeight(void) const;

	virtual bool CanBePickedUp(const double s[], const double p[]) const;
	
	virtual void PickupPoint(const double s[], double p[]) const;
	
	virtual void StartSteerToPosition(const double target[]);
	
	virtual void SteerToPosition(const double s[], const double target[], double u[]);

	virtual bool ReachedPosition(const double s[], const double target[]) const;
	
	virtual void SampleControl(double u[]) const;

	virtual void Draw(const double s[]);
	
	double m_shapeBodyLength;
	double m_shapeBodyWidth;
	double m_shapeBodyHeight;
	double m_shapeWheelCylRadius;
	double m_shapeWheelCylHeight;
	double m_shapeAttachLength;
	double m_shapeAttachWidth;
	double m_shapeAttachHeight;
	double m_shapeAttachRadius;
		
	TriMesh    m_tmeshDrawBody;
	TriMesh    m_tmeshDrawWheel;
	PQPTriMesh m_tmeshCollisionWithAttachment;
	PQPTriMesh m_tmeshCollisionNoAttachment;


	double m_minSteerAngle;
	double m_maxSteerAngle;
	double m_minVel;
	double m_maxVel;
	double m_minAcc;
	double m_maxAcc;
	double m_minSteerVel;
	double m_maxSteerVel;

	PIDController m_pidSteer;
	PIDController m_pidVel;
    };    
}

#endif


