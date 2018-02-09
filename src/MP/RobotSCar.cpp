#include "MP/RobotSCar.hpp"
#include "MP/Params.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/GMaterial.hpp"

namespace Abetare
{
    RobotSCar::RobotSCar(void)
    {
	m_stateAllocator.SetDim(5);
	m_controlAllocator.SetDim(2);
	m_cfgAllocator.SetDim(3);
	
//min/max values
//50,3,1,1
	m_maxSteerAngle = 70 * Constants::DEG2RAD;
	m_maxVel        = 8; 
	m_maxAcc        = 2;
	m_maxSteerVel   = 2;
	
	m_minSteerAngle = -m_maxSteerAngle;
	m_minVel        = -m_maxVel;
	m_minAcc        = -m_maxAcc;
	m_minSteerVel   = -m_maxSteerVel;
	

//bodies
	GMaterial *gmat;
	
	m_shapeBodyLength     = 2.7;
	m_shapeBodyWidth      = 2.7;
	m_shapeBodyHeight     = 2.5;
	m_shapeWheelCylRadius = 1.0;
	m_shapeWheelCylHeight = 0.4;

	m_shapeAttachLength   = 2.7;
	m_shapeAttachWidth    = 0.3;
	m_shapeAttachHeight   = 0.3;
	m_shapeAttachRadius   = 1.8;
	

//collision mesh
	m_tmeshCollisionWithAttachment.AddBox(-m_shapeWheelCylRadius,
					      -0.5 * m_shapeBodyWidth - m_shapeWheelCylHeight,
					      0,
					      m_shapeBodyLength + m_shapeAttachLength + 2 * m_shapeAttachRadius,
					      0.5 * m_shapeBodyWidth + m_shapeWheelCylHeight,
					      m_shapeWheelCylRadius + 0.5 * m_shapeBodyHeight);

	m_tmeshCollisionNoAttachment.AddBox(-m_shapeWheelCylRadius,
					    -0.5 * m_shapeBodyWidth - m_shapeWheelCylHeight,
					    0,
					    m_shapeBodyLength,// + m_shapeWheelCylRadius,
					    0.5 * m_shapeBodyWidth + m_shapeWheelCylHeight,
					    m_shapeWheelCylRadius + 0.5 * m_shapeBodyHeight);

	
//drawing meshes	
	gmat = new GMaterial();
	gmat->SetPearl();
	m_tmeshDrawBody.SetMainMaterial(gmat);
	m_tmeshDrawBody.AddBox(-m_shapeWheelCylRadius,
			       -0.5 * m_shapeBodyWidth,
			       m_shapeWheelCylRadius - 0.5 * m_shapeBodyHeight,
			       m_shapeBodyLength,
			       0.5 * m_shapeBodyWidth,
			       m_shapeWheelCylRadius + 0.5 * m_shapeBodyHeight);
	m_tmeshDrawBody.AddBox(m_shapeBodyLength,
			       -0.5 * m_shapeAttachWidth,
			       m_shapeWheelCylRadius + 0.5 * m_shapeBodyHeight - m_shapeAttachHeight,
			       m_shapeBodyLength + m_shapeAttachLength,
			       0.5 * m_shapeAttachWidth,
			       m_shapeWheelCylRadius + 0.5 * m_shapeBodyHeight);
	const int nv = m_tmeshDrawBody.GetNrVertices();
	m_tmeshDrawBody.AddCylinderZ(m_shapeAttachRadius,
			       m_shapeWheelCylRadius + 0.5 * m_shapeBodyHeight - m_shapeAttachHeight,
			       m_shapeWheelCylRadius + 0.5 * m_shapeBodyHeight);
				     
	m_tmeshDrawBody.ApplyTrans(nv, m_tmeshDrawBody.GetNrVertices() - 1,
				   m_shapeBodyLength + m_shapeAttachLength + m_shapeAttachRadius,
				   0, 0);
	

	gmat = new GMaterial();
	gmat->SetRuby();
	m_tmeshDrawWheel.SetMainMaterial(gmat);
	m_tmeshDrawWheel.AddCylinderY(m_shapeWheelCylRadius,
				      -0.5 * m_shapeWheelCylHeight,
				       0.5 * m_shapeWheelCylHeight);
	m_tmeshDrawWheel.ApplyTrans(0, m_tmeshDrawWheel.GetNrVertices() - 1,
				    0, 0, m_shapeWheelCylRadius);


    }

    bool RobotSCar::IsStateWithinBounds(const double s[]) const 
    {
	
	if(s[STATE_VEL] < m_minVel || s[STATE_VEL] > m_maxVel)
	    return false;
	const double sa = Algebra2D::AngleNormalize(s[STATE_STEER_ANGLE], -M_PI);
	
//	printf("sa = %f\n", Constants::RAD2DEG * sa);
	
	return sa >= m_minSteerAngle && sa <= m_maxSteerAngle;
    }

    void RobotSCar::ClampState(double s[]) const
    {
	if(s[STATE_VEL] < m_minVel)
	    s[STATE_VEL] = m_minVel;
	else if(s[STATE_VEL] > m_maxVel)
	    s[STATE_VEL] = m_maxVel;
	
	const double sa = Algebra2D::AngleNormalize(s[STATE_STEER_ANGLE], -M_PI);

	if(sa < m_minSteerAngle)
	    s[STATE_STEER_ANGLE] = m_minSteerAngle;
	else if(sa > m_maxSteerAngle)
	    s[STATE_STEER_ANGLE] = m_maxSteerAngle;
    }
    


    void RobotSCar::MotionEqs(const double s[], const double t, const double u[], double ds[])
    {
	const double steer    = s[STATE_STEER_ANGLE];
	const double cpsi     = cos(steer);
	ds[STATE_X]           = s[STATE_VEL] * cos(s[STATE_THETA]) * cpsi;
	ds[STATE_Y]           = s[STATE_VEL] * sin(s[STATE_THETA]) * cpsi;
	ds[STATE_THETA]       = s[STATE_VEL] * sin(steer) / m_shapeBodyLength;
	ds[STATE_VEL]         = u[CONTROL_ACC];
	ds[STATE_STEER_ANGLE] = u[CONTROL_STEER_VEL];	    
    }

    double RobotSCar::GetPickupHeight(void) const
    {
	return  m_shapeWheelCylRadius + 0.5 * m_shapeBodyHeight - m_shapeAttachHeight;
    }

    bool RobotSCar::CanBePickedUp(const double s[], const double p[]) const
    {
	double pp[2];
	PickupPoint(s, pp);
	return Algebra2D::PointDistSquared(p, pp) <= m_shapeAttachRadius * m_shapeAttachRadius;
    }
    
    void RobotSCar::PickupPoint(const double s[], double p[]) const
    {
	p[0] = m_shapeBodyLength + m_shapeAttachLength + m_shapeAttachRadius;
	p[1] = 0;
	Algebra2D::AngleMultPoint(s[STATE_THETA], p, p);
	p[0] += s[STATE_X];
	p[1] += s[STATE_Y];
    }

    void RobotSCar::SampleControl(double u[]) const
    {
	u[CONTROL_ACC] = RandomUniformReal(m_minAcc, m_maxAcc);
	u[CONTROL_STEER_VEL] = RandomUniformReal(m_minVel, m_maxVel);
    }

    bool RobotSCar::ReachedPosition(const double s[], const double target[]) const
    {
	double p[2];
	
	PickupPoint(s, p);
	return Algebra2D::PointDistSquared(p, target) <= m_shapeAttachRadius * m_shapeAttachRadius;
    }
    

    void RobotSCar::StartSteerToPosition(const double target[])
    {
	m_pidSteer.Reset();
	m_pidVel.Reset();

	m_pidSteer.SetDesiredValue(0);
	m_pidVel.SetDesiredValue(RandomUniformReal(0.75, 1.0) * m_maxVel);
    }
    
    void RobotSCar::SteerToPosition(const double s[], const double target[], double ctrlu[])
    {	
	double p[2];

	PickupPoint(s, p);
	
	const double u[2] = {target[0] - p[0], target[1] - p[1]};
	const double v[2] = {cos(s[STATE_THETA] + s[STATE_STEER_ANGLE]), 
			     sin(s[STATE_THETA] + s[STATE_STEER_ANGLE])};

	ctrlu[CONTROL_STEER_VEL] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v), PARAM_ROBOT_DT);
	if(ctrlu[CONTROL_STEER_VEL] > m_maxSteerVel)
	    ctrlu[CONTROL_STEER_VEL] = m_maxSteerVel;
	else if(ctrlu[CONTROL_STEER_VEL] < m_minSteerVel)
	    ctrlu[CONTROL_STEER_VEL] = m_minSteerVel;

	ctrlu[CONTROL_ACC] = m_pidVel.Update(s[STATE_VEL], PARAM_ROBOT_DT);
	if(ctrlu[CONTROL_ACC] > m_maxAcc)
	    ctrlu[CONTROL_ACC] = m_maxAcc;
	else if(ctrlu[CONTROL_ACC] < m_minAcc)
	    ctrlu[CONTROL_ACC] = m_minAcc;
    }
    
    void RobotSCar::Draw(const double s[])
    {
	double T[Algebra3D::Trans_NR_ENTRIES];
	double R[Algebra3D::Rot_NR_ENTRIES];
	double Rs[Algebra3D::Rot_NR_ENTRIES];
	
	T[0] = s[STATE_X];
	T[1] = s[STATE_Y];
	T[2] = 0;
	Algebra3D::ZAxisAngleAsRot(s[STATE_THETA], R);
	Algebra3D::ZAxisAngleAsRot(s[STATE_STEER_ANGLE], Rs);
	 
	GDrawPushTransformation();
	GDrawMultTransRot(T, R);
	m_tmeshDrawBody.Draw();
	
//back left
	GDrawPushTransformation();
	GDrawMultTrans(0, -0.5 * m_shapeBodyWidth - 0.5 * m_shapeWheelCylHeight, 0);
	m_tmeshDrawWheel.Draw();
	GDrawPopTransformation();
	
//back right
	GDrawPushTransformation();
	GDrawMultTrans(0, 0.5 * m_shapeBodyWidth + 0.5 * m_shapeWheelCylHeight, 0);
	m_tmeshDrawWheel.Draw();
	GDrawPopTransformation();

//front left
	GDrawPushTransformation();
	GDrawMultTrans(m_shapeBodyLength, -0.5 * m_shapeBodyWidth - 0.5 * m_shapeWheelCylHeight, 0);
	GDrawMultRot(Rs);
	m_tmeshDrawWheel.Draw();
	GDrawPopTransformation();

//front right
	GDrawPushTransformation();
	GDrawMultTrans(m_shapeBodyLength, 0.5 * m_shapeBodyWidth + 0.5 * m_shapeWheelCylHeight, 0);
	GDrawMultRot(Rs);
	m_tmeshDrawWheel.Draw();
	GDrawPopTransformation();


	GDrawPopTransformation();
    }
    
}


