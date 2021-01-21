/**********************************************************************************************************
 * @�ļ�     setPoint.c
 * @˵��     �������켣����ֵ����
 * @�汾  	 V1.0
 * @����     Leomoon
 * @����     2020.10
**********************************************************************************************************/
#include "setPoint.h"
#include "Drv_time.h"
#include "Ano_RC.h"

#include "flightMode.h"

setpoint_t _set;

/**********************************************************************************************************
*�� �� ��: PosUpdate
*����˵��: λ������ֵ����
*��    ��: ʱ�������� �켣���ɵ�ǰʱ��
*�� �� ֵ: void
**********************************************************************************************************/
void PosUpdate(float dT_s, float T)
{
	float v = 0.0;
	float a = 0.0;
	
	_set.pos.x = v*T; _set.pos.y = 1.0f*sin(a*T);      _set.pos.z = 1.5f*cos(a*T);
	_set.vel.x = v;   _set.vel.y = 1.0f*a*cos(a*T);    _set.vel.z =-1.5f*a*sin(a*T);
	_set.acc.x = 0;   _set.acc.y =-1.0f*a*a*sin(a*T);  _set.acc.x =-1.5f*a*a*cos(a*T);
}

/**********************************************************************************************************
*�� �� ��: IntegrateIteration
*����˵��: ����̬���л��ֵ���
*��    ��: ʱ������
*�� �� ֵ: void
**********************************************************************************************************/
static void IntegrateIteration(float dT_s)
{
	static float R_former[9] = {1,0,0, 0,1,0, 0,0,1};
	static Vector3f_t W_former;
	float R_dot[9];
	
	_set.ang_vel.x = dT_s*_set.ang_acc.x + W_former.x;
	_set.ang_vel.y = dT_s*_set.ang_acc.y + W_former.y;
	_set.ang_vel.z = dT_s*_set.ang_acc.z + W_former.z;
	
	R_dot[0] = R_former[1]*_set.ang_vel.z 		  + R_former[2]*_set.ang_vel.y*(-1.0f);
	R_dot[1] = R_former[0]*_set.ang_vel.z*(-1.0f) + R_former[2]*_set.ang_vel.x        ;
	R_dot[2] = R_former[0]*_set.ang_vel.y 	      + R_former[1]*_set.ang_vel.x*(-1.0f);
	R_dot[3] = R_former[4]*_set.ang_vel.z 		  + R_former[5]*_set.ang_vel.y*(-1.0f);
	R_dot[4] = R_former[3]*_set.ang_vel.z*(-1.0f) + R_former[5]*_set.ang_vel.x        ;
	R_dot[5] = R_former[3]*_set.ang_vel.y 		  + R_former[4]*_set.ang_vel.x*(-1.0f);
	R_dot[6] = R_former[7]*_set.ang_vel.z 		  + R_former[8]*_set.ang_vel.y*(-1.0f);
	R_dot[7] = R_former[6]*_set.ang_vel.z*(-1.0f) + R_former[8]*_set.ang_vel.x        ;
	R_dot[8] = R_former[6]*_set.ang_vel.y 		  + R_former[7]*_set.ang_vel.x*(-1.0f); 
	
	_set.att[0]=dT_s*R_dot[0]+R_former[0]; _set.att[1]=dT_s*R_dot[1]+R_former[1]; _set.att[2]=dT_s*R_dot[2]+R_former[2];
	_set.att[3]=dT_s*R_dot[3]+R_former[3]; _set.att[4]=dT_s*R_dot[4]+R_former[4]; _set.att[5]=dT_s*R_dot[5]+R_former[5];
	_set.att[6]=dT_s*R_dot[6]+R_former[6]; _set.att[7]=dT_s*R_dot[7]+R_former[7]; _set.att[8]=dT_s*R_dot[8]+R_former[8];
	
	
	R_former[0]=_set.att[0]; R_former[1] = _set.att[1]; R_former[2] = _set.att[2];
	R_former[3]=_set.att[3]; R_former[4] = _set.att[4]; R_former[5] = _set.att[5];
	R_former[6]=_set.att[6]; R_former[7] = _set.att[7]; R_former[8] = _set.att[8];
	
	W_former.x = _set.ang_vel.x; W_former.y = _set.ang_vel.y; W_former.z = _set.ang_vel.z;
}

/**********************************************************************************************************
*�� �� ��: DifferentialIteration
*����˵��: ����̬����΢�ֵ���
*��    ��: ʱ������
*�� �� ֵ: void
**********************************************************************************************************/
static void DifferentialIteration(float dT_s)
{
	float R_dot[9], R_T[9], W_hat[9];
	static float R_former[9] = {1,0,0, 0,1,0, 0,0,1};
	
	Matrix3_Sub(_set.att, R_former, R_dot);
	float tmp = safe_div(1.0f, dT_s, 0.0f);
	Scalar_Matrix3(tmp, R_dot);
	Matrix3_Tran(_set.att, R_T);
	Matrix3_Mul(R_T, R_dot, W_hat);
	
	R_former[0]=_set.att[0]; R_former[1] = _set.att[1]; R_former[2] = _set.att[2];
	R_former[3]=_set.att[3]; R_former[4] = _set.att[4]; R_former[5] = _set.att[5];
	R_former[6]=_set.att[6]; R_former[7] = _set.att[7]; R_former[8] = _set.att[8];
	
	_set.ang_vel.x = W_hat[7];
	_set.ang_vel.y = W_hat[2];
	_set.ang_vel.z = W_hat[3];
}

/**********************************************************************************************************
*�� �� ��: AttUpdate
*����˵��: ��̬����ֵ����
*��    ��: ʱ�������� �켣���ɵ�ǰʱ��
*�� �� ֵ: void
**********************************************************************************************************/
void AttUpdate(float dT_s, float T)
{
	/*******��̬�Ļ��ֵ�������********/
//	if(T>=0 && T<=10)
//	{
//		_set.ang_acc.x = 3.14159/10;
//		_set.ang_acc.y = 0.0;
//		_set.ang_acc.z = 0.0;
//	}
//	else
//	{
//		_set.ang_acc.x = 0.0;
//		_set.ang_acc.y = 0.0;
//		_set.ang_acc.z = 0.0;
//	}
//	IntegrateIteration(dT_s);
	
	/*******��̬��΢�ֵ�������********/
	float pit = M_PI/10*sinf(M_PI/30*T);
	float rol = M_PI/10*cosf(M_PI/30*T);
	float yaw = 0.0f;
	
	_set.att[0] = cosf(pit)*cosf(yaw);
  _set.att[1] = cosf(pit)*sinf(yaw);
  _set.att[2] = -sinf(pit);
  _set.att[3] = sinf(rol)*sinf(pit)*cosf(yaw) - cosf(rol)*sinf(yaw);
  _set.att[4] = sinf(rol)*sinf(pit)*sinf(yaw) + cosf(rol)*cosf(yaw);
  _set.att[5] = sinf(rol)*cosf(pit);
  _set.att[6] = cosf(rol)*sinf(pit)*cosf(yaw) + sinf(rol)*sinf(yaw);
  _set.att[7] = cosf(rol)*sinf(pit)*sinf(yaw) - sinf(rol)*cosf(yaw);
  _set.att[8] = cosf(rol)*cosf(pit);
	
	DifferentialIteration(dT_s);
	
}


/**********************************************************************************************************
*�� �� ��: SetPointUpdate
*����˵��: �켣����ֵ����
*��    ��: void
*�� �� ֵ: void
**********************************************************************************************************/
void SetPointUpdate(void)
{
	static uint64_t T_now=0, T_former=0, T_start=0;
	float dT_s;
	float T;
	
	if(GetFlightMode() == MISSION)		
	{
		T_now = GetSysTime_us();
		T = (T_now-T_start)*1e-6f;
		dT_s = (T_now - T_former)*1e-6f;
		PosUpdate(dT_s, T);
		AttUpdate(dT_s, T);
		T_former = GetSysTime_us();
	}
	else
	{
		T_start = GetSysTime_us();
		T_former = T_start;
		
		_set.pos.x=0.0f; _set.pos.y=0.0f; _set.pos.z=0.0f;
		_set.vel.x=0.0f; _set.vel.y=0.0f; _set.vel.z=0.0f;
		_set.acc.x=0.0f; _set.acc.y=0.0f; _set.acc.z=0.0f;
		
		_set.att[0]=1; _set.att[1]=0; _set.att[2]=0;
		_set.att[3]=0; _set.att[4]=1; _set.att[5]=0;
		_set.att[6]=0; _set.att[7]=0; _set.att[8]=1;
		_set.ang_vel.x=0.0f; _set.ang_vel.y=0.0f; _set.ang_vel.z=0.0f;
		_set.ang_acc.x=0.0f; _set.ang_acc.y=0.0f; _set.ang_acc.z=0.0f;
	}
}



/**********************************************************************************************************
*�� �� ��: GetDesiredPos
*����˵��: ��ȡ�����켣�е�λ����Ϣ
*��    ��: λ������ָ��
*�� �� ֵ: void
**********************************************************************************************************/
void GetDesiredPos(Vector3f_t *pos)
{
	*pos = _set.pos;
}

/**********************************************************************************************************
*�� �� ��: GetDesiredVel
*����˵��: ��ȡ�����켣�е��ٶ���Ϣ
*��    ��: �ٶ�����ָ��
*�� �� ֵ: void
**********************************************************************************************************/
void GetDesiredVel(Vector3f_t *vel)
{
	*vel = _set.vel;
}

/**********************************************************************************************************
*�� �� ��: GetDesiredAcc
*����˵��: ��ȡ�����켣�еļ��ٶ���Ϣ
*��    ��: ���ٶ�����ָ��
*�� �� ֵ: void
**********************************************************************************************************/
void GetDesiredAcc(Vector3f_t *acc)
{
	*acc = _set.acc;
}

/**********************************************************************************************************
*�� �� ��: GetDesiredAtt
*����˵��: ��ȡ�����켣�е���̬��Ϣ
*��    ��: �������Ҿ�������ͷ
*�� �� ֵ: void
**********************************************************************************************************/
void GetDesiredAtt(float *dcm)
{
	Matrix3_Copy(_set.att, dcm);
}

/**********************************************************************************************************
*�� �� ��: GetDesiredAngVel
*����˵��: ��ȡ�����켣�еĽ��ٶ���Ϣ
*��    ��: ���ٶ�����ָ��
*�� �� ֵ: void
**********************************************************************************************************/
void GetDesiredAngVel(Vector3f_t *ang_vel)
{
	*ang_vel = _set.ang_vel;
}

/**********************************************************************************************************
*�� �� ��: GetDesiredAngAcc
*����˵��: ��ȡ�����켣�еĽǼ��ٶ���Ϣ
*��    ��: v�Ǽ��ٶ�����ָ��
*�� �� ֵ: void
**********************************************************************************************************/
void GetDesiredAngAcc(Vector3f_t *ang_acc)
{
	*ang_acc = _set.ang_acc;
}

