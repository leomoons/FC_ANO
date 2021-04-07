 /**********************************************************************************************************
 * @文件     ndob.c
 * @说明     外界扰动的非线性扰动观测器算法估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "ndob.h"
#include "Drv_time.h"
#include "mathConfig.h"

#include "controller.h"
#include "mat.h"

nDOB_t _dob;

/**********************************************************************************************************
*函 数 名: ndobForceOutput
*功能说明: 非线性扰动力观测输出
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ndobForceOutput(void)
{
	
	Vector3f_t pv = Matrix3MulVector3(_dob.KoP, _state.vel_fb);
	// Output process
	_dob.F_I = Vector3f_Add(_dob.zP, pv);
	
}

/**********************************************************************************************************
*函 数 名: ndobForceUpdate
*功能说明: 非线性扰动力观测更迭代
*形    参: 步长时间 dT_s
*返 回 值: 无
**********************************************************************************************************/
static void ndobForceUpdate(float dT_s)
{	
	/************定义中间变量***************/
	Vector3f_t v1, v2, v3, v4, v5, v6, v7;
	Vector3f_t pv = Matrix3MulVector3(_dob.KoP, _state.vel_fb);

	// Update process
	v1.x=0.0f; v1.y=0.0f; v1.z=_veh.mass*GRAVITY_ACCEL;	//third element in the brackets
	v2 = Matrix3MulVector3(_state.R_fb, _ctrl.F_b);				//fourth element in the brackets
	
	v3 = Vector3f_Add(_dob.zP, pv);
	v4 = Vector3f_Sub(v3, v1);
	v5 = Vector3f_Add(v4, v2);
	v6.x = v5.x/_veh.mass; v6.y = v5.y/_veh.mass; v6.z = v5.z/_veh.mass; 
	v7 = Matrix3MulVector3(_dob.KoP, v6);
	_dob.zP_dot.x=-v7.x; _dob.zP_dot.y=-v7.y; _dob.zP_dot.z=-v7.z; 
	
	_dob.zP.x = _dob.zP.x+_dob.zP_dot.x * dT_s;
	_dob.zP.y = _dob.zP.y+_dob.zP_dot.y * dT_s; 
	_dob.zP.z = _dob.zP.z+_dob.zP_dot.z * dT_s; 
	

//	_dob.zP_dot.x = ConstrainFloat(_dob.zP_dot.x, -100.0, 100.0);
//	_dob.zP_dot.y = ConstrainFloat(_dob.zP_dot.y, -100.0, 100.0);
//	_dob.zP_dot.z = ConstrainFloat(_dob.zP_dot.z, -100.0, 100.0);
}

/**********************************************************************************************************
*函 数 名: ndobForceOutput
*功能说明: 非线性扰动力矩观测输出
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ndobMomentOutput(void)
{
	Vector3f_t pW = Matrix3MulVector3(_dob.KoR, _state.W_fb);
	// Output process
	_dob.M_b = Vector3f_Add(_dob.zR, pW);
}

/**********************************************************************************************************
*函 数 名: ndobMomentUpdate
*功能说明: 非线性姿态扰动力矩观测更迭代
*形    参: 步长时间 dT_s
*返 回 值: 无
**********************************************************************************************************/
static void ndobMomentUpdate(float dT_s)
{
	/********定义中间变量***********/
	Vector3f_t v1, v2, v3, v4, v5, v6;
	float W_fb_hat[9], m1[9];
	
	Vector3f_t pW = Matrix3MulVector3(_dob.KoR, _state.W_fb);

	
	// Update Process

	Hat_Map(W_fb_hat, _state.W_fb);
	Matrix3_Mul(W_fb_hat, _veh.J, m1);
	v1 = Matrix3MulVector3(m1, _state.W_fb);	//WxJW
		
	v2 = Vector3f_Add(_dob.zR, pW);
	v3 = Vector3f_Sub(v2, v1);
	v4 = Vector3f_Add(v3, _ctrl.M_b);
	
	v5 = Matrix3MulVector3(_veh.J_inv, v4);
	v6 = Matrix3MulVector3(_dob.KoR, v5);
	_dob.zR_dot.x=-v6.x; _dob.zR_dot.y=-v6.y; _dob.zR_dot.z=-v6.z;
	
	_dob.zR.x = _dob.zR.x+_dob.zR_dot.x * dT_s;
	_dob.zR.y = _dob.zR.y+_dob.zR_dot.y * dT_s; 
	_dob.zR.z = _dob.zR.z+_dob.zR_dot.z * dT_s;
	
}

/**********************************************************************************************************
*函 数 名: ndobInit
*功能说明: 非线性扰动观测器初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ndobInit(void)
{
	_dob.KoP[0] = 3.0f; _dob.KoP[1] =     0; _dob.KoP[2] =     0;
	_dob.KoP[3] =    0; _dob.KoP[4] = 3.0f; _dob.KoP[5] =     0;
	_dob.KoP[6] =    0; _dob.KoP[7] =     0; _dob.KoP[8] = 3.0f;
	//P_x1=0.0; P_y1=0.0; P_z1=0.0;
	_dob.zP.x = 0.0; _dob.zP.y = 0.0; _dob.zP.z = 0.0;
	_dob.zP_dot.x = 0.0; _dob.zP_dot.y = 0.0; _dob.zP_dot.z = 0.0;
	
	_dob.KoR[0] = 0.03f;  _dob.KoR[1] =     0; _dob.KoR[2] =     0;
	_dob.KoR[3] =     0; _dob.KoR[4] = 0.03f; _dob.KoR[5] =     0;
	_dob.KoR[6] =     0; _dob.KoR[7] =     0; _dob.KoR[8] = 0.03f;
	//R_x1=0.0; R_y1=0.0; R_z1=0.0;
	_dob.zR.x = 0.0; _dob.zR.y = 0.0; _dob.zR.z = 0.0;
	_dob.zR_dot.x = 0.0; _dob.zR_dot.y = 0.0; _dob.zR_dot.z = 0.0;
}

/**********************************************************************************************************
*函 数 名: ndobOutput
*功能说明: 非线性扰动观测器输出
*形    参: 步长时间
*返 回 值: 无
**********************************************************************************************************/
void ndobOutput(void)
{
	ndobForceOutput();
	ndobMomentOutput();
}

/**********************************************************************************************************
*函 数 名: ndobUpdate
*功能说明: 非线性扰动观测器状态更新
*形    参: 步长时间
*返 回 值: 无
**********************************************************************************************************/
void ndobUpdate(float dT_s)
{	
	ndobForceUpdate(dT_s);
	ndobMomentUpdate(dT_s);
}

/**********************************************************************************************************
*函 数 名: ndobZero
*功能说明: 非线性扰动观测器状态清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ndobZero(void)
{
	_dob.F_I.x = _dob.F_I.y = _dob.F_I.z = 0.0f;
	_dob.M_b.x = _dob.M_b.y = _dob.M_b.z = 0.0f;
	_dob.zP.x = _dob.zP.y = _dob.zP.z = 0.0f;
	_dob.zR.x = _dob.zR.y = _dob.zR.z = 0.0f;
	_dob.zP_dot.x = _dob.zP_dot.y = _dob.zP_dot.z = 0.0f;
	_dob.zR_dot.x = _dob.zR_dot.y = _dob.zR_dot.z = 0.0f;
}
