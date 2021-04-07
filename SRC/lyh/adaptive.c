/**********************************************************************************************************
 * @文件     adaptive.c
 * @说明     外界扰动的自适应算法估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.11
**********************************************************************************************************/
#include "adaptive.h"
#include "Drv_time.h"

#include "controller.h"

adaptive_t _ada;


/**********************************************************************************************************
*函 数 名: adaptiveInit
*功能说明: 扰动的自适应算法估计初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void adaptiveInit(void)
{
	_ada.gamax[0]=3.0f; _ada.gamax[1]=0.0f; _ada.gamax[2]=0.0f;
	_ada.gamax[3]=0.0f; _ada.gamax[4]=3.0f; _ada.gamax[5]=0.0f;
	_ada.gamax[6]=0.0f; _ada.gamax[7]=0.0f; _ada.gamax[8]=3.0f;
	
	_ada.gamaR[0]=0.7f; _ada.gamaR[1]=0.0f; _ada.gamaR[2]=0.0f;
	_ada.gamaR[3]=0.0f; _ada.gamaR[4]=0.7f; _ada.gamaR[5]=0.0f;
	_ada.gamaR[6]=0.0f; _ada.gamaR[7]=0.0f; _ada.gamaR[8]=0.7f;
	
	_ada.Wx[0]=1.0f; _ada.Wx[1]=0.0f; _ada.Wx[2]=0.0f;
	_ada.Wx[3]=0.0f; _ada.Wx[4]=1.0f; _ada.Wx[5]=0.0f;
	_ada.Wx[6]=0.0f; _ada.Wx[7]=0.0f; _ada.Wx[8]=1.0f;
	
	_ada.Wx_T[0]=1.0f; _ada.Wx_T[1]=0.0f; _ada.Wx_T[2]=0.0f;
	_ada.Wx_T[3]=0.0f; _ada.Wx_T[4]=1.0f; _ada.Wx_T[5]=0.0f;
	_ada.Wx_T[6]=0.0f; _ada.Wx_T[7]=0.0f; _ada.Wx_T[8]=1.0f;
	
	_ada.WR[0]=1.0f; _ada.WR[1]=0.0f; _ada.WR[2]=0.0f;
	_ada.WR[3]=0.0f; _ada.WR[4]=1.0f; _ada.WR[5]=0.0f;
	_ada.WR[6]=0.0f; _ada.WR[7]=0.0f; _ada.WR[8]=1.0f;
	
	_ada.WR_T[0]=1.0f; _ada.WR_T[1]=0.0f; _ada.WR_T[2]=0.0f;
	_ada.WR_T[3]=0.0f; _ada.WR_T[4]=1.0f; _ada.WR_T[5]=0.0f;
	_ada.WR_T[6]=0.0f; _ada.WR_T[7]=0.0f; _ada.WR_T[8]=1.0f;
	
	_ada.cax[0]=1.0f; _ada.cax[1]=0.0f; _ada.cax[2]=0.0f;
	_ada.cax[3]=0.0f; _ada.cax[4]=1.0f; _ada.cax[5]=0.0f;
	_ada.cax[6]=0.0f; _ada.cax[7]=0.0f; _ada.cax[8]=1.0f;
	
	_ada.caR[0]=1.0f; _ada.caR[1]=0.0f; _ada.caR[2]=0.0f;
	_ada.caR[3]=0.0f; _ada.caR[4]=1.0f; _ada.caR[5]=0.0f;
	_ada.caR[6]=0.0f; _ada.caR[7]=0.0f; _ada.caR[8]=1.0f;
	
	_ada.Bx = 2.0f;
	_ada.BR = 5.0f;
	
	_ada.dx.x = 0.0f; _ada.dx.y = 0.0f; _ada.dx.z = 0.0f;
	_ada.dx_dot.x = 0.0f; _ada.dx_dot.y = 0.0f; _ada.dx_dot.z = 0.0f;
	_ada.F_I.x = 0.0f; _ada.F_I.y = 0.0f; _ada.F_I.z = 0.0f; 
	
	_ada.dR.x = 0.0f; _ada.dR.y = 0.0f; _ada.dR.z = 0.0f;
	_ada.dR_dot.x = 0.0f; _ada.dR_dot.y = 0.0f; _ada.dR_dot.z = 0.0f;
	_ada.M_b.x = 0.0f; _ada.M_b.y = 0.0f; _ada.M_b.z = 0.0f;
}

/**********************************************************************************************************
*函 数 名: adaptiveForceOutput
*功能说明: 自适应算法 扰动力观测输出
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void adaptiveForceOutput(void)
{
	_ada.F_I = Matrix3MulVector3(_ada.Wx, _ada.dx);
}

/**********************************************************************************************************
*函 数 名: adaptiveForceUpdate
*功能说明: 自适应算法 扰动力观测更新迭代
*形    参: 步长时间
*返 回 值: 无
**********************************************************************************************************/
static void adaptiveForceUpdate(float dT_s)
{
	float R_fb_T[9];
	Matrix3_Tran(_state.R_fb, R_fb_T);
	

	
	/*********定义中间变量**********/
	Vector3f_t v1, v2, v3;
	
	v1 = Matrix3MulVector3(_ada.cax, _state.pos_err);
	v2 = Vector3f_Add(_state.vel_err, v1);
	
	v3 = Matrix3MulVector3(_ada.Wx_T, v2);
	_ada.dx_dot = Matrix3MulVector3(_ada.gamax, v3);
	
	//integration
	_ada.dx.x+=(_ada.dx_dot.x*dT_s); _ada.dx.y+=(_ada.dx_dot.y*dT_s); _ada.dx.z+=(_ada.dx_dot.z*dT_s);
}

/**********************************************************************************************************
*函 数 名: adaptiveMomentOutput
*功能说明: 自适应算法 扰动力矩观测输出
*形    参: 步长时间
*返 回 值: 无
**********************************************************************************************************/
static void adaptiveMomentOutput(void)
{
	_ada.M_b = Matrix3MulVector3(_ada.WR, _ada.dR);
}

/**********************************************************************************************************
*函 数 名: adaptiveMomentUpdate
*功能说明: 自适应算法 扰动力矩观测更新迭代
*形    参: 步长时间
*返 回 值: 无
**********************************************************************************************************/
static void adaptiveMomentUpdate(float dT_s)
{
	
	/*********定义中间变量**********/
	Vector3f_t v1, v2, v3;
	
	v1 = Matrix3MulVector3(_ada.caR, _state.R_err);
	v2 = Vector3f_Add(_state.W_err, v1);
	
	v3 = Matrix3MulVector3(_ada.WR_T, v2);
	_ada.dR_dot = Matrix3MulVector3(_ada.gamaR, v3);
	_ada.dR.x+=(_ada.dR_dot.x*dT_s); _ada.dR.y+=(_ada.dR_dot.y*dT_s); _ada.dR.z+=(_ada.dR_dot.z*dT_s);

}

/**********************************************************************************************************
*函 数 名: adaptiveOutput
*功能说明: 扰动的自适应算法估计输出
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void adaptiveOutput()
{
	adaptiveForceOutput();
	adaptiveMomentOutput();
}

/**********************************************************************************************************
*函 数 名: adaptiveUpdate
*功能说明: 扰动的自适应算法估计更新迭代
*形    参: 步长时间
*返 回 值: 无
**********************************************************************************************************/
void adaptiveUpdate(float dT_s)
{
	adaptiveForceUpdate(dT_s);
	adaptiveMomentUpdate(dT_s);
	
}

/**********************************************************************************************************
*函 数 名: adaptiveZero
*功能说明: 自适应算法状态清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void adaptiveZero(void)
{
	_ada.F_I.x = _ada.F_I.y = _ada.F_I.z = 0.0f;
	_ada.M_b.x = _ada.M_b.y = _ada.M_b.z = 0.0f;
	_ada.dx.x = _ada.dx.y = _ada.dx.z = 0.0f;
	_ada.dx_dot.x = _ada.dx_dot.y = _ada.dx_dot.z = 0.0f;
	_ada.dR.x = _ada.dR.y = _ada.dR.z = 0.0f;
	_ada.dR_dot.x = _ada.dR_dot.y = _ada.dR_dot.z = 0.0f;
}



