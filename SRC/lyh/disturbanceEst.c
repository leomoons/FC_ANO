/**********************************************************************************************************
 * @文件     disturbanceEst.c
 * @说明     扰动估计的总和
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "disturbanceEst.h"
#include "ndob.h"
#include "adaptive.h"
#include "Drv_time.h"
#include "flightMode.h"
#include "controller.h"

estimate_set_t _est;

enum
{
	NO_ESTIMATOR,
	NDOB,
	ADAPTIVE
};

#define ESTIMATOR NDOB


/**********************************************************************************************************
*函 数 名: estimatorInit
*功能说明: 扰动观测模块初始化初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void estimatorInit(void)
{
	if(ESTIMATOR == NO_ESTIMATOR)
	{
		;
	}
	else if(ESTIMATOR == NDOB)
	{
		ndobInit();
	}
	else if(ESTIMATOR == ADAPTIVE)
	{
		adaptiveInit();
	}
	
}

/**********************************************************************************************************
*函 数 名: estimatorOutput
*功能说明: 扰动观测模块输出
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void estimatorOutput(void)
{
	if(GetFlightMode() == OBSERVER)
	{
		if(ESTIMATOR == NO_ESTIMATOR)
		{
			_est.F_I.x = _est.F_I.y = _est.F_I.z = 0.0f;
			_est.M_b.x = _est.M_b.y = _est.M_b.z = 0.0f;
		}
		else if(ESTIMATOR == NDOB)
		{
			ndobOutput();
			_est.F_I = _dob.F_I;
			_est.M_b = _dob.M_b;
		}
		else if(ESTIMATOR == ADAPTIVE)
		{
			adaptiveOutput();
			_est.F_I = _ada.F_I;
			_est.M_b = _ada.M_b;
		}
	}
	else 
	{
		estimatorZero();
	}
	
	
	// 进行限幅
	_est.F_b.x = ConstrainFloat(_est.F_b.x, -1.5f, 1.5f);
	_est.F_b.y = ConstrainFloat(_est.F_b.y, -1.5f, 1.5f);
	_est.F_b.z = ConstrainFloat(_est.F_b.z, -1.5f, 1.5f);
	_est.M_b.x = ConstrainFloat(_est.M_b.x, -0.2f, 0.2f);
	_est.M_b.y = ConstrainFloat(_est.M_b.y, -0.2f, 0.2f);
	_est.M_b.z = ConstrainFloat(_est.M_b.z, -0.2f, 0.2f);
	
	
	float R_fb_T[9];
	Matrix3_Tran(_state.R_fb, R_fb_T);
	_est.F_b = Matrix3MulVector3(R_fb_T, _est.F_I);
	
}

/**********************************************************************************************************
*函 数 名: estimatorUpdate
*功能说明: 扰动观测模块更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void estimatorUpdate(void)
{
	static uint64_t pT8;
	float dT_s = (GetSysTime_us() - pT8) * 1e-6;
	dT_s = ConstrainFloat(dT_s, 0.0005,0.01);
	pT8 = GetSysTime_us();
	
	//dT_s = 0.005f;
	
	if(GetFlightMode() == OBSERVER)
	{
		if(ESTIMATOR == NO_ESTIMATOR)
		{
			;
		}
		else if(ESTIMATOR == NDOB)
		{
			ndobUpdate(dT_s);
		}
		else if(ESTIMATOR == ADAPTIVE)
		{
			adaptiveUpdate(dT_s);
		}
	}
	else
	{
		estimatorZero();
	}
	
	
	
//	if(ESTIMATOR == NO_ESTIMATOR)
//	{
//		;
//	}
//	else if(ESTIMATOR == NDOB)
//	{
//		ndobUpdate(dT_s);
//	}
//	else if(ESTIMATOR == ADAPTIVE)
//	{
//		adaptiveUpdate(dT_s);
//	}
	
}


/**********************************************************************************************************
*函 数 名: estimatorZero
*功能说明: 扰动观测模块清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void estimatorZero(void)
{
	ndobZero();
	adaptiveZero();
	_est.F_I.x = _est.F_I.y = _est.F_I.z = 0.0f;
	_est.F_b.x = _est.F_b.y = _est.F_b.z = 0.0f;
	_est.M_b.x = _est.M_b.y = _est.M_b.z = 0.0f;
}


