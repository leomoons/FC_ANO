/**********************************************************************************************************
 * @�ļ�    smc.c
 * @˵��    ��ģ��ʽ���λ�ģ������
 * @�汾  	V1.0
 * @����   	Leomoon
 * @����    2020.12
**********************************************************************************************************/
#include "smc.h"
#include "param.h" 

#include "controller.h"
#include "pd.h"

smc_ctrl_t _smc;
float delta[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

/**********************************************************************************************************
*�� �� ��: smcCtrlInit
*����˵��: SMC��ʽ�ļ��ο�����������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void smcCtrlInit(void)
{
	_smc.csP[1]= _smc.csP[2]= _smc.csP[3]= _smc.csP[5]= _smc.csP[6]= _smc.csP[7]= 0; 
	ParamGetData(CONTROLLER_SMC_csP_X, &(_smc.csP[0]), 4);
	ParamGetData(CONTROLLER_SMC_csP_Y, &(_smc.csP[4]), 4);
	ParamGetData(CONTROLLER_SMC_csP_Z, &(_smc.csP[8]), 4);
	
	_smc.HsP[1]= _smc.HsP[2]= _smc.HsP[3]= _smc.HsP[5]= _smc.HsP[6]= _smc.HsP[7]= 0; 
	ParamGetData(CONTROLLER_SMC_HsP_X, &(_smc.HsP[0]), 4);
	ParamGetData(CONTROLLER_SMC_HsP_Y, &(_smc.HsP[4]), 4);
	ParamGetData(CONTROLLER_SMC_HsP_Z, &(_smc.HsP[8]), 4);
	
	_smc.ksP[1]= _smc.ksP[2]= _smc.ksP[3]= _smc.ksP[5]= _smc.ksP[6]= _smc.ksP[7]= 0; 
	ParamGetData(CONTROLLER_SMC_ksP_X, &(_smc.ksP[0]), 4);
	ParamGetData(CONTROLLER_SMC_ksP_Y, &(_smc.ksP[4]), 4);
	ParamGetData(CONTROLLER_SMC_ksP_Z, &(_smc.ksP[8]), 4);
	
	_smc.csR[1]= _smc.csR[2]= _smc.csR[3]= _smc.csR[5]= _smc.csR[6]= _smc.csR[7]= 0; 
	ParamGetData(CONTROLLER_SMC_csR_X, &(_smc.csR[0]), 4);
	ParamGetData(CONTROLLER_SMC_csR_Y, &(_smc.csR[4]), 4);
	ParamGetData(CONTROLLER_SMC_csR_Z, &(_smc.csR[8]), 4);
	
	_smc.HsR[1]= _smc.HsR[2]= _smc.HsR[3]= _smc.HsR[5]= _smc.HsR[6]= _smc.HsR[7]= 0; 
	ParamGetData(CONTROLLER_SMC_HsR_X, &(_smc.HsR[0]), 4);
	ParamGetData(CONTROLLER_SMC_HsR_Y, &(_smc.HsR[4]), 4);
	ParamGetData(CONTROLLER_SMC_HsR_Z, &(_smc.HsR[8]), 4);
	
	_smc.ksP[1]= _smc.ksR[2]= _smc.ksR[3]= _smc.ksR[5]= _smc.ksR[6]= _smc.ksR[7]= 0; 
	ParamGetData(CONTROLLER_SMC_ksR_X, &(_smc.ksR[0]), 4);
	ParamGetData(CONTROLLER_SMC_ksR_Y, &(_smc.ksR[4]), 4);
	ParamGetData(CONTROLLER_SMC_ksR_Z, &(_smc.ksR[8]), 4);
}

/**********************************************************************************************************
*�� �� ��: smcCtrlUpdateParam
*����˵��: SMC��ʽ�ļ��ο����������޸�
*��    ��: ����ѡ��param, �����޸�ֵdata
*�� �� ֵ: ��
**********************************************************************************************************/
void smcCtrlUpdateParam(uint16_t param, float data)
{
	switch(param)
	{
		case CONTROLLER_SMC_csP_X:
			_smc.csP[0] = data;
		break;
		case CONTROLLER_SMC_csP_Y:
			_smc.csP[4] = data;
		break;
		case CONTROLLER_SMC_csP_Z:
			_smc.csP[8] = data;
		break;
		case CONTROLLER_SMC_HsP_X:
			_smc.HsP[0] = data;
		break;
		case CONTROLLER_SMC_HsP_Y:
			_smc.HsP[4] = data;
		break;
		case CONTROLLER_SMC_HsP_Z:
			_smc.HsP[8] = data;
		break;
		case CONTROLLER_SMC_ksP_X:
			_smc.ksP[0] = data;
		break;
		case CONTROLLER_SMC_ksP_Y:
			_smc.ksP[4] = data;
		break;
		case CONTROLLER_SMC_ksP_Z:
			_smc.ksP[8] = data;
		break;
		
		case CONTROLLER_SMC_csR_X:
			_smc.csR[0] = data;
		break;
		case CONTROLLER_SMC_csR_Y:
			_smc.csR[4] = data;
		break;
		case CONTROLLER_SMC_csR_Z:
			_smc.csR[8] = data;
		break;
		case CONTROLLER_SMC_HsR_X:
			_smc.HsR[0] = data;
		break;
		case CONTROLLER_SMC_HsR_Y:
			_smc.HsR[4] = data;
		break;
		case CONTROLLER_SMC_HsR_Z:
			_smc.HsR[8] = data;
		break;
		case CONTROLLER_SMC_ksR_X:
			_smc.ksR[0] = data;
		break;
		case CONTROLLER_SMC_ksR_Y:
			_smc.ksR[4] = data;
		break;
		case CONTROLLER_SMC_ksR_Z:
			_smc.ksR[8] = data;
		break;
	}
	
	//��flash���޸���Ӧ����ֵ
	ParamUpdateData(param, &data);
}

/**********************************************************************************************************
*�� �� ��: ForceCal
*����˵��: SMC���ο������е�����������
*��    ��: ��
*�� �� ֵ: ���������� 
**********************************************************************************************************/
static Vector3f_t ForceCal(void)
{
	Vector3f_t v1, v2, v3, v4, v5, v6, v7, v8;
	float R_T[9];
	
	/**************����λ�ú��ٶ����****************/
	_state.pos_err = Vector3f_Sub(_state.pos_fb, _state.pos_des);
	_state.vel_err = Vector3f_Sub(_state.vel_fb, _state.vel_des);
	
	/******����sP(λ�������Ļ�ģ��)*******/
	v1 = Matrix3MulVector3(_smc.csP, _state.pos_err);
	_smc.sP  = Vector3f_Add(_state.vel_err, v1);
	
	/************����������еĲ�����(Fa)**********/
	v2.x=0.0f; v2.y=0.0f; v2.z=GRAVITY_ACCEL;	
	v3 = Matrix3MulVector3(_smc.csP, _state.vel_err);
	v4 = Vector3f_Add(v2, _state.acc_des);
	v5 = Vector3f_Sub(v4, v3);
	_smc.Fa.x = _veh.mass * v5.x;
	_smc.Fa.y = _veh.mass * v5.y;
	_smc.Fa.z = _veh.mass * v5.z;
	
	/*******���ɻ�ģ��(satsP)********/
	if(abs(_smc.sP.x) > delta[0])	_smc.satsP.x = sign(_smc.sP.x);
	else	_smc.satsP.x = 1/delta[0]*_smc.sP.x;
	if(abs(_smc.sP.y) > delta[1])	_smc.satsP.y = sign(_smc.sP.y);
	else	_smc.satsP.y = 1/delta[1]*_smc.sP.y;
	if(abs(_smc.sP.z) > delta[2])	_smc.satsP.z = sign(_smc.sP.z);
	else	_smc.satsP.z = 1/delta[2]*_smc.sP.z;
	
	/**********����������еĻ�ģ��(Fs)***************/
	v6 = Matrix3MulVector3(_smc.ksP, _smc.sP);
	v6.x=-v6.x; v6.y=-v6.y; v6.z=-v6.z;
	v7 = Matrix3MulVector3(_smc.HsP, _smc.satsP);
	_smc.Fs = Vector3f_Sub(v6, v7);
	
	/*******��(Fa+Fs)ת������������ϵ��*********/
	v8 = Vector3f_Add(_smc.Fa, _smc.Fs);
	Matrix3_Tran(_state.R_fb, R_T);		// Transpose of attitude matrix
	return Matrix3MulVector3(R_T, v8);
}

/**********************************************************************************************************
*�� �� ��: MomentCal
*����˵��: ���ο������е����ؼ���
*��    ��: ��
*�� �� ֵ: �������� 
**********************************************************************************************************/
static Vector3f_t MomentCal(void)
{
	/***********��������м���***************/
	Vector3f_t v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14;
	float m1[9];
	Vector3f_t alpha_d;
	float R_fb_T[9], W_fb_hat[9];
	float I3[9], E[9];
	Matrix3_Eye(I3);
	
	/*********��̬�ͽ��ٶ��������************/
	Hat_Map(W_fb_hat, _state.W_fb);
	Matrix3_Tran(_state.R_fb, R_fb_T);
	Attitude_Error();
	Angular_Rate_Error();
	
	/*********����sR(��̬�Ļ�ģ��)***********/
	v1 = Matrix3MulVector3(_smc.csR, _state.R_err);
	_smc.sR = Vector3f_Add(_state.W_err, v1);
	
	/*************�����м���alpha_d***********/
	v2 = Matrix3MulVector3(_state.R_des, _state.W_des);	// First element
	v3 = Matrix3MulVector3(R_fb_T, v2);
	v4 = Matrix3MulVector3(W_fb_hat, v3);
	v5 = Matrix3MulVector3(_state.R_des, _state.W_dot_des); // Second element
	v6 = Matrix3MulVector3(R_fb_T, v5);
	alpha_d = Vector3f_Sub(v6, v4);
	
	/********�����м����E***********/
	Matrix3_Mul(R_fb_T, _state.R_des, m1);
	float tmp1 = Matrix3_Trace(m1);
	Scalar_Matrix3(tmp1, I3);
	Matrix3_Sub(I3, m1, E);
	Scalar_Matrix3(0.5f, E);
	
	/********������������еĲ�����Ma******/
	v7 = Matrix3MulVector3(_veh.J, _state.W_fb);	// First element
	v8 = Matrix3MulVector3(W_fb_hat, v7);
	v9 = Matrix3MulVector3(E, _state.W_err);	// Second element
	v10 = Matrix3MulVector3(_smc.csR, v9);
	v11 = Vector3f_Sub(alpha_d, v10);
	v12 = Matrix3MulVector3(_veh.J, v11);
	_smc.Ma = Vector3f_Add(v8, v12);
	
	/**********���ɻ�ģ��satsR******************/
	if(abs(_smc.sR.x) > delta[3])	_smc.satsR.x = sign(_smc.sR.x);
	else	_smc.satsR.x = 1/delta[3]*_smc.sR.x;
	if(abs(_smc.sR.y) > delta[4])	_smc.satsR.y = sign(_smc.sR.y);
	else	_smc.satsR.y = 1/delta[4]*_smc.sR.y;
	if(abs(_smc.sR.z) > delta[5])	_smc.satsR.z = sign(_smc.sR.z);
	else	_smc.satsR.z = 1/delta[5]*_smc.sR.z;

	/**********������������еĻ�ģ��(Ms)***************/
	v13 = Matrix3MulVector3(_smc.ksR, _smc.sR);
	v13.x=-v13.x; v13.y=-v13.y; v13.z=-v13.z;
	v14 = Matrix3MulVector3(_smc.HsR, _smc.satsR);
	_smc.Ms = Vector3f_Sub(v13, v14);
	
	return Vector3f_Add(_smc.Ma, _smc.Ms);
}


/**********************************************************************************************************
*�� �� ��: smcCtrlUpdate
*����˵��: SMC��ʽ�ļ��ο�����������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void smcCtrlUpdate(void)
{
	_ctrl_only.F_b = ForceCal();
	_ctrl_only.M_b = MomentCal();
	
	_ctrl_only.wrench[0] = _ctrl_only.F_b.x;
	_ctrl_only.wrench[1] = _ctrl_only.F_b.y;
	_ctrl_only.wrench[2] = _ctrl_only.F_b.z;
	_ctrl_only.wrench[3] = _ctrl_only.M_b.x;
	_ctrl_only.wrench[4] = _ctrl_only.M_b.y;
	_ctrl_only.wrench[5] = _ctrl_only.M_b.z;
}

