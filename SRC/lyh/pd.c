/**********************************************************************************************************
 * @�ļ�     disturbanceEst.c
 * @˵��     �Ŷ����Ƶ��ܺ�
 * @�汾  	 V1.0
 * @����     Leomoon
 * @����     2020.10
**********************************************************************************************************/
#include "pd.h"
#include "param.h"
#include "mathConfig.h"

#include "controller.h"

pd_ctrl_t _pd;

/**********************************************************************************************************
*�� �� ��: pdCtrlInit
*����˵��: PD��ʽ�ļ��ο�����������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void pdCtrlInit(void)
{
	_pd.Kp[1]= _pd.Kp[2]= _pd.Kp[3]= _pd.Kp[5]= _pd.Kp[6]= _pd.Kp[7]= 0; 
	ParamGetData(CONTROLLER_PD_Kp_X, &(_pd.Kp[0]), 4);
	ParamGetData(CONTROLLER_PD_Kp_Y, &(_pd.Kp[4]), 4);
	ParamGetData(CONTROLLER_PD_Kp_Z, &(_pd.Kp[8]), 4);
	
	_pd.Kv[1]= _pd.Kv[2]= _pd.Kv[3]= _pd.Kv[5]= _pd.Kv[6]= _pd.Kv[7]= 0; 
	ParamGetData(CONTROLLER_PD_Kv_X, &(_pd.Kv[0]), 4);
	ParamGetData(CONTROLLER_PD_Kv_Y, &(_pd.Kv[4]), 4);
	ParamGetData(CONTROLLER_PD_Kv_Z, &(_pd.Kv[8]), 4);
	
	_pd.KR[1]= _pd.KR[2]= _pd.KR[3]= _pd.KR[5]= _pd.KR[6]= _pd.KR[7]= 0;
	ParamGetData(CONTROLLER_PD_KR_X, &(_pd.KR[0]), 4);
	ParamGetData(CONTROLLER_PD_KR_Y, &(_pd.KR[4]), 4);
	ParamGetData(CONTROLLER_PD_KR_Z, &(_pd.KR[8]), 4);
	
	_pd.KW[1]= _pd.KW[2]= _pd.KW[3]= _pd.KW[5]= _pd.KW[6]= _pd.KW[7]= 0;
	ParamGetData(CONTROLLER_PD_KW_X, &(_pd.KW[0]), 4);
	ParamGetData(CONTROLLER_PD_KW_Y, &(_pd.KW[4]), 4);
	ParamGetData(CONTROLLER_PD_KW_Z, &(_pd.KW[8]), 4);
}

/**********************************************************************************************************
*�� �� ��: pdCtrlUpdateParam
*����˵��: PD��ʽ�ļ��ο����������޸�
*��    ��: ����ѡ��param, �����޸�ֵdata
*�� �� ֵ: ��
**********************************************************************************************************/
void pdCtrlUpdateParam(uint16_t param, float data)
{
	switch(param)
	{
		case CONTROLLER_PD_Kp_X:
			_pd.Kp[0] = data;
		break;
		case CONTROLLER_PD_Kp_Y:
			_pd.Kp[4] = data;
		break;
		case CONTROLLER_PD_Kp_Z:
			_pd.Kp[8] = data;
		break;
		case CONTROLLER_PD_Kv_X:
			_pd.Kv[0] = data;
		break;
		case CONTROLLER_PD_Kv_Y:
			_pd.Kv[4] = data;
		break;
		case CONTROLLER_PD_Kv_Z:
			_pd.Kv[8] = data;
		break;
		case CONTROLLER_PD_KR_X:
			_pd.KR[0] = data;
		break;
		case CONTROLLER_PD_KR_Y:
			_pd.KR[4] = data;
		break;
		case CONTROLLER_PD_KR_Z:
			_pd.KR[8] = data;
		break;
		case CONTROLLER_PD_KW_X:
			_pd.KW[0] = data;
		break;
		case CONTROLLER_PD_KW_Y:
			_pd.KW[4] = data;
		break;
		case CONTROLLER_PD_KW_Z:
			_pd.KW[8] = data;
		break;
	}
	
	//��flash���޸���Ӧ����ֵ
	ParamUpdateData(param, &data);
}	

/**********************************************************************************************************
*�� �� ��: Attitude_Error
*����˵��: ���ο����е���̬������
*��    ��: void
*�� �� ֵ: void
**********************************************************************************************************/
void Attitude_Error(void)
{
	float R_des_T[9], R_fb_T[9], tmp1[9], tmp2[9], tmp3[9];
	
	Matrix3_Tran(_state.R_des, R_des_T);
	Matrix3_Tran(_state.R_fb, R_fb_T);
	
	Matrix3_Mul(R_des_T, _state.R_fb, tmp1);
	Matrix3_Mul(R_fb_T, _state.R_des, tmp2);
	Matrix3_Sub(tmp1, tmp2, tmp3);
	
	// vee_map
	_state.R_err.x = tmp3[7]/2;
	_state.R_err.y = tmp3[2]/2;
	_state.R_err.z = tmp3[3]/2;
}

/**********************************************************************************************************
*�� �� ��: Angular_Rate_Error
*����˵��: ���ο����еĽ��ٶ�������
*��    ��: void
*�� �� ֵ: void
**********************************************************************************************************/
void Angular_Rate_Error(void)
{
	float R_fb_T[9], tmp1[9];
	
	Matrix3_Tran(_state.R_fb, R_fb_T);
	
	Matrix3_Mul(R_fb_T, _state.R_des, tmp1);
	Vector3f_t v1 = Matrix3MulVector3(tmp1, _state.W_des);
	
	_state.W_err = Vector3f_Sub(_state.W_fb, v1);
}

/**********************************************************************************************************
*�� �� ��: ForceCal
*����˵��: ���ο������е�����������
*��    ��: ��
*�� �� ֵ: ���������� 
**********************************************************************************************************/
static Vector3f_t ForceCal(void)
{
	/***********��������м���***************/
	Vector3f_t v1, v2, v3, v4, v5, v6, v7;
	float R_T[9];
	/***********����λ�ú��ٶ����***********/
	_state.pos_err = Vector3f_Sub(_state.pos_fb, _state.pos_des);
	_state.vel_err = Vector3f_Sub(_state.vel_fb, _state.vel_des);
	
	// each element of Force equation 
	v1 = Matrix3MulVector3(_pd.Kp, _state.pos_err);		// first element
	v1.x=-v1.x; v1.y=-v1.y; v1.z=-v1.z;
	v2 = Matrix3MulVector3(_pd.Kv, _state.vel_err); 	// second element
	v3.x=0.0f; v3.y=0.0f; v3.z=_veh.mass*GRAVITY_ACCEL;	//masss*g*[0;0;1]
	v4.x = _veh.mass * _state.acc_des.x;
	v4.y = _veh.mass * _state.acc_des.y;
	v4.z = _veh.mass * _state.acc_des.z;			// mass.*desired_acceleration
	
	// add all the elements
	v5 = Vector3f_Sub(v1, v2);
	v6 = Vector3f_Add(v5, v3);
	v7 = Vector3f_Add(v6, v4);
	
	// Transpose of attitude matrix
	Matrix3_Tran(_state.R_fb, R_T);
	
	return Matrix3MulVector3(R_T, v7);
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
	Vector3f_t v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
	Vector3f_t alpha_d;
	float R_fb_T[9], W_fb_hat[9];
	
	//�м�������
	Hat_Map(W_fb_hat, _state.W_fb);
	Matrix3_Tran(_state.R_fb, R_fb_T);
	Attitude_Error();
	Angular_Rate_Error();
	
	//each element of moment function
	v1 = Matrix3MulVector3(_pd.KR, _state.R_err);		//first element
	v1.x=-v1.x; v1.y=-v1.y; v1.z=-v1.z;
	v2 = Matrix3MulVector3(_pd.KW, _state.W_err);		//second element
	v3 = Matrix3MulVector3(_veh.J, _state.W_fb);	//third element
	v4 = Matrix3MulVector3(W_fb_hat, v3);		
	// �����м���alpha_d
	v5 = Matrix3MulVector3(_state.R_des, _state.W_des);	// First element
	v6 = Matrix3MulVector3(R_fb_T, v5);
	v7 = Matrix3MulVector3(W_fb_hat, v6);
	v8 = Matrix3MulVector3(_state.R_des, _state.W_dot_des); //Second element
	v9 = Matrix3MulVector3(R_fb_T, v8);
	alpha_d = Vector3f_Sub(v9, v7);
	
	// add all the elements
	v10 = Vector3f_Sub(v1, v2);
	v11 = Vector3f_Add(v10, v4);
	
	// J * alpha_d
	v12 = Matrix3MulVector3(_veh.J, alpha_d);
	
	return Vector3f_Add(v11, v12);
}

/**********************************************************************************************************
*�� �� ��: pdCtrlUpdate
*����˵��: PD��ʽ�ļ��ο�����������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void pdCtrlUpdate(void)
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

