#ifndef __PARAM_H
#define __PARAM_H
#include "stm32f4xx.h"

#define PARAM_START_ADDR	0x00001000

enum PARAM_TYPE
{
	PARAM_CHECK_NUM,
	PARAM_CHECK_SUM,
		
	/*******PD����������ϵ��********/
		CONTROLLER_PD_Kp_X,
		CONTROLLER_PD_Kp_Y,
		CONTROLLER_PD_Kp_Z,
		CONTROLLER_PD_Kv_X,
		CONTROLLER_PD_Kv_Y,
		CONTROLLER_PD_Kv_Z,
		CONTROLLER_PD_KR_X,
		CONTROLLER_PD_KR_Y,
		CONTROLLER_PD_KR_Z,
		CONTROLLER_PD_KW_X,
		CONTROLLER_PD_KW_Y,
		CONTROLLER_PD_KW_Z,
		
	/*******SMC����������ϵ��********/
		CONTROLLER_SMC_csP_X,
		CONTROLLER_SMC_csP_Y,
		CONTROLLER_SMC_csP_Z,
		CONTROLLER_SMC_HsP_X,
		CONTROLLER_SMC_HsP_Y,
		CONTROLLER_SMC_HsP_Z,
		CONTROLLER_SMC_ksP_X,
		CONTROLLER_SMC_ksP_Y,
		CONTROLLER_SMC_ksP_Z,
		CONTROLLER_SMC_csR_X,
		CONTROLLER_SMC_csR_Y,
		CONTROLLER_SMC_csR_Z,
		CONTROLLER_SMC_HsR_X,
		CONTROLLER_SMC_HsR_Y,
		CONTROLLER_SMC_HsR_Z,
		CONTROLLER_SMC_ksR_X,
		CONTROLLER_SMC_ksR_Y,
		CONTROLLER_SMC_ksR_Z,
		
	/******************************/
    PARAM_NUM
};

union Parameter_u
{
	//������������������4����Ϊfloatռ4���ֽ�
	uint8_t buffer[PARAM_NUM*4];
	float data[PARAM_NUM];
};

void ParamInit(void);
void ParamReadFromFlash(void);
void ParamBufferReset(void);
void ParamSaveToFlash(void);
void ParamUpdateData(uint16_t dataNum, const void * data);
void ParamGetData(uint16_t dataNum, void *data, uint8_t length);
//const char* ParamGetString(uint8_t paramNum);
void Param_save_cnt_tox(uint8_t cnt);

#endif
