#ifndef __LYH_SMC_H
#define __LYH_SMC_H
#include "mathConfig.h"


typedef struct 
{
	float csP[9];
	float HsP[9];
	float ksP[9];
	float csR[9];
	float HsR[9];
	float ksR[9];
	
	Vector3f_t sP;		//λ�û�ģ��
	Vector3f_t satsP;
	Vector3f_t Fa;	
	Vector3f_t Fs;
	
	Vector3f_t sR;		//��̬��ģ��
	Vector3f_t satsR;
	Vector3f_t Ma;	
	Vector3f_t Ms;
}smc_ctrl_t;
extern smc_ctrl_t _smc;

void smcCtrlInit(void);
void smcCtrlUpdateParam(uint16_t param, float data);
void smcCtrlUpdate(void);

#endif
