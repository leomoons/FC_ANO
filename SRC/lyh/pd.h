#ifndef __LYH_PD_H
#define __LYH_PD_H
#include "stm32f4xx.h"

typedef struct 
{
	float Kp[9];
	float Kv[9];
	float KR[9];
	float KW[9];
}pd_ctrl_t;

void pdCtrlInit(void);
void pdCtrlUpdateParam(uint16_t param, float data);
void Attitude_Error(void);
void Angular_Rate_Error(void);
void pdCtrlUpdate(void);

#endif /* !__LYH_PD_H */
