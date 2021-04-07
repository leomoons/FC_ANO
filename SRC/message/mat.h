#ifndef __MAT_H
#define __MAT_H

#include "stdint.h"
#include "mathConfig.h"

typedef struct
{
	Vector3f_t pos_des;
	Vector3f_t vel_des;
	Vector3f_t acc_des;
	float R_des[9];
	Vector3f_t W_des;
	Vector3f_t Wdot_des;
	
	Vector3f_t pos_fb;
	Vector3f_t vel_fb;
	float R_fb[9];
	Vector3f_t W_fb;
}matData;

void Mat_Get_Byte(uint8_t byte);
void Mat_Get_Data_Task(void);

Vector3f_t GetMatPosDes(void);
Vector3f_t GetMatPosFb(void);
Vector3f_t GetMatVelDes(void);
Vector3f_t GetMatVelFb(void);
Vector3f_t GetMatAccDes(void);

void GetMatDcmDes(float* dcm);
void GetMatDcmFb(float* dcm);
Vector3f_t GetMatWDes(void);
Vector3f_t GetMatWFb(void);
Vector3f_t GetMatWdotDes(void);


void SendPD(void);
void SendSMC(void);
void SendDOB(void);
void SendAR(void);
void SendAttDes(void);
void SendLog(void);

#endif
