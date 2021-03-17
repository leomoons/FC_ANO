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
	Vector3f_t force;
	float R_fb[9];
	Vector3f_t W_fb;
	Vector3f_t moment;
	
	Vector3f_t zP;
	Vector3f_t zR;
}matData;

void Mat_Get_Byte(uint8_t byte);
void Mat_Get_Data_Task(void);

Vector3f_t GetMatPosDes(void);
Vector3f_t GetMatPosFb(void);
Vector3f_t GetMatVelDes(void);
Vector3f_t GetMatVelFb(void);
Vector3f_t GetMatAccDes(void);
Vector3f_t GetMatForce(void);

void GetMatDcmDes(float* dcm);
void GetMatDcmFb(float* dcm);
Vector3f_t GetMatWDes(void);
Vector3f_t GetMatWFb(void);
Vector3f_t GetMatWdotDes(void);
Vector3f_t GetMatMoment(void);

Vector3f_t GetMatzP(void);
Vector3f_t GetMatzR(void);

void SendWrench(void);


#endif
