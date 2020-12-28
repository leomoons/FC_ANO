#ifndef __OPTITRACK_H
#define __OPTITRACK_H
#include "stdint.h"
#include "mathConfig.h"

typedef struct 
{
	uint8_t online;			//����һ��û����λ����Ϣ�ͽ�online��Ϊ0
	
	Vector3f_t pos;
	Vector3f_t vel;
	Vector3f_t acc;
	
	float quat[4];		//ʹ����Ԫ������
	Vector3f_t euler;	//ŷ���Ǳ�ʾ
	float dcm[9];			//�������Ҿ���
	Vector3f_t W;			//���ٶ�
	Vector3f_t W_dot;		//�Ǽ��ٶ�
}Optitrack_t;

//void OptitrackInit(void);
void Opti_Get_Byte(uint8_t byte);
void Opti_Get_Data_Task(void);

Vector3f_t GetOptiPos(void);
Vector3f_t GetOptiVel(void);
Vector3f_t GetOptiAcc(void);

void GetOptiAttQuat(float* quat);
void GetOptiAttDCM(float* dcm);
Vector3f_t GetOptiAttEuler(void);
Vector3f_t GetOptiAngVel(void);
Vector3f_t GetOptiAngAcc(void);

#endif
