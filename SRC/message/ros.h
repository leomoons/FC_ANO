#ifndef __ROS_H
#define __ROS_H

#include "stdint.h"
#include "mathConfig.h"

typedef struct 
{
	uint8_t online;			//超过一秒没更新位姿信息就将online置为0
	
	Vector3f_t pos_des;
	Vector3f_t pos_fb;
	Vector3f_t vel_des;
	Vector3f_t vel_fb;
	Vector3f_t acc_des;
}receiveData;

void Ros_Get_Byte(uint8_t byte);
void Ros_Get_Data_Task(void);

Vector3f_t GetRosPosDes(void);
Vector3f_t GetRosPosFb(void);
Vector3f_t GetRosVelDes(void);
Vector3f_t GetRosVelFb(void);
Vector3f_t GetRosAccDes(void);


void Send_to_Ros(void);



#endif
