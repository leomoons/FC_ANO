/**********************************************************************************************************
 * @文件     mat.c
 * @说明     和matlab端进行数据交互
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2021.3
**********************************************************************************************************/
#include "mat.h"
#include "controller.h"
#include "Drv_usart.h"
#include "controller.h"

#define MYHWADDR 0x05

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
matData _mat;


/**********************************************************************************************************
*函 数 名: Mat_Get_Byte
*功能说明: 一帧数据前三个字节用于校验和额外功能（记录有效数据长度），
*         最后一位将前面所有字节数据加和用于校验
*形    参: 字节
*返 回 值: void
**********************************************************************************************************/
//Mat_data_len记录一帧数据的所有字节长度，包含了校验用的四个字节
uint8_t Mat_RxBuffer[220], Mat_data_len, Mat_Data_OK;
void Mat_Get_Byte(uint8_t byte)
{
	static uint8_t _data_len = 0;
	static uint8_t _sta = 0;
	static uint8_t _rx_buf_len = 0;
	
	if(_sta==0 && byte==0xAA)	//帧头0xAA
	{
		_sta = 1;
		Mat_RxBuffer[0] = byte;
	}
	else if(_sta==1 && byte==0x12)	//数据源，0x12表示数据来自Ros
	{
		_sta = 2;
		Mat_RxBuffer[1] = byte;
	}
	else if(_sta==2)	//数据长度
	{
		_sta = 3;
		Mat_RxBuffer[2] = byte;
		_data_len = byte;
		_rx_buf_len = 3;
	}
	else if(_sta==3 && _data_len>0)
	{
		_data_len--;
		Mat_RxBuffer[_rx_buf_len++] = byte;
		if(_data_len==0)	_sta = 4;
	}
	else if(_sta==4)
	{
		_sta = 0;
		Mat_RxBuffer[_rx_buf_len] = byte;
		if(!Mat_Data_OK)
		{
			Mat_data_len = _rx_buf_len+1;
			Mat_Data_OK = 1;
		}
	}
	else 
		_sta = 0;
}

/**********************************************************************************************************
*函 数 名: Mat_Get_Data_Task
*功能说明: Mat数据接收总函数
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void Mat_Get_Data_Task(void)
{
	if(Mat_Data_OK)
	{
		Mat_Data_OK = 0;
		uint32_t check_sum = 0;
		for(uint8_t i=0; i<(Mat_data_len-1); i++)
		{
			check_sum += *(Mat_RxBuffer+i);
		}
		uint8_t tmp = check_sum%256;
		if(!(tmp == *(Mat_RxBuffer+Mat_data_len-1))) return; //最后一个字节用于加和校验
		
		_mat.pos_des.x = ((float)((s32)(((*(Mat_RxBuffer+6))<<24) + ((*(Mat_RxBuffer+5))<<16) + ((*(Mat_RxBuffer+4))<<8) + (*(Mat_RxBuffer+3)))))/10000000;
		_mat.pos_des.y = ((float)((s32)(((*(Mat_RxBuffer+10))<<24) + ((*(Mat_RxBuffer+9))<<16) + ((*(Mat_RxBuffer+8))<<8) + (*(Mat_RxBuffer+7)))))/10000000;
		_mat.pos_des.z = ((float)((s32)(((*(Mat_RxBuffer+14))<<24) + ((*(Mat_RxBuffer+13))<<16) + ((*(Mat_RxBuffer+12))<<8) + (*(Mat_RxBuffer+11)))))/10000000;
			
		_mat.vel_des.x = ((float)((s32)(((*(Mat_RxBuffer+18))<<24) + ((*(Mat_RxBuffer+17))<<16) + ((*(Mat_RxBuffer+16))<<8) + (*(Mat_RxBuffer+15)))))/10000000;
		_mat.vel_des.y = ((float)((s32)(((*(Mat_RxBuffer+22))<<24) + ((*(Mat_RxBuffer+21))<<16) + ((*(Mat_RxBuffer+20))<<8) + (*(Mat_RxBuffer+19)))))/10000000;
		_mat.vel_des.z = ((float)((s32)(((*(Mat_RxBuffer+26))<<24) + ((*(Mat_RxBuffer+25))<<16) + ((*(Mat_RxBuffer+24))<<8) + (*(Mat_RxBuffer+23)))))/10000000;
			
		_mat.acc_des.x = ((float)((s32)(((*(Mat_RxBuffer+30))<<24) + ((*(Mat_RxBuffer+29))<<16) + ((*(Mat_RxBuffer+28))<<8) + (*(Mat_RxBuffer+27)))))/10000000;
		_mat.acc_des.y = ((float)((s32)(((*(Mat_RxBuffer+34))<<24) + ((*(Mat_RxBuffer+33))<<16) + ((*(Mat_RxBuffer+32))<<8) + (*(Mat_RxBuffer+31)))))/10000000;
		_mat.acc_des.z = ((float)((s32)(((*(Mat_RxBuffer+38))<<24) + ((*(Mat_RxBuffer+37))<<16) + ((*(Mat_RxBuffer+36))<<8) + (*(Mat_RxBuffer+35)))))/10000000;
		
		_mat.R_des[0] = ((float)((s32)(((*(Mat_RxBuffer+42))<<24) + ((*(Mat_RxBuffer+41))<<16) + ((*(Mat_RxBuffer+40))<<8) + (*(Mat_RxBuffer+39)))))/10000000;
		_mat.R_des[3] = ((float)((s32)(((*(Mat_RxBuffer+46))<<24) + ((*(Mat_RxBuffer+45))<<16) + ((*(Mat_RxBuffer+44))<<8) + (*(Mat_RxBuffer+43)))))/10000000;
		_mat.R_des[6] = ((float)((s32)(((*(Mat_RxBuffer+50))<<24) + ((*(Mat_RxBuffer+49))<<16) + ((*(Mat_RxBuffer+48))<<8) + (*(Mat_RxBuffer+47)))))/10000000;
		_mat.R_des[1] = ((float)((s32)(((*(Mat_RxBuffer+54))<<24) + ((*(Mat_RxBuffer+53))<<16) + ((*(Mat_RxBuffer+52))<<8) + (*(Mat_RxBuffer+51)))))/10000000;
		_mat.R_des[4] = ((float)((s32)(((*(Mat_RxBuffer+58))<<24) + ((*(Mat_RxBuffer+57))<<16) + ((*(Mat_RxBuffer+56))<<8) + (*(Mat_RxBuffer+55)))))/10000000;
		_mat.R_des[7] = ((float)((s32)(((*(Mat_RxBuffer+62))<<24) + ((*(Mat_RxBuffer+61))<<16) + ((*(Mat_RxBuffer+60))<<8) + (*(Mat_RxBuffer+59)))))/10000000;
		_mat.R_des[2] = ((float)((s32)(((*(Mat_RxBuffer+66))<<24) + ((*(Mat_RxBuffer+65))<<16) + ((*(Mat_RxBuffer+64))<<8) + (*(Mat_RxBuffer+63)))))/10000000;
		_mat.R_des[5] = ((float)((s32)(((*(Mat_RxBuffer+70))<<24) + ((*(Mat_RxBuffer+69))<<16) + ((*(Mat_RxBuffer+68))<<8) + (*(Mat_RxBuffer+67)))))/10000000;
		_mat.R_des[8] = ((float)((s32)(((*(Mat_RxBuffer+74))<<24) + ((*(Mat_RxBuffer+73))<<16) + ((*(Mat_RxBuffer+72))<<8) + (*(Mat_RxBuffer+71)))))/10000000;
			
		_mat.W_des.x = ((float)((s32)(((*(Mat_RxBuffer+78))<<24) + ((*(Mat_RxBuffer+77))<<16) + ((*(Mat_RxBuffer+76))<<8) + (*(Mat_RxBuffer+75)))))/10000000;
		_mat.W_des.y = ((float)((s32)(((*(Mat_RxBuffer+82))<<24) + ((*(Mat_RxBuffer+81))<<16) + ((*(Mat_RxBuffer+80))<<8) + (*(Mat_RxBuffer+79)))))/10000000;
		_mat.W_des.z = ((float)((s32)(((*(Mat_RxBuffer+86))<<24) + ((*(Mat_RxBuffer+85))<<16) + ((*(Mat_RxBuffer+84))<<8) + (*(Mat_RxBuffer+83)))))/10000000;
			
		_mat.Wdot_des.x = ((float)((s32)(((*(Mat_RxBuffer+90))<<24) + ((*(Mat_RxBuffer+89))<<16) + ((*(Mat_RxBuffer+88))<<8) + (*(Mat_RxBuffer+87)))))/10000000;
		_mat.Wdot_des.y = ((float)((s32)(((*(Mat_RxBuffer+94))<<24) + ((*(Mat_RxBuffer+93))<<16) + ((*(Mat_RxBuffer+92))<<8) + (*(Mat_RxBuffer+91)))))/10000000;
		_mat.Wdot_des.z = ((float)((s32)(((*(Mat_RxBuffer+98))<<24) + ((*(Mat_RxBuffer+97))<<16) + ((*(Mat_RxBuffer+96))<<8) + (*(Mat_RxBuffer+95)))))/10000000;
		
		
		_mat.pos_fb.x = ((float)((s32)(((*(Mat_RxBuffer+102))<<24) + ((*(Mat_RxBuffer+101))<<16) + ((*(Mat_RxBuffer+100))<<8) + (*(Mat_RxBuffer+99)))))/10000000;
		_mat.pos_fb.y = ((float)((s32)(((*(Mat_RxBuffer+106))<<24) + ((*(Mat_RxBuffer+105))<<16) + ((*(Mat_RxBuffer+104))<<8) + (*(Mat_RxBuffer+103)))))/10000000;
		_mat.pos_fb.z = ((float)((s32)(((*(Mat_RxBuffer+110))<<24) + ((*(Mat_RxBuffer+109))<<16) + ((*(Mat_RxBuffer+108))<<8) + (*(Mat_RxBuffer+107)))))/10000000;
			
		_mat.vel_fb.x = ((float)((s32)(((*(Mat_RxBuffer+114))<<24) + ((*(Mat_RxBuffer+113))<<16) + ((*(Mat_RxBuffer+112))<<8) + (*(Mat_RxBuffer+111)))))/10000000;
		_mat.vel_fb.y = ((float)((s32)(((*(Mat_RxBuffer+118))<<24) + ((*(Mat_RxBuffer+117))<<16) + ((*(Mat_RxBuffer+116))<<8) + (*(Mat_RxBuffer+115)))))/10000000;
		_mat.vel_fb.z = ((float)((s32)(((*(Mat_RxBuffer+122))<<24) + ((*(Mat_RxBuffer+121))<<16) + ((*(Mat_RxBuffer+120))<<8) + (*(Mat_RxBuffer+119)))))/10000000;
			
		_mat.force.x = ((float)((s32)(((*(Mat_RxBuffer+126))<<24) + ((*(Mat_RxBuffer+125))<<16) + ((*(Mat_RxBuffer+124))<<8) + (*(Mat_RxBuffer+123)))))/10000000;
		_mat.force.y = ((float)((s32)(((*(Mat_RxBuffer+130))<<24) + ((*(Mat_RxBuffer+129))<<16) + ((*(Mat_RxBuffer+128))<<8) + (*(Mat_RxBuffer+127)))))/10000000;
		_mat.force.z = ((float)((s32)(((*(Mat_RxBuffer+134))<<24) + ((*(Mat_RxBuffer+133))<<16) + ((*(Mat_RxBuffer+132))<<8) + (*(Mat_RxBuffer+131)))))/10000000;
			
		_mat.R_fb[0] = ((float)((s32)(((*(Mat_RxBuffer+138))<<24) + ((*(Mat_RxBuffer+137))<<16) + ((*(Mat_RxBuffer+136))<<8) + (*(Mat_RxBuffer+135)))))/10000000;
		_mat.R_fb[3] = ((float)((s32)(((*(Mat_RxBuffer+142))<<24) + ((*(Mat_RxBuffer+141))<<16) + ((*(Mat_RxBuffer+140))<<8) + (*(Mat_RxBuffer+139)))))/10000000;
		_mat.R_fb[6] = ((float)((s32)(((*(Mat_RxBuffer+146))<<24) + ((*(Mat_RxBuffer+145))<<16) + ((*(Mat_RxBuffer+144))<<8) + (*(Mat_RxBuffer+143)))))/10000000;
		_mat.R_fb[1] = ((float)((s32)(((*(Mat_RxBuffer+150))<<24) + ((*(Mat_RxBuffer+149))<<16) + ((*(Mat_RxBuffer+148))<<8) + (*(Mat_RxBuffer+147)))))/10000000;
		_mat.R_fb[4] = ((float)((s32)(((*(Mat_RxBuffer+154))<<24) + ((*(Mat_RxBuffer+153))<<16) + ((*(Mat_RxBuffer+152))<<8) + (*(Mat_RxBuffer+151)))))/10000000;
		_mat.R_fb[7] = ((float)((s32)(((*(Mat_RxBuffer+158))<<24) + ((*(Mat_RxBuffer+157))<<16) + ((*(Mat_RxBuffer+156))<<8) + (*(Mat_RxBuffer+155)))))/10000000;
		_mat.R_fb[2] = ((float)((s32)(((*(Mat_RxBuffer+162))<<24) + ((*(Mat_RxBuffer+161))<<16) + ((*(Mat_RxBuffer+160))<<8) + (*(Mat_RxBuffer+159)))))/10000000;
		_mat.R_fb[5] = ((float)((s32)(((*(Mat_RxBuffer+166))<<24) + ((*(Mat_RxBuffer+165))<<16) + ((*(Mat_RxBuffer+164))<<8) + (*(Mat_RxBuffer+163)))))/10000000;
		_mat.R_fb[8] = ((float)((s32)(((*(Mat_RxBuffer+170))<<24) + ((*(Mat_RxBuffer+169))<<16) + ((*(Mat_RxBuffer+168))<<8) + (*(Mat_RxBuffer+167)))))/10000000;
			
		_mat.W_fb.x = ((float)((s32)(((*(Mat_RxBuffer+174))<<24) + ((*(Mat_RxBuffer+173))<<16) + ((*(Mat_RxBuffer+172))<<8) + (*(Mat_RxBuffer+171)))))/10000000;
		_mat.W_fb.y = ((float)((s32)(((*(Mat_RxBuffer+178))<<24) + ((*(Mat_RxBuffer+177))<<16) + ((*(Mat_RxBuffer+176))<<8) + (*(Mat_RxBuffer+175)))))/10000000;
		_mat.W_fb.z = ((float)((s32)(((*(Mat_RxBuffer+182))<<24) + ((*(Mat_RxBuffer+181))<<16) + ((*(Mat_RxBuffer+180))<<8) + (*(Mat_RxBuffer+179)))))/10000000;
			
		_mat.moment.x = ((float)((s32)(((*(Mat_RxBuffer+186))<<24) + ((*(Mat_RxBuffer+185))<<16) + ((*(Mat_RxBuffer+184))<<8) + (*(Mat_RxBuffer+183)))))/10000000;
		_mat.moment.y = ((float)((s32)(((*(Mat_RxBuffer+190))<<24) + ((*(Mat_RxBuffer+189))<<16) + ((*(Mat_RxBuffer+188))<<8) + (*(Mat_RxBuffer+187)))))/10000000;
		_mat.moment.z = ((float)((s32)(((*(Mat_RxBuffer+194))<<24) + ((*(Mat_RxBuffer+193))<<16) + ((*(Mat_RxBuffer+192))<<8) + (*(Mat_RxBuffer+191)))))/10000000;
		
		_mat.zP.x = ((float)((s32)(((*(Mat_RxBuffer+198))<<24) + ((*(Mat_RxBuffer+197))<<16) + ((*(Mat_RxBuffer+196))<<8) + (*(Mat_RxBuffer+195)))))/10000000;
		_mat.zP.y = ((float)((s32)(((*(Mat_RxBuffer+202))<<24) + ((*(Mat_RxBuffer+201))<<16) + ((*(Mat_RxBuffer+200))<<8) + (*(Mat_RxBuffer+199)))))/10000000;
		_mat.zP.z = ((float)((s32)(((*(Mat_RxBuffer+206))<<24) + ((*(Mat_RxBuffer+205))<<16) + ((*(Mat_RxBuffer+204))<<8) + (*(Mat_RxBuffer+203)))))/10000000;
		
		_mat.zR.x = ((float)((s32)(((*(Mat_RxBuffer+210))<<24) + ((*(Mat_RxBuffer+209))<<16) + ((*(Mat_RxBuffer+208))<<8) + (*(Mat_RxBuffer+207)))))/10000000;
		_mat.zR.y = ((float)((s32)(((*(Mat_RxBuffer+214))<<24) + ((*(Mat_RxBuffer+213))<<16) + ((*(Mat_RxBuffer+212))<<8) + (*(Mat_RxBuffer+211)))))/10000000;
		_mat.zR.z = ((float)((s32)(((*(Mat_RxBuffer+218))<<24) + ((*(Mat_RxBuffer+217))<<16) + ((*(Mat_RxBuffer+216))<<8) + (*(Mat_RxBuffer+215)))))/10000000;
		
		CtrlTask();
		
		SendWrench();
	}
}

Vector3f_t GetMatPosDes(void)
{
	return _mat.pos_des;
}
Vector3f_t GetMatPosFb(void)
{
	return _mat.pos_fb;
}
Vector3f_t GetMatVelDes(void)
{
	return _mat.vel_des;
}
Vector3f_t GetMatVelFb(void)
{
	return _mat.vel_fb;
}
Vector3f_t GetMatAccDes(void)
{
	return _mat.acc_des;
}
Vector3f_t GetMatForce(void)
{
	return _mat.force;
}

void GetMatDcmDes(float* dcm)
{
	Matrix3_Copy(_mat.R_des, dcm);
}
void GetMatDcmFb(float* dcm)
{
	Matrix3_Copy(_mat.R_fb, dcm);
}
Vector3f_t GetMatWDes(void)
{
	return _mat.W_des;
}
Vector3f_t GetMatWFb(void)
{
	return _mat.W_fb;
}
Vector3f_t GetMatWdotDes(void)
{
	return _mat.Wdot_des;
}
Vector3f_t GetMatMoment(void)
{
	return _mat.moment;
}
Vector3f_t GetMatzP(void)
{
	return _mat.zP;
}
Vector3f_t GetMatzR(void)
{
	return _mat.zR;
}

//////////////////////////////////////////////////////////////////////////////////////
//STM32端发送串口信号到Matlab端，
uint8_t sendBuf[104];
float Wrench[24];
#include "disturbanceEst.h"
#include "ndob.h"
/**********************************************************************************************************
*函 数 名: SendWrench
*功能说明: 发送控制效果相关数据到Ros端，通过usart3
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void SendWrench(void)
{
//	static uint16_t loop_cnt = 0;
//	if(loop_cnt%10==0)
//	{
		uint8_t _cnt = 0;
		
		// 控制器生成控制效果
		Wrench[0] = _ctrl_only.F_b.x;
		Wrench[1] = _ctrl_only.F_b.y;
		Wrench[2] = _ctrl_only.F_b.z;
		Wrench[3] = _ctrl_only.M_b.x;
		Wrench[4] = _ctrl_only.M_b.y;
		Wrench[5] = _ctrl_only.M_b.z;
		
		// 扰动估计（惯性坐标系中）
		Wrench[6] = _dob.F_I.x;
		Wrench[7] = _dob.F_I.y;
		Wrench[8] = _dob.F_I.z;
		Wrench[9] = _dob._est.M_b.x;
		Wrench[10] = _dob._est.M_b.y;
		Wrench[11] = _dob._est.M_b.z;
	
		// zP zR
		Wrench[12] = _dob.zP.x;
		Wrench[13] = _dob.zP.y;
		Wrench[14] = _dob.zP.z;
		Wrench[15] = _dob.zR.x;
		Wrench[16] = _dob.zR.y;
		Wrench[17] = _dob.zR.z;
		
		// zP_dot zR_dot
		Wrench[18] = _dob.zP_dot.x;
		Wrench[19] = _dob.zP_dot.y;
		Wrench[20] = _dob.zP_dot.z;
		Wrench[21] = _dob.zR_dot.x;
		Wrench[22] = _dob.zR_dot.y;
		Wrench[23] = _dob.zR_dot.z;
		
		
		// 往发送buf里装填字符串
		sendBuf[_cnt++] = 0xAA;
		sendBuf[_cnt++] = MYHWADDR;	//数据发送端识别码
		sendBuf[_cnt++] = 96;
		
		uint32_t check_sum = sendBuf[0]+sendBuf[1]+sendBuf[2];
		for(int i=0; i<24; i++)
		{
			int32_t Datai = Wrench[i]*10000000;
			
			sendBuf[3+i*4] = BYTE0(Datai);
			sendBuf[4+i*4] = BYTE1(Datai);
			sendBuf[5+i*4] = BYTE2(Datai);
			sendBuf[6+i*4] = BYTE3(Datai);
			check_sum += (sendBuf[3+i*4]+sendBuf[4+i*4]+sendBuf[5+i*4]+sendBuf[6+i*4]);
		}
		sendBuf[99] = (uint8_t)check_sum%256;
		
		Usart2_Send(sendBuf, 100);
//	}
}














