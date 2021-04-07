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
uint8_t Mat_RxBuffer[180], Mat_data_len, Mat_Data_OK;
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
			
		_mat.R_fb[0] = ((float)((s32)(((*(Mat_RxBuffer+126))<<24) + ((*(Mat_RxBuffer+125))<<16) + ((*(Mat_RxBuffer+124))<<8) + (*(Mat_RxBuffer+123)))))/10000000;
		_mat.R_fb[3] = ((float)((s32)(((*(Mat_RxBuffer+130))<<24) + ((*(Mat_RxBuffer+129))<<16) + ((*(Mat_RxBuffer+128))<<8) + (*(Mat_RxBuffer+127)))))/10000000;
		_mat.R_fb[6] = ((float)((s32)(((*(Mat_RxBuffer+134))<<24) + ((*(Mat_RxBuffer+133))<<16) + ((*(Mat_RxBuffer+132))<<8) + (*(Mat_RxBuffer+131)))))/10000000;
		_mat.R_fb[1] = ((float)((s32)(((*(Mat_RxBuffer+138))<<24) + ((*(Mat_RxBuffer+137))<<16) + ((*(Mat_RxBuffer+136))<<8) + (*(Mat_RxBuffer+135)))))/10000000;
		_mat.R_fb[4] = ((float)((s32)(((*(Mat_RxBuffer+142))<<24) + ((*(Mat_RxBuffer+141))<<16) + ((*(Mat_RxBuffer+140))<<8) + (*(Mat_RxBuffer+139)))))/10000000;
		_mat.R_fb[7] = ((float)((s32)(((*(Mat_RxBuffer+146))<<24) + ((*(Mat_RxBuffer+145))<<16) + ((*(Mat_RxBuffer+144))<<8) + (*(Mat_RxBuffer+143)))))/10000000;
		_mat.R_fb[2] = ((float)((s32)(((*(Mat_RxBuffer+150))<<24) + ((*(Mat_RxBuffer+149))<<16) + ((*(Mat_RxBuffer+148))<<8) + (*(Mat_RxBuffer+147)))))/10000000;
		_mat.R_fb[5] = ((float)((s32)(((*(Mat_RxBuffer+154))<<24) + ((*(Mat_RxBuffer+153))<<16) + ((*(Mat_RxBuffer+152))<<8) + (*(Mat_RxBuffer+151)))))/10000000;
		_mat.R_fb[8] = ((float)((s32)(((*(Mat_RxBuffer+158))<<24) + ((*(Mat_RxBuffer+157))<<16) + ((*(Mat_RxBuffer+156))<<8) + (*(Mat_RxBuffer+155)))))/10000000;
			
		_mat.W_fb.x = ((float)((s32)(((*(Mat_RxBuffer+162))<<24) + ((*(Mat_RxBuffer+161))<<16) + ((*(Mat_RxBuffer+160))<<8) + (*(Mat_RxBuffer+159)))))/10000000;
		_mat.W_fb.y = ((float)((s32)(((*(Mat_RxBuffer+166))<<24) + ((*(Mat_RxBuffer+165))<<16) + ((*(Mat_RxBuffer+164))<<8) + (*(Mat_RxBuffer+163)))))/10000000;
		_mat.W_fb.z = ((float)((s32)(((*(Mat_RxBuffer+170))<<24) + ((*(Mat_RxBuffer+169))<<16) + ((*(Mat_RxBuffer+168))<<8) + (*(Mat_RxBuffer+167)))))/10000000;
		
		//用于控制单元测试时用
		CtrlTask();
		
		// 选择以测试不同模块
		//SendPD();				//测试单独PD模块	
		//SendSMC();			//测试单独SMC模块
		//SendDOB();		//测试ndob模块
		//SendAR();			//测试adaptive模块
		
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



uint8_t sendBuf[110];
/**********************************************************************************************************
*函 数 名: SendPD
*功能说明: 发送PD控制效果相关数据到Ros端，通过usart2
*形    参: void
*返 回 值: void
**********************************************************************************************************/
float pdData[6];
void SendPD(void)
{
	uint8_t _cnt = 0;
		
		// 控制器生成控制效果 （机体坐标）
		pdData[0] = _ctrl.F_b.x;
		pdData[1] = _ctrl.F_b.y;
		pdData[2] = _ctrl.F_b.z;
		pdData[3] = _ctrl.M_b.x;
		pdData[4] = _ctrl.M_b.y;
		pdData[5] = _ctrl.M_b.z;
		
		// 往发送buf里装填字符串
		sendBuf[_cnt++] = 0xAA;
		sendBuf[_cnt++] = MYHWADDR;	//数据发送端识别码
		sendBuf[_cnt++] = 24;
		
		uint32_t check_sum = sendBuf[0]+sendBuf[1]+sendBuf[2];
		for(int i=0; i<6; i++)
		{
			int32_t Datai = pdData[i]*10000000;
			
			sendBuf[3+i*4] = BYTE0(Datai);
			sendBuf[4+i*4] = BYTE1(Datai);
			sendBuf[5+i*4] = BYTE2(Datai);
			sendBuf[6+i*4] = BYTE3(Datai);
			check_sum += (sendBuf[3+i*4]+sendBuf[4+i*4]+sendBuf[5+i*4]+sendBuf[6+i*4]);
		}
		sendBuf[27] = (uint8_t)check_sum%256;
		
		Usart2_Send(sendBuf, 28);
		//Usart3_Send(sendBuf, 28);
}

/**********************************************************************************************************
*函 数 名: SendSMC
*功能说明: 发送SMC控制效果相关数据到Ros端，通过usart2
*形    参: void
*返 回 值: void
**********************************************************************************************************/
#include "smc.h"
float smcData[18];
void SendSMC(void)
{
	uint8_t _cnt = 0;
		
		// 控制器生成控制效果 （机体坐标）
		smcData[0] = _ctrl.F_b.x;
		smcData[1] = _ctrl.F_b.y;
		smcData[2] = _ctrl.F_b.z;
		smcData[3] = _ctrl.M_b.x;
		smcData[4] = _ctrl.M_b.y;
		smcData[5] = _ctrl.M_b.z;
	
		// Fa Fs
		smcData[6] = _smc.Fa.x;
		smcData[7] = _smc.Fa.y;
		smcData[8] = _smc.Fa.z;
		smcData[9] = _smc.Fs.x;
		smcData[10] = _smc.Fs.y;
		smcData[11] = _smc.Fs.z;
	
		// Ma Ms 
		smcData[12] = _smc.Ma.x;
		smcData[13] = _smc.Ma.y;
		smcData[14] = _smc.Ma.z;
		smcData[15] = _smc.Ms.x;
		smcData[16] = _smc.Ms.y;
		smcData[17] = _smc.Ms.z;
		
		// 往发送buf里装填字符串
		sendBuf[_cnt++] = 0xAA;
		sendBuf[_cnt++] = MYHWADDR;	//数据发送端识别码
		sendBuf[_cnt++] = 72;
		
		uint32_t check_sum = sendBuf[0]+sendBuf[1]+sendBuf[2];
		for(int i=0; i<18; i++)
		{
			int32_t Datai = smcData[i]*10000000;
			
			sendBuf[3+i*4] = BYTE0(Datai);
			sendBuf[4+i*4] = BYTE1(Datai);
			sendBuf[5+i*4] = BYTE2(Datai);
			sendBuf[6+i*4] = BYTE3(Datai);
			check_sum += (sendBuf[3+i*4]+sendBuf[4+i*4]+sendBuf[5+i*4]+sendBuf[6+i*4]);
		}
		sendBuf[75] = (uint8_t)check_sum%256;
		
		Usart2_Send(sendBuf, 76);
}

#include "ndob.h"
/**********************************************************************************************************
*函 数 名: SendDOB
*功能说明: 发送ndob相关数据到Ros端，通过usart2
*形    参: void
*返 回 值: void
**********************************************************************************************************/
float dobData[24];
void SendDOB(void)
{
		uint8_t _cnt = 0;
		
		// 控制器生成控制效果 （机体坐标）
		dobData[0] = _ctrl.F_b.x;
		dobData[1] = _ctrl.F_b.y;
		dobData[2] = _ctrl.F_b.z;
		dobData[3] = _ctrl.M_b.x;
		dobData[4] = _ctrl.M_b.y;
		dobData[5] = _ctrl.M_b.z;
		
		// 扰动估计（惯性坐标系）
		dobData[6] = _dob.F_I.x;
		dobData[7] = _dob.F_I.y;
		dobData[8] = _dob.F_I.z;
		dobData[9] = _dob.M_b.x;
		dobData[10] = _dob.M_b.y;
		dobData[11] = _dob.M_b.z;
	
		// zP zR
		dobData[12] = _dob.zP.x;
		dobData[13] = _dob.zP.y;
		dobData[14] = _dob.zP.z;
		dobData[15] = _dob.zR.x;
		dobData[16] = _dob.zR.y;
		dobData[17] = _dob.zR.z;
		
		// zP_dot zR_dot
		dobData[18] = _dob.zP_dot.x;
		dobData[19] = _dob.zP_dot.y;
		dobData[20] = _dob.zP_dot.z;
		dobData[21] = _dob.zR_dot.x;
		dobData[22] = _dob.zR_dot.y;
		dobData[23] = _dob.zR_dot.z;
		
		
		// 往发送buf里装填字符串
		sendBuf[_cnt++] = 0xAA;
		sendBuf[_cnt++] = MYHWADDR;	//数据发送端识别码
		sendBuf[_cnt++] = 96;
		
		uint32_t check_sum = sendBuf[0]+sendBuf[1]+sendBuf[2];
		for(int i=0; i<24; i++)
		{
			int32_t Datai = dobData[i]*10000000;
			
			sendBuf[3+i*4] = BYTE0(Datai);
			sendBuf[4+i*4] = BYTE1(Datai);
			sendBuf[5+i*4] = BYTE2(Datai);
			sendBuf[6+i*4] = BYTE3(Datai);
			check_sum += (sendBuf[3+i*4]+sendBuf[4+i*4]+sendBuf[5+i*4]+sendBuf[6+i*4]);
		}
		sendBuf[99] = (uint8_t)check_sum%256;
		
		Usart2_Send(sendBuf, 100);
}


#include "adaptive.h"
/**********************************************************************************************************
*函 数 名: SendAR
*功能说明: 发送adaptive相关数据到Ros端，通过usart2
*形    参: void
*返 回 值: void
**********************************************************************************************************/
float arData[18];
void SendAR(void)
{
	uint8_t _cnt = 0;
		
	// 控制器生成控制效果 （机体坐标）
	arData[0] = _ctrl.F_b.x;
	arData[1] = _ctrl.F_b.y;
	arData[2] = _ctrl.F_b.z;
	arData[3] = _ctrl.M_b.x;
	arData[4] = _ctrl.M_b.y;
	arData[5] = _ctrl.M_b.z;
		
	// 扰动估计（惯性坐标系）
	arData[6] = _ada.F_I.x;
	arData[7] = _ada.F_I.y;
	arData[8] = _ada.F_I.z;
	arData[9] = _ada.M_b.x;
	arData[10] = _ada.M_b.y;
	arData[11] = _ada.M_b.z;
	
	// dx_dot dR_fot
	arData[12] = _ada.dx_dot.x;
	arData[13] = _ada.dx_dot.y;
	arData[14] = _ada.dx_dot.z;
	arData[15] = _ada.dR_dot.x;
	arData[16] = _ada.dR_dot.y;
	arData[17] = _ada.dR_dot.z;
				
		
	// 往发送buf里装填字符串
	sendBuf[_cnt++] = 0xAA;
	sendBuf[_cnt++] = MYHWADDR;	//数据发送端识别码
	sendBuf[_cnt++] = 72;
		
	uint32_t check_sum = sendBuf[0]+sendBuf[1]+sendBuf[2];
	for(int i=0; i<18; i++)
	{
		int32_t Datai = arData[i]*10000000;
		
		sendBuf[3+i*4] = BYTE0(Datai);
		sendBuf[4+i*4] = BYTE1(Datai);
		sendBuf[5+i*4] = BYTE2(Datai);
		sendBuf[6+i*4] = BYTE3(Datai);
		check_sum += (sendBuf[3+i*4]+sendBuf[4+i*4]+sendBuf[5+i*4]+sendBuf[6+i*4]);
	}
	sendBuf[75] = (uint8_t)check_sum%256;
		
	Usart2_Send(sendBuf, 76);
}


/**********************************************************************************************************
*函 数 名: SendAttDes
*功能说明: 发送期望姿态，角速度，通过usart2
*形    参: void
*返 回 值: void
**********************************************************************************************************/
float AttDes[7];
void SendAttDes()
{
	uint8_t _cnt = 0;
		
	// 期望姿态（四元数相关）
	AttDes[0] = _state.q_des[0];
	AttDes[1] = _state.q_des[1];
	AttDes[2] = _state.q_des[2];
	AttDes[3] = _state.q_des[3];
	
	
	// 期望角速度
	AttDes[4] = _state.W_des.x;
	AttDes[5] = _state.W_des.y;
	AttDes[6] = _state.W_des.z;
			
	// 往发送buf里装填字符串
	sendBuf[_cnt++] = 0xAA;
	sendBuf[_cnt++] = MYHWADDR;	//数据发送端识别码
	sendBuf[_cnt++] = 28;
		
	uint32_t check_sum = sendBuf[0]+sendBuf[1]+sendBuf[2];
	for(int i=0; i<7; i++)
	{
		int32_t Datai = AttDes[i]*10000000;
			
		sendBuf[3+i*4] = BYTE0(Datai);
		sendBuf[4+i*4] = BYTE1(Datai);
		sendBuf[5+i*4] = BYTE2(Datai);
		sendBuf[6+i*4] = BYTE3(Datai);
		check_sum += (sendBuf[3+i*4]+sendBuf[4+i*4]+sendBuf[5+i*4]+sendBuf[6+i*4]);
	}
	sendBuf[31] = (uint8_t)check_sum%256;
		
	Usart2_Send(sendBuf, 32);
}



/**********************************************************************************************************
*函 数 名: SendLog
*功能说明: 发送需要log的信号，通过usart2
*形    参: void
*返 回 值: void
**********************************************************************************************************/
float logData[26];
void SendLog(void)
{
	uint8_t _cnt = 0;
		
	// 位置相关数据
	logData[0] = _state.pos_des.x;
	logData[1] = _state.pos_des.y;
	logData[2] = _state.pos_des.z;
	logData[3] = _state.pos_fb.x;
	logData[4] = _state.pos_fb.y;
	logData[5] = _state.pos_fb.z;
		
	// 姿态相关数据（四元数相关）
	logData[6] = _state.q_des[0];
	logData[7] = _state.q_des[1];
	logData[8] = _state.q_des[2];
	logData[9] = _state.q_des[3];
	logData[10] = _state.q_fb[0];
	logData[11] = _state.q_fb[1];
	logData[12] = _state.q_fb[2];
	logData[13] = _state.q_fb[3];
	
	// 由控制器单独生成的控制效果
	logData[14] = _ctrl_only.F_b.x;
	logData[15] = _ctrl_only.F_b.y;
	logData[16] = _ctrl_only.F_b.z;
	logData[17] = _ctrl_only.M_b.x;
	logData[18] = _ctrl_only.M_b.y;
	logData[19] = _ctrl_only.M_b.z;
		
	// 扰动估计 表示在惯性坐标系中
	logData[20] = _est.F_I.x;
	logData[21] = _est.F_I.y;
	logData[22] = _est.F_I.z;
	logData[23] = _est.M_b.x;
	logData[24] = _est.M_b.y;
	logData[25] = _est.M_b.z;
		
		
	// 往发送buf里装填字符串
	sendBuf[_cnt++] = 0xAA;
	sendBuf[_cnt++] = MYHWADDR;	//数据发送端识别码
	sendBuf[_cnt++] = 104;
		
	uint32_t check_sum = sendBuf[0]+sendBuf[1]+sendBuf[2];
	for(int i=0; i<26; i++)
	{
		int32_t Datai = logData[i]*10000000;
			
		sendBuf[3+i*4] = BYTE0(Datai);
		sendBuf[4+i*4] = BYTE1(Datai);
		sendBuf[5+i*4] = BYTE2(Datai);
		sendBuf[6+i*4] = BYTE3(Datai);
		check_sum += (sendBuf[3+i*4]+sendBuf[4+i*4]+sendBuf[5+i*4]+sendBuf[6+i*4]);
	}
	sendBuf[107] = (uint8_t)check_sum%256;
		
	Usart2_Send(sendBuf, 108);
}



