/**********************************************************************************************************
 * @�ļ�     ros.c
 * @˵��     ��ros�˽������ݽ�������ros�˽���λ�õ���������ͷ�����������̬����������ͷ���
 * @�汾  	 V1.0
 * @����     Leomoon
 * @����     2021.2
**********************************************************************************************************/
#include "ros.h"
#include "controller.h"
#include "Drv_usart.h"

#define MYHWADDR	0x05

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

receiveData _receive;




/**********************************************************************************************************
*�� �� ��: Ros_Get_Byte
*����˵��: һ֡����ǰ�����ֽ�����У��Ͷ��⹦�ܣ���¼��Ч���ݳ��ȣ���
*         ���һλ��ǰ�������ֽ����ݼӺ�����У��
*��    ��: �ֽ�
*�� �� ֵ: void
**********************************************************************************************************/
//Ros_data_len��¼һ֡���ݵ������ֽڳ��ȣ�������У���õ��ĸ��ֽ�
uint8_t Ros_RxBuffer[64], Ros_data_len=0, Ros_Data_OK;
void Ros_Get_Byte(uint8_t byte)
{
	static uint8_t _data_len = 0;
	static uint8_t _sta = 0;
	static uint8_t _rx_buf_len = 0;
	
	if(_sta==0 && byte==0xAA)	//֡ͷ0xAA
	{
		_sta = 1;
		Ros_RxBuffer[0] = byte;
	}
	else if(_sta==1 && byte==0x12)	//����Դ��0x12��ʾ��������Ros
	{
		_sta = 2;
		Ros_RxBuffer[1] = byte;
	}
	else if(_sta==2)	//���ݳ���
	{
		_sta = 3;
		Ros_RxBuffer[2] = byte;
		_data_len = byte;
		_rx_buf_len = 3;
	}
	else if(_sta==3 && _data_len>0)
	{
		_data_len--;
		Ros_RxBuffer[_rx_buf_len++] = byte;
		if(_data_len==0)	_sta = 4;
	}
	else if(_sta==4)
	{
		_sta = 0;
		Ros_RxBuffer[_rx_buf_len] = byte;
		if(!Ros_Data_OK)
		{
			Ros_data_len = _rx_buf_len+1;
			Ros_Data_OK = 1;
		}
	}
	else 
		_sta = 0;
}

/**********************************************************************************************************
*�� �� ��: Ros_Get_Data_Task
*����˵��: Ros���ݽ����ܺ���
*��    ��: void
*�� �� ֵ: void
**********************************************************************************************************/
void Ros_Get_Data_Task(void)
{
	static uint16_t ros_check_cnt;
	
	if(Ros_Data_OK)
	{
		Ros_Data_OK = 0;
		uint8_t check_sum = 0;
		for(uint8_t i=0; i<(Ros_data_len-1); i++)
		{
			check_sum += *(Ros_RxBuffer+i);
		}
		if(!(check_sum == *(Ros_RxBuffer+Ros_data_len-1))) return;	//���һ���ֽ����ڼӺ�У��
		
		_receive.pos_des.x = ((float)((s32)(((*(Ros_RxBuffer+3))<<24) + ((*(Ros_RxBuffer+4))<<16) + ((*(Ros_RxBuffer+5))<<8) + (*(Ros_RxBuffer+6)))))/100;
		_receive.pos_des.y = ((float)((s32)(((*(Ros_RxBuffer+7))<<24) + ((*(Ros_RxBuffer+8))<<16) + ((*(Ros_RxBuffer+9))<<8) + (*(Ros_RxBuffer+10)))))/100;
		_receive.pos_des.z = ((float)((s32)(((*(Ros_RxBuffer+11))<<24) + ((*(Ros_RxBuffer+12))<<16) + ((*(Ros_RxBuffer+13))<<8) + (*(Ros_RxBuffer+14)))))/100;
			
		_receive.pos_fb.x = ((float)((s32)(((*(Ros_RxBuffer+15))<<24) + ((*(Ros_RxBuffer+16))<<16) + ((*(Ros_RxBuffer+17))<<8) + (*(Ros_RxBuffer+18)))))/100;
		_receive.pos_fb.y = ((float)((s32)(((*(Ros_RxBuffer+19))<<24) + ((*(Ros_RxBuffer+20))<<16) + ((*(Ros_RxBuffer+21))<<8) + (*(Ros_RxBuffer+22)))))/100;
		_receive.pos_fb.z = ((float)((s32)(((*(Ros_RxBuffer+23))<<24) + ((*(Ros_RxBuffer+24))<<16) + ((*(Ros_RxBuffer+25))<<8) + (*(Ros_RxBuffer+26)))))/100;
			
		_receive.vel_des.x = ((float)((s32)(((*(Ros_RxBuffer+27))<<24) + ((*(Ros_RxBuffer+28))<<16) + ((*(Ros_RxBuffer+29))<<8) + (*(Ros_RxBuffer+30)))))/100;
		_receive.vel_des.y = ((float)((s32)(((*(Ros_RxBuffer+31))<<24) + ((*(Ros_RxBuffer+32))<<16) + ((*(Ros_RxBuffer+33))<<8) + (*(Ros_RxBuffer+34)))))/100;
		_receive.vel_des.z = ((float)((s32)(((*(Ros_RxBuffer+35))<<24) + ((*(Ros_RxBuffer+36))<<16) + ((*(Ros_RxBuffer+37))<<8) + (*(Ros_RxBuffer+38)))))/100;
			
		_receive.vel_fb.x = ((float)((s32)(((*(Ros_RxBuffer+39))<<24) + ((*(Ros_RxBuffer+40))<<16) + ((*(Ros_RxBuffer+41))<<8) + (*(Ros_RxBuffer+42)))))/100;
		_receive.vel_fb.y = ((float)((s32)(((*(Ros_RxBuffer+43))<<24) + ((*(Ros_RxBuffer+44))<<16) + ((*(Ros_RxBuffer+45))<<8) + (*(Ros_RxBuffer+46)))))/100;
		_receive.vel_fb.z = ((float)((s32)(((*(Ros_RxBuffer+47))<<24) + ((*(Ros_RxBuffer+48))<<16) + ((*(Ros_RxBuffer+49))<<8) + (*(Ros_RxBuffer+50)))))/100;
			
		_receive.acc_des.x = ((float)((s32)(((*(Ros_RxBuffer+51))<<24) + ((*(Ros_RxBuffer+52))<<16) + ((*(Ros_RxBuffer+53))<<8) + (*(Ros_RxBuffer+54)))))/100;
		_receive.acc_des.y = ((float)((s32)(((*(Ros_RxBuffer+55))<<24) + ((*(Ros_RxBuffer+56))<<16) + ((*(Ros_RxBuffer+57))<<8) + (*(Ros_RxBuffer+58)))))/100;
		_receive.acc_des.z = ((float)((s32)(((*(Ros_RxBuffer+59))<<24) + ((*(Ros_RxBuffer+60))<<16) + ((*(Ros_RxBuffer+61))<<8) + (*(Ros_RxBuffer+62)))))/100;
	
		ros_check_cnt = 0;
	}
	
	//����ָ������û���յ�ros���ź�
	if(ros_check_cnt < 1000)
	{
		ros_check_cnt++;
		_receive.online = 1;
	}
	else 
	{
		_receive.online = 0;
	}
}

/**********************************************************************************************************
*�� �� ��: GetRosPosDes
*����˵��: ���Ros�ṩ������λ����Ϣ
*��    ��: void
*�� �� ֵ: λ������
**********************************************************************************************************/
Vector3f_t GetRosPosDes(void)
{
	return _receive.pos_des;
}

/**********************************************************************************************************
*�� �� ��: GetRosPosFb
*����˵��: ���Ros�ṩ�ĵ�ǰλ����Ϣ
*��    ��: void
*�� �� ֵ: λ������
**********************************************************************************************************/
Vector3f_t GetRosPosFb(void)
{
	return _receive.pos_fb;
}

/**********************************************************************************************************
*�� �� ��: GetRosVelDes
*����˵��: ���Ros�ṩ�������ٶ���Ϣ
*��    ��: void
*�� �� ֵ: �ٶ�����
**********************************************************************************************************/
Vector3f_t GetRosVelDes(void)
{
	return _receive.vel_des;
}

/**********************************************************************************************************
*�� �� ��: GetRosVelFb
*����˵��: ���Ros�ṩ�ĵ�ǰ�ٶ���Ϣ
*��    ��: void
*�� �� ֵ: �ٶ�����
**********************************************************************************************************/
Vector3f_t GetRosVelFb(void)
{
	return _receive.vel_fb;
}

/**********************************************************************************************************
*�� �� ��: GetRosAccDes
*����˵��: ���Ros�ṩ���������ٶ���Ϣ
*��    ��: void
*�� �� ֵ: ���ٶ�����
**********************************************************************************************************/
Vector3f_t GetRosAccDes(void)
{
	return _receive.acc_des;
}


//////////////////////////////////////////////////////////////////////////////////////
//STM32�˷��ʹ����źŵ�Ros��
uint8_t Buf[64];
float Data[14];
/**********************************************************************************************************
*�� �� ��: Send_to_Ros
*����˵��: ����������ݵ�Ros�ˣ�ͨ��usart3
*��    ��: void
*�� �� ֵ: void
**********************************************************************************************************/
void Send_to_Ros(void)
{
	static uint16_t loop_cnt=0;
	if(loop_cnt++%10==0)
	{
		uint8_t _cnt = 0;
	
		Data[0] = _state.q_des[0];
		Data[1] = _state.q_des[1];
		Data[2] = _state.q_des[2];
		Data[3] = _state.q_des[3];
		
		Data[4] = _state.q_fb[0];
		Data[5] = _state.q_fb[1];
		Data[6] = _state.q_fb[2];
		Data[7] = _state.q_fb[3];
		
		Data[8] = _state.W_des.x;
		Data[9] = _state.W_des.y;
		Data[10] = _state.W_des.z;
		
		Data[11] = _state.W_fb.x;
		Data[12] = _state.W_fb.y;
		Data[13] = _state.W_fb.z;
		
		// ������buf��װ���ַ���
		Buf[_cnt++] = 0xAA;
		Buf[_cnt++] = MYHWADDR;	//���ݷ��Ͷ�ʶ����
		Buf[_cnt++] = 56;

		uint8_t check_sum = Buf[0]+Buf[1]+Buf[2];
		for(int i=0; i<14; i++)
		{
			int32_t Datai = Data[i]*10000;
			
			Buf[3+i*4] = BYTE3(Datai);
			Buf[4+i*4] = BYTE2(Datai);
			Buf[5+i*4] = BYTE1(Datai);
			Buf[6+i*4] = BYTE0(Datai);
			check_sum += (Buf[3+i*4]+Buf[4+i*4]+Buf[5+i*4]+Buf[6+i*4]);
		}
		
		Buf[59] = check_sum;	//һ֡��������60���ֽ�

		Buf[60] = 0x0d;		// \r
		Buf[61] = 0x0a;		// \n
		Usart3_Send(Buf, 62);	//����
	}
	
}

