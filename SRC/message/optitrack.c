/**********************************************************************************************************
 * @�ļ�     optitrack.c
 * @˵��     ����usart����
 * @�汾  	 V1.0
 * @����     Leomoon
 * @����     2020.10
**********************************************************************************************************/
#include "optitrack.h"


Optitrack_t _opti;


/**********************************************************************************************************
*�� �� ��: Opti_Get_Byte
*����˵��: һ֡����ǰ����ֽ�����У��Ͷ��⹦�ܣ���¼λ�����ݳ��ȣ���
*         ���һλ��ǰ�������ֽ����ݼӺ�����У��
*��    ��: �ֽ�
*�� �� ֵ: void
**********************************************************************************************************/
//Opti_data_len��¼һ֡���ݵ������ֽڳ��ȣ�������У���õ������ֽ�
u8 Opti_RxBuffer[256], Opti_data_len = 0, Opti_Data_OK;
void Opti_Get_Byte(uint8_t byte)
{
	static u8 _data_len = 0;
	static u8 _sta = 0;
	static u8 _rx_buf[256];
	static u8 _rx_buf_len = 0;
	
	if(_sta==0 && byte==0xAA)	//֡ͷ0xAA
	{
		_sta = 1;
		_rx_buf[0] = byte;
	}
	else if(_sta==1 && byte==0x11)	//����Դ��0x11��ʾ��������OptiTrack
	{
		_sta = 2;
		_rx_buf[1] = byte;
	}
	else if(_sta==2)	//����Ŀ�ĵ�
	{
		_sta = 3;
		_rx_buf[2] = byte;
	}
	else if(_sta==3)	//������
	{
		_sta = 4;
		_rx_buf[3] = byte;
	}
	else if(_sta==4)	//���ݳ���
	{
		_sta = 5;
		_rx_buf[4] = byte;
		_data_len = byte;
		_rx_buf_len = 5;
	}
	else if(_sta==5 && _data_len>0)
	{
		_data_len--;
		_rx_buf[_rx_buf_len++] = byte;
		if(_data_len==0)   _sta = 6;
	}
	else if(_sta==6)
	{
		_sta = 0;
		_rx_buf[_rx_buf_len] = byte;
		if(!Opti_Data_OK)
		{
			for(u8 i=0; i<=_rx_buf_len; i++)	Opti_RxBuffer[i] = _rx_buf[i];
			Opti_data_len = _rx_buf_len+1;
			Opti_Data_OK = 1;
		}
	}
	else
		_sta = 0;
}
	


/**********************************************************************************************************
*�� �� ��: Opti_Get_Data_Task
*����˵��: Optitrack���ݽ����ܺ���
*��    ��: void
*�� �� ֵ: void
**********************************************************************************************************/
void Opti_Get_Data_Task(void)
{
	static u16 opti_check_cnt;
	
	if(Opti_Data_OK)
	{
		Opti_Data_OK = 0;
		u8 check_sum = 0;
		for(u8 i=0; i<(Opti_data_len-1); i++)
		{
			check_sum += *(Opti_RxBuffer+i);
		}
		if(!(check_sum == *(Opti_RxBuffer+Opti_data_len-1)))	return;	//���һ���ֽ����ڼӺ�У��
		
		_opti.pos.x = ((float)((s32)(((*(Opti_RxBuffer+5))<<24) + ((*(Opti_RxBuffer+6))<<16) + ((*(Opti_RxBuffer+7))<<8) + (*(Opti_RxBuffer+8)))))/100;
		_opti.pos.y = ((float)((s32)(((*(Opti_RxBuffer+9))<<24) + ((*(Opti_RxBuffer+10))<<16) + ((*(Opti_RxBuffer+11))<<8) + (*(Opti_RxBuffer+12)))))/100;
		_opti.pos.z = ((float)((s32)(((*(Opti_RxBuffer+13))<<24) + ((*(Opti_RxBuffer+14))<<16) + ((*(Opti_RxBuffer+15))<<8) + (*(Opti_RxBuffer+16)))))/100;
		_opti.quat[0] = ((float)((s32)(((*(Opti_RxBuffer+17))<<24) + ((*(Opti_RxBuffer+18))<<16) + ((*(Opti_RxBuffer+19))<<8) + (*(Opti_RxBuffer+20)))))/100;
		_opti.quat[1] = ((float)((s32)(((*(Opti_RxBuffer+21))<<24) + ((*(Opti_RxBuffer+22))<<16) + ((*(Opti_RxBuffer+23))<<8) + (*(Opti_RxBuffer+24)))))/100;
		_opti.quat[2] = ((float)((s32)(((*(Opti_RxBuffer+25))<<24) + ((*(Opti_RxBuffer+26))<<16) + ((*(Opti_RxBuffer+27))<<8) + (*(Opti_RxBuffer+28)))))/100;
		_opti.quat[3] = ((float)((s32)(((*(Opti_RxBuffer+29))<<24) + ((*(Opti_RxBuffer+30))<<16) + ((*(Opti_RxBuffer+31))<<8) + (*(Opti_RxBuffer+32)))))/100;
		
		
		_opti.vel.x = ((float)((s32)(((*(Opti_RxBuffer+33))<<24) + ((*(Opti_RxBuffer+34))<<16) + ((*(Opti_RxBuffer+35))<<8) + (*(Opti_RxBuffer+36)))))/100;
		_opti.vel.y = ((float)((s32)(((*(Opti_RxBuffer+37))<<24) + ((*(Opti_RxBuffer+38))<<16) + ((*(Opti_RxBuffer+39))<<8) + (*(Opti_RxBuffer+40)))))/100;
		_opti.vel.z = ((float)((s32)(((*(Opti_RxBuffer+41))<<24) + ((*(Opti_RxBuffer+42))<<16) + ((*(Opti_RxBuffer+43))<<8) + (*(Opti_RxBuffer+44)))))/100;
		_opti.W.x = ((float)((s32)(((*(Opti_RxBuffer+45))<<24) + ((*(Opti_RxBuffer+46))<<16) + ((*(Opti_RxBuffer+47))<<8) + (*(Opti_RxBuffer+48)))))/100;
		_opti.W.y = ((float)((s32)(((*(Opti_RxBuffer+49))<<24) + ((*(Opti_RxBuffer+50))<<16) + ((*(Opti_RxBuffer+51))<<8) + (*(Opti_RxBuffer+52)))))/100;
		_opti.W.z = ((float)((s32)(((*(Opti_RxBuffer+53))<<24) + ((*(Opti_RxBuffer+54))<<16) + ((*(Opti_RxBuffer+55))<<8) + (*(Opti_RxBuffer+56)))))/100;
		
		_opti.acc.x = ((float)((s32)(((*(Opti_RxBuffer+57))<<24) + ((*(Opti_RxBuffer+58))<<16) + ((*(Opti_RxBuffer+59))<<8) + (*(Opti_RxBuffer+60)))))/100;
		_opti.acc.y = ((float)((s32)(((*(Opti_RxBuffer+61))<<24) + ((*(Opti_RxBuffer+62))<<16) + ((*(Opti_RxBuffer+63))<<8) + (*(Opti_RxBuffer+64)))))/100;
		_opti.acc.z = ((float)((s32)(((*(Opti_RxBuffer+65))<<24) + ((*(Opti_RxBuffer+66))<<16) + ((*(Opti_RxBuffer+67))<<8) + (*(Opti_RxBuffer+68)))))/100;
		_opti.W_dot.x = ((float)((s32)(((*(Opti_RxBuffer+69))<<24) + ((*(Opti_RxBuffer+70))<<16) + ((*(Opti_RxBuffer+71))<<8) + (*(Opti_RxBuffer+72)))))/100;
		_opti.W_dot.y = ((float)((s32)(((*(Opti_RxBuffer+73))<<24) + ((*(Opti_RxBuffer+74))<<16) + ((*(Opti_RxBuffer+75))<<8) + (*(Opti_RxBuffer+76)))))/100;
		_opti.W_dot.z = ((float)((s32)(((*(Opti_RxBuffer+77))<<24) + ((*(Opti_RxBuffer+78))<<16) + ((*(Opti_RxBuffer+79))<<8) + (*(Opti_RxBuffer+80)))))/100;
		
		opti_check_cnt = 0;
	}
	
	//����ָ������û���յ�optitrack���ź�
	if(opti_check_cnt < 1000)
	{
		opti_check_cnt++;
		_opti.online = 1;
	}
	else
	{
		_opti.online = 0;
	}
}

/**********************************************************************************************************
*�� �� ��: GetOptiPos
*����˵��: ���optitrack��λ����Ϣ
*��    ��: void
*�� �� ֵ: λ������
**********************************************************************************************************/
Vector3f_t GetOptiPos(void)
{
	return _opti.pos;
}

/**********************************************************************************************************
*�� �� ��: GetOptiVel
*����˵��: ���optitrack���ٶ���Ϣ
*��    ��: void
*�� �� ֵ: �ٶ�����
**********************************************************************************************************/
Vector3f_t GetOptiVel(void)
{
	return _opti.vel;
}

/**********************************************************************************************************
*�� �� ��: GetOptiAcc
*����˵��: ���optitrack�ļ��ٶ���Ϣ
*��    ��: void
*�� �� ֵ: ���ٶ�����
**********************************************************************************************************/
Vector3f_t GetOptiAcc(void)
{
	return _opti.acc;
}

/**********************************************************************************************************
*�� �� ��: GetOptiAttQuat
*����˵��: ���optitrack����̬��Ϣ(��Ԫ����ʾ)
*��    ��: void
*�� �� ֵ: ��Ԫ������
**********************************************************************************************************/
void GetOptiAttQuat(float* quat)
{
	quat[0] = _opti.quat[0];
	quat[1] = _opti.quat[1];
	quat[2] = _opti.quat[2];
	quat[3] = _opti.quat[3];
}

/**********************************************************************************************************
*�� �� ��: GetOptiAttDCM
*����˵��: ���optitrack����̬��Ϣ(��ת����)
*��    ��: void
*�� �� ֵ: ��ת��������ͷ
**********************************************************************************************************/
void GetOptiAttDCM(float* dcm)
{
	Quater_to_DCM(_opti.dcm, _opti.quat);
	Matrix3_Copy(_opti.dcm, dcm);
}

/**********************************************************************************************************
*�� �� ��: GetOptiAttEuler
*����˵��: ���optitrack����̬��Ϣ(ŷ���Ǳ�ʾ)
*��    ��: void
*�� �� ֵ: ŷ��������
**********************************************************************************************************/
Vector3f_t GetOptiAttEuler(void)
{
	_opti.euler = Quater_to_Euler(_opti.quat);
	return _opti.euler;
}

/**********************************************************************************************************
*�� �� ��: GetOptiAngVel
*����˵��: ���optitrack�Ľ��ٶ���Ϣ
*��    ��: void
*�� �� ֵ: ���ٶ�����
**********************************************************************************************************/
Vector3f_t GetOptiAngVel(void)
{
	return _opti.W;
}

/**********************************************************************************************************
*�� �� ��: GetOptiAngAcc
*����˵��: ���optitrack�ĽǼ��ٶ���Ϣ
*��    ��: void
*�� �� ֵ: �Ǽ��ٶ�����
**********************************************************************************************************/
Vector3f_t GetOptiAngAcc(void)
{
	return _opti.W_dot;
}

