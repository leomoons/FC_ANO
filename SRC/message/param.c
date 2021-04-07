/**********************************************************************************************************
 * @�ļ�     param.c
 * @˵��     ���ο��������Ŷ�����������
 * @�汾  	 V1.0
 * @����     Leomoon
 * @����     2020.12
**********************************************************************************************************/
#include "param.h"
#include "Drv_w25qxx.h"
#include "mathConfig.h"
#include "string.h"
#include "Drv_time.h"

union Parameter_u Param;

// ����д�������
static uint16_t param_save_cnt = 0;

/**********************************************************************************************************
*�� �� ��: ParamInit
*����˵��: ������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ParamInit(void)
{
    ParamReadFromFlash();
	
	//TODO: ���ò�����ȡ�����Ӧ״̬
}

/**********************************************************************************************************
*�� �� ��: ParamDataReset
*����˵��: �����ָ�Ĭ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void ParamDataReset(void)
{
	//PD����������
	Param.data[CONTROLLER_PD_Kp_X] = 1.0;
	Param.data[CONTROLLER_PD_Kp_Y] = 1.0;
	Param.data[CONTROLLER_PD_Kp_Z] = 1.0;
	
	Param.data[CONTROLLER_PD_Kv_X] = 1.0;
	Param.data[CONTROLLER_PD_Kv_Y] = 1.0;
	Param.data[CONTROLLER_PD_Kv_Z] = 1.0;
	
	Param.data[CONTROLLER_PD_KR_X] = 0.3;
	Param.data[CONTROLLER_PD_KR_Y] = 0.3;
	Param.data[CONTROLLER_PD_KR_Z] = 0.2;
	
	Param.data[CONTROLLER_PD_KW_X] = 0.05;
	Param.data[CONTROLLER_PD_KW_Y] = 0.05;
	Param.data[CONTROLLER_PD_KW_Z] = 0.04;
	// SMC����������
	Param.data[CONTROLLER_SMC_csP_X] = 1.0;
	Param.data[CONTROLLER_SMC_csP_Y] = 1.0;
	Param.data[CONTROLLER_SMC_csP_Z] = 1.0;
	
	Param.data[CONTROLLER_SMC_HsP_X] = 0.1;
	Param.data[CONTROLLER_SMC_HsP_Y] = 0.1;
	Param.data[CONTROLLER_SMC_HsP_Z] = 0.1;
	
	Param.data[CONTROLLER_SMC_ksP_X] = 1.0;
	Param.data[CONTROLLER_SMC_ksP_Y] = 1.0;
	Param.data[CONTROLLER_SMC_ksP_Z] = 1.0;
	
	Param.data[CONTROLLER_SMC_csR_X] = 6.0;	//  =ksR/KR
	Param.data[CONTROLLER_SMC_csR_Y] = 6.0;
	Param.data[CONTROLLER_SMC_csR_Z] = 6.0;

	Param.data[CONTROLLER_SMC_HsR_X] = 0.1;
	Param.data[CONTROLLER_SMC_HsR_Y] = 0.1;
	Param.data[CONTROLLER_SMC_HsR_Z] = 0.1;
	
	Param.data[CONTROLLER_SMC_ksR_X] = 0.2;
	Param.data[CONTROLLER_SMC_ksR_Y] = 0.2;
	Param.data[CONTROLLER_SMC_ksR_Z] = 0.2;
	
	param_save_cnt = 1;
}

/**********************************************************************************************************
*�� �� ��: ParamBufferReset
*����˵��: �����洢buffer�ָ�Ĭ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ParamBufferReset(void)
{	
	//�����ָ�Ĭ��ֵ
	ParamDataReset();
	
	param_save_cnt = 1;
}


/**********************************************************************************************************
*�� �� ��: ParamReadFromFlash
*����˵��: �ѷɿز����洢�������ݶ�ȡ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ParamReadFromFlash(void)
{
	float dataSum=0, checkNum=0, checkSum=0;
	
	Flash_PageRead(PARAM_START_ADDR, Param.buffer, PARAM_NUM*4);	//��ȡָ�����ȵ��ֽ�
	
	ParamGetData(PARAM_CHECK_NUM, &checkNum, 4);
	checkNum = ConstrainInt32(checkNum, 0, PARAM_NUM);
	ParamGetData(PARAM_CHECK_SUM, &checkSum, 4);
	
	//���������
	for(u8 i=8; i<(u8)(checkNum*4); i++)
    {
        dataSum += (float)Param.buffer[i];
    }
	
	//�ͱ����У��ͽ��жԱȣ�������������������в���
    if(checkSum != dataSum)
    {
        ParamBufferReset();
    }	
	
}

/**********************************************************************************************************
*�� �� ��: ParamSaveToFlash
*����˵��: �ѷɿز���д��洢�� ����Ƶ��20Hz
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//u8 data[PARAM_NUM*4];
void ParamSaveToFlash(void)
{
	uint32_t i=0;
	float dataSum = 0.0;
	float dataNum = 0.0;
	
	//TODO:�����󲻵ñ������
	
	if(param_save_cnt == 1)
	{
		//�����������
		dataNum = PARAM_NUM;
		memcpy(Param.buffer+PARAM_CHECK_NUM*4, &dataNum, 4);//ע����С������
//		u8 tmpByte = Param.buffer[0];
//		Param.buffer[0] = Param.buffer[3];
//		Param.buffer[3] = tmpByte;
//		tmpByte = Param.buffer[1];
//		Param.buffer[1] = Param.buffer[2];
//		Param.buffer[2] = tmpByte;
		
		//��������Ͳ�����
		for(i=8; i<PARAM_NUM*4; i++)
		{
			dataSum += (float)Param.buffer[i];
		}
		memcpy(Param.buffer+PARAM_CHECK_SUM*4, &dataSum, 4);
		Flash_SectorErase(PARAM_START_ADDR, 1);//������ÿ����д������Ҫ��ʱһ��ʱ�䣬���򲻻�ɹ�
		Delay_ms(200);
		//W25QXX_PageRead(data, PARAM_START_ADDR, PARAM_NUM*4);
		//DelayMs(200);
		Flash_PageWrite(PARAM_START_ADDR, Param.buffer, PARAM_NUM*4);
		//DelayMs(200);
		//W25QXX_PageRead(data, PARAM_START_ADDR, PARAM_NUM*4);
		//DelayMs(200);
	}
	
	if(param_save_cnt > 0)
		param_save_cnt--;
}

/**********************************************************************************************************
*�� �� ��: ParamUpdateData
*����˵��: ���·ɿز���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ParamUpdateData(uint16_t dataNum, const void *data)
{
    memcpy(Param.buffer+dataNum*4, data, 4);
    //�������µ�3���ˢ��һ��Flash
    param_save_cnt = 60;
}

/**********************************************************************************************************
*�� �� ��: ParamGetData
*����˵��: ��ȡ�ɿز���
*��    ��: dataNum: �����б���ĵڼ�������
*          void data: pointer to the buffer that receives the data��ʹ��void���Ա������ͳ�ͻ
*		   length: ����������ռ�ݵ��ֽ���Ŀ
*�� �� ֵ: ��
**********************************************************************************************************/
void ParamGetData(uint16_t dataNum, void *data, uint8_t length)
{
	memcpy(data, Param.buffer+dataNum*4, length);
}

/**********************************************************************************************************
*�� �� ��: Param_save_cnt_tox
*����˵��: �ṩ���ⲿ�ĵ���param_save_cntֵ�ĺ����ӿ�
*��    ��: Ҫ����ֵ
*�� �� ֵ: ��
**********************************************************************************************************/
void Param_save_cnt_tox(uint8_t cnt)
{
	param_save_cnt = 1;
}
