/**********************************************************************************************************
 * @�ļ�     lyhDecode.c
 * @˵��     �򵥵���Ϣ���벢ʵ����Ӧ�����޸Ĺ���
 * @�汾  	 V1.0
 * @����     Leomoon
 * @����     2020.09
**********************************************************************************************************/
#include "lyhDecode.h"  
#include "param.h"

#include "pd.h"
#include "smc.h"

u8 LYH_RxBuffer[10];
u8 LYH_CMD_OK=0;
u8 LYH_PD_OK=0;
u8 LYH_SMC_OK=0;


/**********************************************************************************************************
*�� �� ��: LYH_Data_Receive_Prepare
*����˵��: ������Ϣ����usart�жϵ���ÿ�ν���һ���ֽ�
*��    ��: data��һ���ֽ�
*�� �� ֵ: ��
**********************************************************************************************************/
void LYH_Data_Receive_Prepare(u8 data)
{
	static u8 cmd_state=0, PD_state = 0, SMC_state=0;
	
	//У׼ָ�����
	if(cmd_state==0 && data==0x46)	//֡ͷ0x46('F')
	{
		cmd_state=1;
		LYH_RxBuffer[0]=data;
	}
	else if(cmd_state==1 && data==0x75)	//�ڶ�֡0x75('u')
	{
		cmd_state=2;
		LYH_RxBuffer[1]=data;
	}
	else if(cmd_state==2 && data==0x63)		//����֡0x63('c')
	{
		cmd_state=3;
		LYH_RxBuffer[2]=data;
	}
	else if(cmd_state==3 && data==0x6B)		//����֡0x6B('k')
	{
		cmd_state=4;
		LYH_RxBuffer[3]=data;
	}
	else if(cmd_state==4)		//����ѡ��
	{
		cmd_state = 5;
		LYH_RxBuffer[4]=data;
		LYH_CMD_OK = 1;
	}
	else
		cmd_state = 0;
	
	// �޸�PD����������ָ�����
	if(PD_state==0 && data==0x50)	//֡ͷ0x50('P')
	{
		PD_state=1;
		LYH_RxBuffer[0] = data;
	}
	//ѡ��p:0x70 v:0x76 R:0x52 W:0x57
	else if(PD_state==1 && (data==0x70 || data==0x76 || data==0x52 || data==0x57))	
	{
		PD_state=2;
		LYH_RxBuffer[1] = data;
	}	
	//ѡ���������е�һ��
	else if(PD_state==2 && (data==0x78 || data==0x79 || data==0x7A))
	{
		PD_state=3;
		LYH_RxBuffer[2] = data;
	}
	//��������buffer
	else if(PD_state>=3 && LYH_RxBuffer[0] == 0x50)
	{
		LYH_RxBuffer[PD_state++] = data;
		if(PD_state == 7)
		{
			PD_state=0;
			LYH_PD_OK = 1;
		}
	}
	else 
		PD_state=0;
	
	// �޸�SMC�������Ŀ��Ʋ���
	if(SMC_state==0 && data==0x53)	// ֡ͷ0x53('S')
	{
		SMC_state=1;
		LYH_RxBuffer[0] = data;
	}
	//ѡ�� '1'(0x31):csP,'2'(0x32):HsP,'3'(0x33):ksP,'4'(0x34):csR,'5'(0x35):HsR,'6'(0x36):ksR
	else if(SMC_state==1 && (data==0x31||data==0x32||data==0x33||data==0x34||data==0x35||data==0x36))
	{
		SMC_state=2;
		LYH_RxBuffer[1] = data;
	}
	// ѡ���������е�һ��
	else if(SMC_state==2 && (data==0x78 || data==0x79 || data==0x7A))
	{
		SMC_state=3;
		LYH_RxBuffer[2] = data;
	}
	// ��������Buffer
	else if(SMC_state>=3 && LYH_RxBuffer[0]==0x53)
	{
		LYH_RxBuffer[SMC_state++] = data;
		if(SMC_state == 7)
		{
			SMC_state=0;
			LYH_SMC_OK=1;
		}
	}
	else SMC_state=0;
}

/**********************************************************************************************************
*�� �� ��: LYH_Receive_Loop
*����˵��: ������Ϣ�������Ӧ����������Ƶ��1000Hz
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void LYH_Receive_Loop(void)
{
	
	if(LYH_CMD_OK)
	{
		LYH_CMD_OK = 0;
		
		//����ѡ��:  'r':����reset
		switch(LYH_RxBuffer[4])			
		{
			
			case 'r':
				ParamBufferReset();
				pdCtrlInit();
				smcCtrlInit();
			break;
			
			default:
				break;	
		}
	}
	
	// PD�����������޸� 
	if(LYH_PD_OK)
	{
		LYH_PD_OK = 0;
		
		float paramData;
		uint8_t* pp = (uint8_t*)&paramData;
		pp[0] = LYH_RxBuffer[6];
		pp[1] = LYH_RxBuffer[5];
		pp[2] = LYH_RxBuffer[4];
		pp[3] = LYH_RxBuffer[3];
		
		int paramSelect = 0;
		
		// PD�����������޸�ѡ��
		// 'p':λ�û��ڲ����� 'v'���ٶȻ��ڲ����� 'R'����̬���ڲ����� 'W'�����ٶȻ��ڲ���
		switch(LYH_RxBuffer[1])
		{
			case 'p':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_PD_Kp_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_PD_Kp_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_PD_Kp_Z;
					break;
				}
			break;
			
			case 'v':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_PD_Kv_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_PD_Kv_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_PD_Kv_Z;
					break;
				}
			break;	
				
			case 'R':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_PD_KR_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_PD_KR_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_PD_KR_Z;
					break;
				}
			break;
						
			case 'W':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_PD_KW_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_PD_KW_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_PD_KW_Z;
					break;
				}
			break;						
		}
		pdCtrlUpdateParam(paramSelect, paramData);
	} 
	// SMC�����������޸�
	if(LYH_SMC_OK)
	{
		LYH_SMC_OK = 0;
		
		float paramData;
		uint8_t* pp = (uint8_t*)&paramData;
		pp[0] = LYH_RxBuffer[6];
		pp[1] = LYH_RxBuffer[5];
		pp[2] = LYH_RxBuffer[4];
		pp[3] = LYH_RxBuffer[3];
		
		int paramSelect = 0;
		
		// SMC�����������޸�ѡ��
		//'1':csP,'2':HsP,'3':ksP,'4':csR,'5':HsR,'6':ksR
		switch(LYH_RxBuffer[1])
		{
			case '1':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_SMC_csP_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_SMC_csP_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_SMC_csP_Z;
					break;
				}
			break;
				
			case '2':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_SMC_HsP_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_SMC_HsP_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_SMC_HsP_Z;
					break;
				}
			break;
				
			case '3':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_SMC_ksP_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_SMC_ksP_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_SMC_ksP_Z;
					break;
				}
			break;
				
			case '4':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_SMC_csR_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_SMC_csR_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_SMC_csR_Z;
					break;
				}
			break;
				
			case '5':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_SMC_HsR_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_SMC_HsR_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_SMC_HsR_Z;
					break;
				}
			break;
				
			case '6':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_SMC_ksR_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_SMC_ksR_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_SMC_ksR_Z;
					break;
				}
			break;
		}
		
		smcCtrlUpdateParam(paramSelect, paramData);
	}
}


