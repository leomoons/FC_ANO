/**********************************************************************************************************
 * @文件     lyhDecode.c
 * @说明     简单的信息解码并实现相应参数修改功能
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
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
*函 数 名: LYH_Data_Receive_Prepare
*功能说明: 解码信息，被usart中断调用每次接收一个字节
*形    参: data，一个字节
*返 回 值: 无
**********************************************************************************************************/
void LYH_Data_Receive_Prepare(u8 data)
{
	static u8 cmd_state=0, PD_state = 0, SMC_state=0;
	
	//校准指令解析
	if(cmd_state==0 && data==0x46)	//帧头0x46('F')
	{
		cmd_state=1;
		LYH_RxBuffer[0]=data;
	}
	else if(cmd_state==1 && data==0x75)	//第二帧0x75('u')
	{
		cmd_state=2;
		LYH_RxBuffer[1]=data;
	}
	else if(cmd_state==2 && data==0x63)		//第三帧0x63('c')
	{
		cmd_state=3;
		LYH_RxBuffer[2]=data;
	}
	else if(cmd_state==3 && data==0x6B)		//第四帧0x6B('k')
	{
		cmd_state=4;
		LYH_RxBuffer[3]=data;
	}
	else if(cmd_state==4)		//命令选择，
	{
		cmd_state = 5;
		LYH_RxBuffer[4]=data;
		LYH_CMD_OK = 1;
	}
	else
		cmd_state = 0;
	
	// 修改PD控制器参数指令解析
	if(PD_state==0 && data==0x50)	//帧头0x50('P')
	{
		PD_state=1;
		LYH_RxBuffer[0] = data;
	}
	//选择p:0x70 v:0x76 R:0x52 W:0x57
	else if(PD_state==1 && (data==0x70 || data==0x76 || data==0x52 || data==0x57))	
	{
		PD_state=2;
		LYH_RxBuffer[1] = data;
	}	
	//选择三个轴中的一个
	else if(PD_state==2 && (data==0x78 || data==0x79 || data==0x7A))
	{
		PD_state=3;
		LYH_RxBuffer[2] = data;
	}
	//参数载入buffer
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
	
	// 修改SMC控制器的控制参数
	if(SMC_state==0 && data==0x53)	// 帧头0x53('S')
	{
		SMC_state=1;
		LYH_RxBuffer[0] = data;
	}
	//选择 '1'(0x31):csP,'2'(0x32):HsP,'3'(0x33):ksP,'4'(0x34):csR,'5'(0x35):HsR,'6'(0x36):ksR
	else if(SMC_state==1 && (data==0x31||data==0x32||data==0x33||data==0x34||data==0x35||data==0x36))
	{
		SMC_state=2;
		LYH_RxBuffer[1] = data;
	}
	// 选择三个轴中的一个
	else if(SMC_state==2 && (data==0x78 || data==0x79 || data==0x7A))
	{
		SMC_state=3;
		LYH_RxBuffer[2] = data;
	}
	// 参数载入Buffer
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
*函 数 名: LYH_Receive_Loop
*功能说明: 解码信息并完成相应操作，调用频率1000Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void LYH_Receive_Loop(void)
{
	
	if(LYH_CMD_OK)
	{
		LYH_CMD_OK = 0;
		
		//命令选择:  'r':参数reset
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
	
	// PD控制器参数修改 
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
		
		// PD控制器参数修改选择：
		// 'p':位置环节参数， 'v'：速度环节参数， 'R'：姿态环节参数， 'W'：角速度环节参数
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
	// SMC控制器参数修改
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
		
		// SMC控制器参数修改选择：
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


