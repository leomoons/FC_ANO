/**********************************************************************************************************
 * @�ļ�     motor.c
 * @˵��     �����������
 * @�汾  	 V1.0
 * @����     Leomoon
 * @����     2020.12
**********************************************************************************************************/
#include "motor.h"
#include "Drv_time.h"
#include "controller.h"
#include "flightMode.h"

/**********************************************************************************************************
*�� �� ��: MotorInit
*����˵��: ������Ƴ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void MotorInit(void)
{
	// Init ESC
	int PWM = 5940;
	TIM1->CCR1 = PWM;			
	TIM1->CCR2 = PWM;			
	TIM1->CCR3 = PWM;			
	TIM1->CCR4 = PWM;			
	TIM5->CCR3 = PWM;			
	TIM5->CCR4 = PWM;
	
	Delay_ms(200);
}

/**********************************************************************************************************
*�� �� ��: MotorCtrlTask
*����˵��: ������ƣ��ı�PWMֵ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
int32_t PWM_diff[6], PWM_cur[6], PWM_former[6];
void MotorCtrlTask(void)
{
	float motor[6];
	Matrix6MulVector6(_veh.Binv, _ctrl.wrench, motor);
	//static int32_t PWM_diff[6], PWM_cur[6], PWM_former[6];
	
	if(GetFlightMode() == FAILSAFE)
	{
		int PWM = 5940;
		TIM1->CCR1 = PWM;			
		TIM1->CCR2 = PWM;			
		TIM1->CCR3 = PWM;			
		TIM1->CCR4 = PWM;			
		TIM5->CCR3 = PWM;			
		TIM5->CCR4 = PWM;
	}
	else
	{
		for(uint8_t i=0; i<6; i++)
		{
			int32_t PWM_ = (int32_t)(motor[i] / _veh.T);
//			LowPassFilter1stInt( PWM_diff+i, PWM_, 0.08f);
//			if(PWM_diff[i]>1200)	PWM_cur[i] = 7300;
//			else if(PWM_diff[i]>0 && PWM_diff[i]<=1200)	PWM_cur[i] = 6100+PWM_diff[i];
//			else if(PWM_diff[i]==0) PWM_cur[i] = 5900;
//			else if(PWM_diff[i]<0 && PWM_diff[i]>=-1200) PWM_cur[i] = 5700+PWM_diff[i];
//			else if(PWM_diff[i]<-1200)	PWM_cur[i] = 4500;
			
			if(PWM_>1200)	PWM_cur[i] = 7300;
			else if(PWM_>0 && PWM_<=1200)	PWM_cur[i] = 6100+PWM_;
			else if(PWM_==0) PWM_cur[i] = 5900;
			else if(PWM_<0 && PWM_>=-1200) PWM_cur[i] = 5700+PWM_;
			else if(PWM_<-1200)	PWM_cur[i] = 4500;
			
			//��ֹ��֮֡��仯̫��
//			int tmp = PWM_former[i]-PWM_cur[i];	
//			if(tmp>50)	PWM_cur[i] = PWM_former[i]-50;
//			else if(tmp<-50) PWM_cur[i] = PWM_former[i]+50;
			
			PWM_former[i] = PWM_cur[i];
			
			switch (i)
			{
				case 0:
					TIM1->CCR4 = PWM_cur[i];
					break;
				case 1:
					TIM1->CCR3 = PWM_cur[i];
					break;
				case 2:
					TIM1->CCR2 = PWM_cur[i];
					break;
				case 3:
					TIM1->CCR1 = PWM_cur[i];
					break;
				case 4: 
					TIM5->CCR4 = PWM_cur[i];
					break;
				case 5:
					TIM5->CCR3 = PWM_cur[i];
					break;	
			}
			
		}
	}
}
