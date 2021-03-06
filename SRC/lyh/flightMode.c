/**********************************************************************************************************
 * @文件     flightMode.c
 * @说明     根据段位开关选择飞行模式
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.12
**********************************************************************************************************/
#include "flightMode.h"

uint8_t FLYMODE = 0;

/**********************************************************************************************************
*函 数 名: SetFlightMode
*功能说明: 根据遥控器的信号切换飞行模式
*形    参: 模态
*返 回 值: 无
**********************************************************************************************************/
void SetFlightMode(uint8_t mode)
{
	FLYMODE = mode;
}

/**********************************************************************************************************
*函 数 名: GetCaliStatus
*功能说明: 获取传感器校准状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetFlightMode(void)
{
    return FLYMODE;
}

