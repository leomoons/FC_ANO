#ifndef __LYHDECODE_H
#define __LYHDECODE_H
#include "stm32f4xx.h"

extern uint8_t LYH_RcBuffer[10], LYH_data_ok;

void LYH_Data_Receive_Prepare(uint8_t data);

void LYH_Receive_Loop(void);

#endif
