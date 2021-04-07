#ifndef __FLIGHTMODE_H
#define __FLIGHTMODE_H
#include "stm32f4xx.h"

// flight mode
enum
{
	MANUAL = 0,
	MISSION,
	OBSERVER,	//ÓÐÈÅ¶¯¹Û²âÆ÷
	FAILSAFE
};

void SetFlightMode(uint8_t status);
uint8_t GetFlightMode(void);


#endif
