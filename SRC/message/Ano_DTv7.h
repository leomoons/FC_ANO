#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "stm32f4xx.h"

#define USE_HID			0x01
#define USE_U1			0x02
#define USE_U2			0x04

#define FRAME_HEAD		0xAA

#define LOG_COLOR_BLACK	0
#define LOG_COLOR_RED	  1
#define LOG_COLOR_GREEN	2

typedef enum
{
	CSID_XF1,
	CSID_XF2,
	CSID_XF3,
	CSID_XF4,
	CSID_XF5,
  CSID_XF6,
	CSID_XFA,
	CSID_NUM
} _enu_cyclesendid;

void AnoDTRxOneByte(uint8_t type,uint8_t data);
void AnoDTRxOneByteUart( uint8_t data );
void AnoDTRxOneByteUsb( uint8_t data );
void AnoDTSendStr(uint8_t type, uint8_t dest_addr, uint8_t string_color,char *str);
void ANO_DT_Init(void);
void ANO_DT_TaskMs(void);

#endif

