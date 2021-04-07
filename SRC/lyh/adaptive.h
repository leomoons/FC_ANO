#ifndef __ADAPTIVE_H
#define __ADAPTIVE_H
#include "mathConfig.h"
#include "disturbanceEst.h"

typedef struct
{
	// coefficient
	float gamax[9];
	float gamaR[9];
	float Wx[9];
	float Wx_T[9];
	float WR[9];
	float WR_T[9];
	float cax[9];
	float caR[9];
	float Bx;
	float BR;
	
	
	Vector3f_t F_I;
	Vector3f_t M_b;
	
	
	Vector3f_t dx;
	Vector3f_t dR;
	Vector3f_t dx_dot;
	Vector3f_t dR_dot;
	
}adaptive_t;
extern adaptive_t _ada;

void adaptiveInit(void);

void adaptiveOutput(void);
void adaptiveUpdate(float dT_s);

void adaptiveZero(void);

#endif
