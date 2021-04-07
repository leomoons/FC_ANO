#ifndef __NDOB_H
#define __NDOB_H
#include "mathConfig.h"
#include "disturbanceEst.h"

typedef struct
{
	// Coefficient
	float KoP[9];
	float KoR[9];
	
	Vector3f_t zP;
	Vector3f_t zR;
	
	Vector3f_t zP_dot;
	Vector3f_t zR_dot;
	
	Vector3f_t F_I;
	Vector3f_t M_b;
}nDOB_t;
extern nDOB_t _dob;

void ndobInit(void);

void ndobUpdate(float dT_s);
void ndobOutput(void);

void ndobZero(void);
#endif
