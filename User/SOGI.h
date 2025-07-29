#ifndef __SOGI__H__
#define __SOGI__H__

#include <arm_math.h>

typedef struct
{
	float v_alpha;
	float v_beta;
	float k;
	float w0;
	float T;//定时器的周期

	float q;
	float d;
	float w1;
	float theta;
	
	float kp;
	float ki;
	float err;
	float err_prev;
} SOGI_TypeDef;

extern SOGI_TypeDef sogi;

void SOGI_Init(SOGI_TypeDef *sogi, float w0, float T, float w1);
void SOGI_Update(SOGI_TypeDef *sogi, float v_in);
void SOGI_PLL(SOGI_TypeDef *sogi);

#endif