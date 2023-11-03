#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct
{
	fp32 Kp,Ki,Kd;
	fp32 max_out;
	fp32 max_iout;
	fp32 set,fdb;
	fp32 Pout,Iout,Dout,out;
	fp32 Dbuf[3];
	fp32 error[3];
}PID_Typedef_t;

void PID_Init(PID_Typedef_t *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_Calc(PID_Typedef_t *pid, fp32 ref, fp32 set);
void PID_Clear(PID_Typedef_t *pid);
#endif