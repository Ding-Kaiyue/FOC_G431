#include "PID.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
		
void PID_Init(PID_Typedef_t *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
	if (pid == NULL || PID == NULL)
	{
		return;
	}
	pid->Kp = PID[0];
	pid->Ki = PID[1];
	pid->Kd = PID[2];
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_Calc(PID_Typedef_t *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	pid->error[0] = set - ref;
	
	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = pid->error[0] - pid->error[1];
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout,pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out,pid->max_out);
	return pid->out;
}

void PID_Reset(PID_Typedef_t *pid,fp32 kp,fp32 ki,fp32 kd)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

void PID_Clear(PID_Typedef_t *pid)
{
	if(pid == NULL)
	{
		return;
	}
	pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
	pid->fdb = pid->set = 0.0f;
}