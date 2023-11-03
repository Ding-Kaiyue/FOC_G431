#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "Bsp_Motor.h"

enum {
	SECTOR1        = 1,
	SECTOR2        = 2,
    SECTOR3        = 3,
    SECTOR4        = 4,
    SECTOR5        = 5,
    SECTOR6        = 6,
};

typedef struct
{
	fp32 Ialpha,Ibeta;
	fp32 Id,Iq;
	fp32 Id_set,Iq_set;
}Curr_Components_t;		//电流值结构体

typedef struct
{
	fp32 Vd,Vq;
	fp32 Valpha,Vbeta;
}Volt_Components;		//电压值结构体

typedef struct
{
	fp32 Tcm1,Tcm2,Tcm3;
}Pwm_Components;		//SVPWM结构体

typedef struct
{
	Curr_Components_t Curr_Components;
	Volt_Components Volt_Components;
	Pwm_Components Pwm_Components;
}FOC_Typedef_t;			//FOC结构体


void Over_Modulation(uint16_t time1, uint16_t time2);
void Vdq_Normalization(FOC_Typedef_t *foc);
void Clarke_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg);
void Park_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg);
void PID_Current_Loop_Calc(FOC_Typedef_t *foc);
void InvPark_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg);
void SVPWM_Calc(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg);

#endif