#ifndef __INIT_TASK_H
#define __INIT_TASK_H

#include "FOC.h"
#include "Bsp_Motor.h"
#include "main.h"

#define CURRENT_LOOP_IQ_KP 0.0001f
#define CURRENT_LOOP_IQ_KI 0.00000000000f
#define CURRENT_LOOP_IQ_KD 0.00000000000f
#define CURRENT_LOOP_IQ_MAX_OUT 1.0f
#define CURRENT_LOOP_IQ_MAX_IOUT 0.01f

#define CURRENT_LOOP_ID_KP 0.0001f
#define CURRENT_LOOP_ID_KI 0.00000000000f
#define CURRENT_LOOP_ID_KD 0.00000000000f
#define CURRENT_LOOP_ID_MAX_OUT 1.0f
#define CURRENT_LOOP_ID_MAX_IOUT 0.01f

#define SPEED_LOOP_KP 0.001f
#define SPEED_LOOP_KI 0.0000000f
#define SPEED_LOOP_KD 0.0000000f
#define SPEED_LOOP_MAX_OUT 1.0f
#define SPEED_LOOP_MAX_IOUT 0.01f

#define POSITION_LOOP_KP 0.001f
#define POSITION_LOOP_KI 0.0000000f
#define POSITION_LOOP_KD 0.0000000f
#define POSITION_LOOP_MAX_OUT 1.0f
#define POSITION_LOOP_MAX_IOUT 0.01f

#define SLOW_POS_LOOP_KP 0.01f
#define SLOW_POS_LOOP_KI 0.001f
#define SLOW_POS_LOOP_KD 0.001f
#define SLOW_POS_LOOP_MAX_OUT 1.0f
#define SLOW_POS_LOOP_MAX_IOUT 0.01f

#define MIX_LOOP_KP 0.01f
#define MIX_LOOP_KI 0.0f
#define MIX_LOOP_KD 0.01f
#define MIX_LOOP_MAX_OUT_POS 1.0f
#define MIX_LOOP_MAX_OUT_SPEED 1.0f

void Vbus_Voltage_Get_Init(Sensor_Msg_Typedef_t *Sensor_Msg);
void PID_All_Init();
void Init_Params(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void TIM_Init(void);
void ADC_Init(void);
void Drive_Init(void);

#endif