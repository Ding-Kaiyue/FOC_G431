#ifndef __MEDIUM_FREQ_TASK_H
#define __MEDIUM_FREQ_TASK_H

#include "main.h"
#include "FOC.h"
#include "Bsp_Motor.h"

void Medium_Freq_State_Mechine(void);
void Process_Motor_Angle(Sensor_Msg_Typedef_t *Sensor_Msg);
void Speed_Calc(Sensor_Msg_Typedef_t *Sensor_Msg);
void Calibration_Motor(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void Speed_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void Slow_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void Abs_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void Inc_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void Torque_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void Open_Torque_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void Mix_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
void LED_State(uint8_t error_number);


#endif