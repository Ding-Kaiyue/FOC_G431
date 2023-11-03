#ifndef __HIGH_FREQ_FOC_TASK_H
#define __HIGH_FREQ_FOC_TASK_H

#include "main.h"
#include "FOC.h"
#include "Bsp_Motor.h"

void High_Freq_FOC(void);
void Calibration_Motor(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg);
#endif