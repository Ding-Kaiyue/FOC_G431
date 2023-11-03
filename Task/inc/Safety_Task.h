#ifndef __SAFETY_TASK_H
#define __SAFETY_TASK_H

#include "main.h"
#include "FOC.h"
#include "Bsp_Motor.h"

#define VBUS_UPPER_LIMIT 30     //过压
#define VBUS_LOWER_LIMIT 11     //欠压
#define IQ_UPPER_LIMIT   5      //电流上限
#define CELSIUS_UPPER_LIMIT 60  //温度上限

enum {
    NO_ERROR  = 0,
    OVER_VOLTAGE = 1,
    UNDER_VOLTAGE = 2,
    OVER_CURRENT = 3,
    OVER_TEMPERATURE = 4,
};

void Safety_Protection(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *sensor_msg);

#endif