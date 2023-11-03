#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"

#define SQRT3 1.73205080756888
#define PI 3.14159265358979f
#define V_BUS 24		    //初始母线电压24
#define POLE_PAIR_NUM 7	    //电机极性对数
#define ENCODER_BIT12

typedef struct
{
	//Msg 处理后的数据
	fp32 theta_elec;
	fp32 theta_mech, last_theta_mech;
	fp32 theta_gear;
	int32_t ecd, series_ecd, last_series_ecd;
	fp32 series_theta_mech, last_series_theta_mech;
	int32_t rounds;
	fp32 speed_rpm;
	fp32 Vbus;
	fp32 Ia;
	fp32 Ib;
	fp32 Ic;
	fp32 Celsius;
}Sensor_Msg_Typedef_t;

void Motor_Angle_Get(Sensor_Msg_Typedef_t *sensor_msg);
void Phase_Current_Get(Sensor_Msg_Typedef_t *sensor_msg);
void Vbus_Voltage_Get(Sensor_Msg_Typedef_t *sensor_msg);
int8_t Motor_Temperature_Celsius_Get(Sensor_Msg_Typedef_t *sensor_msg);

#endif