#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "main.h"


typedef struct
{
	//ADC_raw 原始ADC数据
	int16_t Hall[5];
	uint16_t Temperature;
	uint16_t Vbus_raw;
	uint16_t Phase_Current_Raw[2];
}ADC_RawData_t;

void Sensor_Message_Get_ADC1();
void Sensor_Message_Get_ADC2();
#endif