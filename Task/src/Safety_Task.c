#include "Safety_Task.h"
#include "Bsp_Motor.h"
#include "FOC.h"

/* 错误码 0->No error; 1->Over voltage; 2->Lower voltage; 3->Over current; 4->Over temperature; */
uint8_t error_number = NO_ERROR;  


void Safety_Protection(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    
    if(Sensor_Msg->Vbus > VBUS_UPPER_LIMIT)
    {
        error_number = OVER_VOLTAGE;
    }
    else if(Sensor_Msg->Vbus < VBUS_LOWER_LIMIT)
    {
        error_number = UNDER_VOLTAGE;
    }
    else if(FOC->Curr_Components.Iq > IQ_UPPER_LIMIT)
    {
        error_number = OVER_CURRENT;
    }
    else if(Motor_Temperature_Celsius_Get(Sensor_Msg) > CELSIUS_UPPER_LIMIT)
    {
        error_number = OVER_TEMPERATURE;
    }
    else 
    {
        error_number = NO_ERROR;
    }
}

