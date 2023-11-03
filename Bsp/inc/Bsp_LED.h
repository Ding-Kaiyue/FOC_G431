#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "main.h"

void LED_Light();
void LED_Dark();
void LED_Toggle(uint8_t id_num);
void LED_Error_Toggle();
void LED_Warning_Toggle();

#endif