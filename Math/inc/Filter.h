#ifndef __FILTER_H
#define __FILTER_H

#include "main.h"

enum {
    FIRST_ORDER     = 0,
    SECOND_ORDER    = 1,
    KALMAN_FILTER   = 2,
};


void Filter_Init(uint8_t type);
fp32 First_Order_Filter(fp32 speed_in);
fp32 Second_Order_Filter(fp32 speed_in);


#define ALPHA  0.05f
#endif