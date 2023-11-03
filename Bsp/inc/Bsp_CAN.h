#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#define MOTOR_CMD_ID 0x200
#define MOTOR_ID1 0x201
#define MOTOR_ID2 0x202
#define MOTOR_ID3 0x203
#define MOTOR_ID4 0x204

typedef struct
{
    uint8_t work_mode;
    int32_t value;	
    int32_t pos_h, pos_l;
    int32_t cmd_pos, cmd_speed, cmd_current;
    int16_t cmd_vd, cmd_vq;
}recv_msg_fdcan_t;

void FDCAN_Filter();
void FDCAN_cmd(FDCAN_HandleTypeDef *hfdcan, uint16_t position, int16_t speed_rpm, int16_t given_current, int8_t temperature, uint8_t error_number);
#endif