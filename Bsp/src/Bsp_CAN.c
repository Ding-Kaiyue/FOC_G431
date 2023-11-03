/**
	****************************************************************************
	* @file					Bsp_CAN.c
	* @author				Kaiyue Ding
	* @version  			V1.0.0
	* @date					2023/10/20
	* @brief				FDCAN板级支持包
	****************************************************************************
	* @attention		None
	*
	****************************************************************************
	*/
#include "Bsp_CAN.h"
#include "fdcan.h"
#include "stm32g4xx_hal_fdcan.h"
#include "Bsp_LED.h"

FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
recv_msg_fdcan_t recv_msg_fdcan;
uint8_t RxBuffer[25] = {0};
uint8_t TxData[8] = {0};

bool_t led_warning_flag = FALSE;


/**
 * **********************************************************************************
 * @fn       FDCAN_Filter
 * @version  V1.0.0
 * @date     2023/10/20
 * @brief    FDCAN过滤器
 * @param    None
 * **********************************************************************************
 * @attention NONE
 * **********************************************************************************
*/
void FDCAN_Filter()
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7FF;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
}


/**
 * ****************************************************************************************************************
 * @fn 		HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0Its)
 * @version	V1.0.0
 * @date	2023/10/20
 * @brief	FDCAN回调函数(正式情况下仅有四个work_mode)
 *          如果work_mode为速度模式，则value为速度值rpm(范围为-3000~3000)
 *          如果work_mode为绝对位置模式，则value为角度值(范围为0~4096)
 *          如果work_mode为相对位置模式，则value为增量角度值(范围为int32)
 *          如果work_mode为力矩模式，则value为Iq_ref(范围为？)
 *          如果work_mode为限位模式，则value为电流值
 * @param   hfdcan: pointer to a FDCAN_HandleTypeDef structure
 * @param   Rxfifo0Its: FIFO0 Interrupt happened
 * ****************************************************************************************************************
 * @attention NONE
 * ****************************************************************************************************************
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&RxHeader,RxBuffer) != HAL_OK)
        {
            Error_Handler();
        }
        else if(RxHeader.Identifier == MOTOR_CMD_ID)
        {
            recv_msg_fdcan.work_mode = RxBuffer[0];
            if(recv_msg_fdcan.work_mode == OPEN_TORQUE_MODE)
            {
                recv_msg_fdcan.cmd_vd = (RxBuffer[1] << 8 | RxBuffer[2]);
                recv_msg_fdcan.cmd_vq = (RxBuffer[3] << 8 | RxBuffer[4]);
            }
            else {
                recv_msg_fdcan.value = (RxBuffer[1] << 24 | RxBuffer[2] << 16 | RxBuffer[3] << 8 | RxBuffer[4]);
            }
            
            /* 限位时以下两个变量是有用的 */
            if(recv_msg_fdcan.work_mode == POS_LIMIT_MODE) {
                recv_msg_fdcan.pos_l = (RxBuffer[5] << 24 | RxBuffer[6] << 16 | RxBuffer[7] << 8 | RxBuffer[8]);
                recv_msg_fdcan.pos_h = (RxBuffer[9] << 24 | RxBuffer[10] << 16 | RxBuffer[11] << 8 | RxBuffer[12]);
            }
            /* 混合控制时以下三个变量是有用的 */
            else if(recv_msg_fdcan.work_mode == MIX_MODE) {
                recv_msg_fdcan.cmd_pos = (RxBuffer[13] << 24 | RxBuffer[14] << 16 | RxBuffer[15] << 8 | RxBuffer[16]);
                recv_msg_fdcan.cmd_speed = (RxBuffer[17] << 24 | RxBuffer[18] << 16 | RxBuffer[19] << 17 | RxBuffer[20]);
                recv_msg_fdcan.cmd_current = (RxBuffer[21] << 24 | RxBuffer[22] << 16 | RxBuffer[23] << 23 | RxBuffer[24]);
            }
        }

        if(RxHeader.Identifier != MOTOR_ID1)
        {
            led_warning_flag = FALSE;
        }
        else if(RxHeader.Identifier == MOTOR_ID1)
        {
            led_warning_flag = TRUE;
            LED_Warning_Toggle();
        }
    } 
}


/**
 * *******************************************************************************************************************************************************
 * @fn      FDCAN_cmd(FDCAN_HandleTypeDef *hfdcan, uint16_t position, int16_t speed_rpm, int16_t given_current, uint8_t working_state, int8_t temperature)
 * @version V1.0.0
 * @date    2023/10/20
 * @brief   FDCAN发送函数
 *          电机反馈数据: 1->编码器数据，范围[0,4096]
 *                       2->电机速度
 *                       3->速度环输出的电机目标电流Iq_set
 *                       4->电机温度
 *                       5->错误码
 * @param   hfdcan: pointer to a FDCAN_HandleTypeDef structure.
 * @param   position: [0,4096]
 * @param   speed_rpm: [-3000,3000]
 * @param   given_current: Iq_set
 * @param   temperature: [-45,70]
 * @param   error_number: 0->No error, 1->OVER_VOLTAGE, 2->UNDER_VOLTAGE, 3->OVER_CURRENT, 4->OVER_TEMPERATURE
 * *******************************************************************************************************************************************************
 * @attention NONE
 * *******************************************************************************************************************************************************
 */
void FDCAN_cmd(FDCAN_HandleTypeDef *hfdcan, uint16_t position, int16_t speed_rpm, int16_t given_current, int8_t temperature, uint8_t error_number)
{
    TxHeader.Identifier = MOTOR_ID1;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = 0x08;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxData[0] = position << 8;
    TxData[1] = position;
    TxData[2] = speed_rpm << 8;
    TxData[3] = speed_rpm;
    TxData[4] = given_current << 8;
    TxData[5] = given_current;
    TxData[6] = temperature;
    TxData[7] = error_number;
}

