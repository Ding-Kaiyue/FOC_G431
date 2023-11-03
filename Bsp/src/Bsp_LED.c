/**
	****************************************************************************
	* @file					Bsp_LED.c
	* @author				Kaiyue Ding
	* @version  			V1.0.0
	* @date					2023/10/20
	* @brief				LED状态板级支持包
	****************************************************************************
	* @attention		None
	*
	****************************************************************************
	*/

#include "Bsp_LED.h"
#include "stm32g4xx.h"

uint16_t toggle_time = 0;

/**
 * **************************************************************
 * @fn 		LED_Light
 * @version	V1.0.0
 * @date	2023/10/20
 * @brief	LED常亮(电机处于校准状态)
 * @param	NONE
 * **************************************************************
 * @attention NONE
 * **************************************************************
 */
void LED_Light()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
}


/**
 * **************************************************************
 * @fn 		LED_Dark
 * @version	V1.0.0
 * @date	2023/10/20
 * @brief	LED常灭(未用到)
 * @param	NONE
 * **************************************************************
 * @attention NONE
 * **************************************************************
 */
void LED_Dark()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
}


/**
 * **************************************************************
 * @fn 		LED_Toggle
 * @version	V1.0.0
 * @date	2023/10/20
 * @brief	正常工作模式下LED跟随id闪烁
 * @param	NONE
 * **************************************************************
 * @attention NONE
 * **************************************************************
 */
void LED_Toggle(uint8_t id_num)
{
	for(uint8_t i = 0; i < id_num; i++)
	{
		//亮200
		for (uint8_t i = 0; i < 200; i++)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
		}
		//灭200
		for (uint8_t i = 0; i < 200; i++)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		}
	}
	//灭600
	for(uint16_t i = 0; i < 600; i++)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	}
}
	
/**
 * **************************************************************
 * @fn 		LED_Error_Toggle
 * @version	V1.0.0
 * @date	2023/10/20
 * @brief	电机异常工作状态下LED快闪
 * @param	NONE
 * **************************************************************
 * @attention NONE
 * **************************************************************
 */

void LED_Error_Toggle()
{
	for(uint8_t i = 0; i < 200; i++){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	}
	for(uint8_t i = 0; i < 200; i++){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
    }
}


/**
 * ***************************************************************
 * @fn         LED_Warning_Toggle
 * @version    V1.0.0
 * @date	   2023/10/20
 * @brief      电机警告状态总线上有相同ID设备LED慢闪,
 *             在Bsp_CAN.c里执行
 * @param      NONE
 * ***************************************************************
 * @attention  NONE
 * ***************************************************************
 */
void LED_Warning_Toggle()
{
	for(uint8_t i = 0; i < 1000; i++)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	}
	for(uint8_t i = 0; i < 1000; i++)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	}
}