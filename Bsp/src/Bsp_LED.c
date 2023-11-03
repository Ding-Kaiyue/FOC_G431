/**
	****************************************************************************
	* @file					Bsp_LED.c
	* @author				Kaiyue Ding
	* @version  			V1.0.0
	* @date					2023/10/20
	* @brief				LED״̬�弶֧�ְ�
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
 * @brief	LED����(�������У׼״̬)
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
 * @brief	LED����(δ�õ�)
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
 * @brief	��������ģʽ��LED����id��˸
 * @param	NONE
 * **************************************************************
 * @attention NONE
 * **************************************************************
 */
void LED_Toggle(uint8_t id_num)
{
	for(uint8_t i = 0; i < id_num; i++)
	{
		//��200
		for (uint8_t i = 0; i < 200; i++)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
		}
		//��200
		for (uint8_t i = 0; i < 200; i++)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		}
	}
	//��600
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
 * @brief	����쳣����״̬��LED����
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
 * @brief      �������״̬����������ͬID�豸LED����,
 *             ��Bsp_CAN.c��ִ��
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