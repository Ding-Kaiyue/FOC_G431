/**
	****************************************************************************
	* @file					Bsp_ADC.c
	* @author				Kaiyue Ding
	* @version  			V1.0.0
	* @date					2023/10/20
	* @brief				ADC采样板级支持包
	****************************************************************************
	* @attention		
	* 					Ib     -> ADC2_IN2 -> rank1 -> ADC2_Data[0]
	*					vbus   -> ADC2_IN3 -> rank2 -> ADC2_Data[1]
	*                   Temp   -> ADC2_IN4 -> rank3 -> ADC2_Data[2]
	*					hall_2 -> ADC2_in13 -> rank4 -> ADC2_Data[3] -> hall[1]
	*                   hall_1 -> ADC2_IN17 -> rank5 -> ADC2_Data[4] -> hall[0]
	*
    *                   Ia     -> ADC1_IN1 -> rank1 -> ADC1_Data[0]
	*                   hall_3 -> ADC1_IN3 -> rank2 -> ADC1_Data[1] -> hall[2]
	*                   hall_4 -> ADC1_IN4 -> rank3 -> ADC1_Data[2] -> hall[3]
	*					hall_5 -> ADC1_IN15 -> rank4 -> ADC1_Data[3] -> hall[4]        
	****************************************************************************
	*/

#include "Bsp_ADC.h"
#include "Init_Task.h"

uint32_t sum1_tmp = 0, sum2_tmp = 0;
extern uint16_t ADC1_Raw_Data[64][5];
extern uint16_t ADC2_Raw_Data[64][4];
uint16_t ADC1_Data[7],ADC2_Data[3];
ADC_RawData_t adc_rawdata;


/**
 * *************************************************
 * @fn 		Sensor_Message_Get_ADC1
 * @version	V1.0.0
 * @date	2023/10/20
 * @brief	ADC1原始数据获取
 * @param	NONE
 * *************************************************
 * @attention NONE
 * *************************************************
 */
void Sensor_Message_Get_ADC1()
{
	for(uint8_t i=0; i<7;i++)
	{
		for(uint8_t j=0; j<64; j++)
		{
			sum1_tmp += ADC1_Raw_Data[j][i];
		}
		ADC1_Data[i] = sum1_tmp >> 6;
		sum1_tmp = 0;
	}

	#ifdef ENCODER_BIT16
	uint16_t adc_offset = 32768;
	#endif
	#ifdef ENCODER_BIT14
	uint16_t adc_offset = 8192;
	#endif
	#ifdef ENCODER_BIT12
    uint16_t adc_offset = 2048;
	#endif

	adc_rawdata.Hall[2] = ADC1_Data[1] - adc_offset;
	adc_rawdata.Hall[3] = ADC1_Data[2] - adc_offset;
	adc_rawdata.Hall[4] = ADC1_Data[3] - adc_offset;
	adc_rawdata.Phase_Current_Raw[0] = ADC1_Data[0];
}


/**
 * *************************************************
 * @fn 		Sensor_Message_Get_ADC2
 * @version	V1.0.0
 * @date	2023/10/20
 * @brief	ADC2原始数据获取
 * @param	NONE
 * *************************************************
 * @attention NONE
 * *************************************************
 */
void Sensor_Message_Get_ADC2()
{
	for(uint8_t i=0; i<2;i++)
	{
		for(uint8_t j=0; j<64; j++)
		{
			sum2_tmp += ADC2_Raw_Data[j][i];
		}
		ADC2_Data[i] = sum2_tmp >> 6;
		sum2_tmp = 0;
	}

	#ifdef ENCODER_BIT16
	uint16_t adc_offset = 32768;
	#endif
	#ifdef ENCODER_BIT14
	uint16_t adc_offset = 8192;
	#endif
	#ifdef ENCODER_BIT12
    uint16_t adc_offset = 2048;
	#endif

	adc_rawdata.Hall[0] = ADC2_Data[4] - adc_offset;
	adc_rawdata.Hall[1] = ADC2_Data[3] - adc_offset;
	adc_rawdata.Temperature = ADC2_Data[2];
	adc_rawdata.Vbus_raw = ADC2_Data[1];
	adc_rawdata.Phase_Current_Raw[1] = ADC2_Data[0];
}