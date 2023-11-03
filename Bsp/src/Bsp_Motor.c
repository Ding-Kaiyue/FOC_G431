/**
	****************************************************************************
	* @file					Bsp_Motor.c
	* @author				Kaiyue Ding
	* @version  			V1.0.0
	* @date					2023/10/24
	* @brief				电机数据反馈板级支持包
	****************************************************************************
	* @attention		None
	*
	****************************************************************************
	*/

#include "Bsp_Motor.h"
#include "Bsp_ADC.h"

extern ADC_RawData_t adc_rawdata;

/**
 * @brief From -45 to 70
 */ 
const uint16_t NTC_TABLE[116] =																	\
{																								\
	63743,	63623,	63497,	63364,	63225,	63077,	62922,	62759,	62588,	62409,				\
	62221,	62024,	61817,	61602,	61376,	61140,	60895,	60638,	60371,	60093,				\
	59804,	59504,	59192,	58868,	58533,	58185,	57826,	57454,	57071,	56675,				\
	56266,	55846,	55413,	54968,	54511,	54042,	53561,	53069,	52565,	52050,				\
	51523,	50986,	50439,	49882,	49315,	48738,	48153,	47559,	46957,	46348,				\
	45731,	45108,	44479,	43845,	43205,	42561,	41914,	41263,	40609,	39954,				\
	39297,	38639,	37981,	37323,	36666,	36010,	35356,	34704,	34055,	33410,				\
	32768,	32131,	31498,	30871,	30249,	29633,	29023,	28420,	27823,	27234,				\
	26652,	26079,	25512,	24954,	24405,	23864,	23331,	22807,	22292,	21786,				\
	21288,	20800,	20321,	19851,	19390,	18938,	18495,	18061,	17636,	17220,				\
	16813,	16415,	16025,	15644,	15271,	14907,	14551,	14203,	13864,	13532,				\
	13208,	12892,	12583,	12282,	11988,	11701												\
};

#define abs(x) (((x) < 0)? (-(x)) : (x))

/**
 * ********************************************************************************
 * @fn		Motor_Angle_Get(Sensor_Msg_Typedef_t *sensor_msg)
 * @version	V1.0.0
 * @date	2023/10/20
 * @brief	电机角度获取
 * @param	sensor_msg: pointer to a Sensor_Msg_Typedef structure
 * ********************************************************************************
 * @attention	None
 * ********************************************************************************
 */
void Motor_Angle_Get(Sensor_Msg_Typedef_t *sensor_msg)
{
	sensor_msg->theta_elec = PI - atan2f(adc_rawdata.Hall[2],(SQRT3/3) * (adc_rawdata.Hall[3] + adc_rawdata.Hall[4]));
	sensor_msg->theta_mech = sensor_msg->theta_elec / POLE_PAIR_NUM;
	sensor_msg->theta_gear = PI - atan2f(adc_rawdata.Hall[0],adc_rawdata.Hall[1]);

    #ifdef ENCODER_BIT16
    uint16_t encoder_range = 65536;
    #endif
    #ifdef ENCODER_BIT14
    uint16_t encoder_range = 16384;
    #endif
    #ifdef ENCODER_BIT12
    uint16_t encoder_range = 4096;
    #endif

    sensor_msg->ecd = sensor_msg->theta_mech / (2*PI) * encoder_range;
}

/**
 * ********************************************************************************
 * @fn        Phase_Current_Get(Sensor_Msg_Typedef_t *sensor_msg)
 * @version   V1.0.0
 * @date	  2023/10/20
 * @brief     电机相电流获取
 * @param	sensor_msg: pointer to a Sensor_Msg_Typedef structure
 * ********************************************************************************
 * @attention	None
 * ********************************************************************************
 */
void Phase_Current_Get(Sensor_Msg_Typedef_t *sensor_msg)
{
	#ifdef ENCODER_BIT16
	uint16_t phase_current_offset = 32768;
	uint16_t encoder_range = 65536;
	#endif
	#ifdef ENCODER_BIT14
	uint16_t phase_current_offset = 8192;
	uint16_t encoder_range = 16384;
    #endif
    #ifdef ENCODER_BIT12
    uint16_t phase_current_offset = 2048;
	uint16_t encoder_range = 4096;
    #endif
    
	sensor_msg->Ia = (adc_rawdata.Phase_Current_Raw[0] - phase_current_offset) * 3.3 / encoder_range / 0.271;
	sensor_msg->Ib = (adc_rawdata.Phase_Current_Raw[1] - phase_current_offset) * 3.3 / encoder_range / 0.271;
	sensor_msg->Ic = -sensor_msg->Ia - sensor_msg->Ib;
}


/**
 * ********************************************************************************
 * @fn        Vbus_Voltage_Get(Sensor_Msg_Typedef_t *sensor_msg)
 * @version   V1.0.0
 * @date	  2023/10/20
 * @brief     母线电压获取
 * @param	  sensor_msg: pointer to a Sensor_Msg_Typedef structure.
 * ********************************************************************************
 * @attention	None
 * ********************************************************************************
 */
void Vbus_Voltage_Get(Sensor_Msg_Typedef_t *sensor_msg)
{
	fp32 tmp_vbus = 0.0;
	
	#ifdef ENCODER_BIT16
	uint16_t encoder_range = 65536;
	#endif
	#ifdef ENCODER_BIT14
	uint16_t encoder_range = 16384;
    #endif
    #ifdef ENCODER_BIT12
	uint16_t encoder_range = 4096;
    #endif

	tmp_vbus = adc_rawdata.Vbus_raw * V_BUS / encoder_range;
	sensor_msg->Vbus = (sensor_msg->Vbus * 0.95) + (tmp_vbus * 0.05);
}


/**
 * ********************************************************************************
 * @fn        Motor_Temperature_Get(Sensor_Msg_Typedef_t *sensor_msg)
 * @version	  V1.0.0
 * @date      2023/10/24
 * @brief     电机温度获取，用于过温保护
 * @param	  sensor_msg: pointer to a Sensor_Msg_Typedef structure.
 * ********************************************************************************
 * @attention  None
 * ********************************************************************************
 */
int8_t Motor_Temperature_Celsius_Get(Sensor_Msg_Typedef_t *sensor_msg)
{
	for (uint8_t i = 0; i < 150; i++)
	{
		if(abs(adc_rawdata.Temperature - NTC_TABLE[i]) <= 15)
		{
			sensor_msg->Celsius = i - 45;
		}
	}
	return sensor_msg->Celsius;
}

