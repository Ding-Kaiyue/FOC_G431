/**
  ************************************************************************************
  * @file				Init_Task.c
  * @author				Kaiyue Ding
  * @version  			V1.0.0
  * @date				2023/10/20
  * @brief				初始化任务，开启定时器中断、ADC采样，初始化参数、开启逆变器
  **********************************************************************************
  * @attention		None
  *
  **********************************************************************************
  */

#include "Init_Task.h"
#include "Bsp_ADC.h"
#include "PID.h"
#include "tim.h"
#include "adc.h"

extern ADC_RawData_t adc_rawdata;
uint16_t ADC1_Raw_Data[64][4];
uint16_t ADC2_Raw_Data[64][5];
PID_Typedef_t PID_Current_Loop_Iq, PID_Current_Loop_Id, PID_Speed_Loop,PID_Position_Loop, PID_Slow_Position_Loop, PID_Mix_Loop_Pos, PID_Mix_Loop_Speed;
const fp32 PID_Current_Loop_Iq_Params[3] = {CURRENT_LOOP_IQ_KP, CURRENT_LOOP_IQ_KI, CURRENT_LOOP_IQ_KD};
const fp32 PID_Current_Loop_Id_Params[3] = {CURRENT_LOOP_ID_KP, CURRENT_LOOP_ID_KI, CURRENT_LOOP_ID_KD};
const fp32 PID_Speed_Loop_Params[3] = {SPEED_LOOP_KP, SPEED_LOOP_KI, SPEED_LOOP_KD};
const fp32 PID_Position_Loop_Params[3] = {POSITION_LOOP_KP, POSITION_LOOP_KI, POSITION_LOOP_KD};
const fp32 PID_Slow_Position_Loop_Params[3] = {SLOW_POS_LOOP_KP, SLOW_POS_LOOP_KI, SLOW_POS_LOOP_KD};
const fp32 PID_Mix_Loop_Params_Pos[3] = {MIX_LOOP_KP, 0, 0};
const fp32 PID_Mix_Loop_Params_Speed[3] = {0,0,MIX_LOOP_KD};

/**
 * ****************************************************************
 * @fn        Vbus_Voltage_Get_Init(Sensor_Msg_Typedef *Sensor_Msg)
 * @version   V1.0.0
 * @date	  2023/10/20
 * @brief     上电时获得一次母线电压值，便于后续计算母线电压
 * @param	  sensor_msg: pointer to a Sensor_Msg_Typedef structure.
 * ****************************************************************
 * @attention	None
 * ****************************************************************
 */
void Vbus_Voltage_Get_Init(Sensor_Msg_Typedef_t *Sensor_Msg)
{
    #ifdef ENCODER_BIT16
	uint16_t resolution = 65536;
	#endif
	#ifdef ENCODER_BIT14
	uint16_t resolution = 16384;
    #endif
    #ifdef ENCODER_BIT12
	uint16_t resolution = 4096;
    #endif

    Sensor_Msg->Vbus = adc_rawdata.Vbus_raw * V_BUS / resolution;
}


/**
 * ****************************************************************
 * @fn        PID_All_Init()
 * @version   V1.0.0
 * @date      2023/10/20
 * @brief     PID 初始化
 * @param     NONE
 * ****************************************************************
 * @attention None
 * ****************************************************************
 */
void PID_All_Init()
{
    PID_Init(&PID_Current_Loop_Iq,PID_Current_Loop_Iq_Params,CURRENT_LOOP_IQ_MAX_OUT,CURRENT_LOOP_ID_MAX_IOUT);
    PID_Init(&PID_Current_Loop_Id,PID_Current_Loop_Id_Params,CURRENT_LOOP_ID_MAX_OUT,CURRENT_LOOP_ID_MAX_IOUT);   
    PID_Init(&PID_Speed_Loop,PID_Speed_Loop_Params,SPEED_LOOP_MAX_OUT,SPEED_LOOP_MAX_IOUT);
    PID_Init(&PID_Position_Loop,PID_Position_Loop_Params,POSITION_LOOP_MAX_OUT,POSITION_LOOP_MAX_IOUT);
	PID_Init(&PID_Slow_Position_Loop,PID_Slow_Position_Loop_Params,SLOW_POS_LOOP_MAX_OUT, SLOW_POS_LOOP_MAX_IOUT);
	PID_Init(&PID_Mix_Loop_Pos, PID_Mix_Loop_Params_Pos, MIX_LOOP_MAX_OUT_POS, 0);
	PID_Init(&PID_Mix_Loop_Params_Speed,PID_Mix_Loop_Params_Speed, MIX_LOOP_MAX_OUT_SPEED,0);
}


/**
 * *************************************************************************
 * @fn        Init_Params(FOC_Typedef *FOC, Sensor_Msg_Typedef *Sensor_Msg)
 * @version   V1.0.0
 * @date      2023/10/23
 * @brief     FOC and Sensor Msg initialization
 * @param     FOC: pointer to a FOC_Typedef structure.
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure.
 * *************************************************************************
 * @attention  None
 * *************************************************************************
 */
void Init_Params(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
	FOC->Curr_Components.Ialpha = FOC->Curr_Components.Ibeta = 0.0f;
	FOC->Curr_Components.Id = FOC->Curr_Components.Iq = 0.0f;
	FOC->Curr_Components.Id_set = FOC->Curr_Components.Iq_set = 0.0f;
	FOC->Volt_Components.Valpha = FOC->Volt_Components.Vbeta = 0.0f;
	FOC->Volt_Components.Vd = FOC->Volt_Components.Vq = 0.0f;
	FOC->Pwm_Components.Tcm1 = FOC->Pwm_Components.Tcm2 = FOC->Pwm_Components.Tcm3 = 0;
	Sensor_Msg->rounds = 0;
	Sensor_Msg->last_theta_mech = 0.0f;
	Sensor_Msg->series_ecd = Sensor_Msg->last_series_ecd = 0;
	Sensor_Msg->series_theta_mech = Sensor_Msg->last_series_theta_mech = 0.0f;
}


/* 时钟初始化 */
void TIM_Init(void)
{
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
}


/* ADC初始化 */
void ADC_Init(void)
{
	HAL_ADC_Init(&hadc1);
	HAL_ADC_Init(&hadc2);
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) ADC1_Raw_Data, 64*4);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*) ADC2_Raw_Data, 64*5);
}


/* 逆变器初始化 */
void Drive_Init(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}






