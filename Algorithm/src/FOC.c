/**
	*******************************************************************************
	* @file					FOC.c
	* @author				Kaiyue Ding
	* @version  			V1.0.0
	* @date					2023/10/20
	* @brief				FOC算法
	*******************************************************************************
	* @attention		None
	*
	*******************************************************************************
	*/

#include "FOC.h"
#include "pid.h"
#include "arm_math.h"
#include "Bsp_ADC.h"
#include "Init_Task.h"


extern PID_Typedef_t PID_Current_Loop_Iq, PID_Current_Loop_Id;

#define PWM_Output_Limit(val)							\
{														\
	(val > PWM_TS) ? (val = PWM_TS) : (val);			\
	(val < 0) ? (val = 0) : (val);						\
}


/**
 * ********************************************************************************
 * @fn        Over_Modulation(uint16_t time1,uint16_t time2)
 * @version   V1.0.0
 * @date	  2023/10/23
 * @brief     过调制处理
 * @param	  time1: pwm自动重装载值1
 * @param	  time2: pwm自动重装载值2
 * ********************************************************************************
 * @attention	None
 * ********************************************************************************
 */
void Over_Modulation(uint16_t time1, uint16_t time2)
{
	if((time1 + time2) > PWM_TS)
	{
		time1 /= (time1 + time2) * PWM_TS;
		time2 /= (time1 + time2) * PWM_TS;
	}
}


/**
 * *******************************************************************************
 * @fn 		  Vdq_Normalization(FOC_Typedef_t *foc)
 * @version	  V1.0.0
 * @date	  2023/10/23
 * @brief	  电流闭环Vdq限幅
 * @param	  foc: pointer to a FOC_Typedef structure
 * *******************************************************************************
 * @attention None
 * *******************************************************************************
 */
void Vdq_Normalization(FOC_Typedef_t *foc)
{
	fp32 Vdq_tmp = 0.0;
	arm_sqrt_f32(foc->Volt_Components.Vd * foc->Volt_Components.Vd + foc->Volt_Components.Vq * foc->Volt_Components.Vq, &Vdq_tmp);
	if(Vdq_tmp > 1.0f)
	{
		foc->Volt_Components.Vd /= Vdq_tmp;
        foc->Volt_Components.Vq /= Vdq_tmp;
	}
}


/**
 * ********************************************************************************
 * @fn		  Clarke_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg)
 * @version   V1.0.0
 * @date	  2023/10/20
 * @brief	  Clarke变换
 * @param	  foc: pointer to a FOC_Typedef structure
 * @param	  sensor_msg: pointer to a Sensor_Msg_Typedef structure
 * @def		  Ialpha = Ia
 *            Ibeta = (1/sqrt(3)) * Ia + (2/sqrt(3)) * Ib
 * ********************************************************************************
 * @attention	None
 * ********************************************************************************
 */
void Clarke_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg)
{
    foc->Curr_Components.Ialpha = sensor_msg->Ia;
    foc->Curr_Components.Ibeta = SQRT3 / 3 * sensor_msg->Ia + (2 / SQRT3) * sensor_msg->Ib;
}


/**
 * ********************************************************************************
 * @fn		  Park_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg)
 * @version   V1.0.0
 * @date	  2023/10/20
 * @brief	  Park变换
 * @param	  foc: pointer to a FOC_Typedef structure
 * @param	  sensor_msg: pointer to a Sensor_Msg_Typedef structure
 * @def		  Id = Ialpha * cosVal + Ibeta * sinVal
 * 			  Iq = - Ialpha * sinVal + Ibeta * cosVal
 * ********************************************************************************
 * @attention	None
 * ********************************************************************************
 */
void Park_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg)
{
    fp32 cos_theta = arm_cos_f32(sensor_msg->theta_elec);
    fp32 sin_theta = arm_sin_f32(sensor_msg->theta_elec);
    foc->Curr_Components.Id = foc->Curr_Components.Ialpha * cos_theta + foc->Curr_Components.Ibeta * sin_theta;
    foc->Curr_Components.Iq = -foc->Curr_Components.Ialpha * sin_theta + foc->Curr_Components.Ibeta * cos_theta;
}


/**
 * *******************************************************************************
 * @fn		  PID_Current_Loop_Calc(FOC_Typedef_t *foc)
 * @version   V1.0.0
 * @date	  2023/10/20
 * @brief	  电流闭环PID计算
 * @param	  foc: pointer to a FOC_Typedef structure
 * *******************************************************************************
 * @attention None
 * *******************************************************************************
 */
void PID_Current_Loop_Calc(FOC_Typedef_t *foc)
{
	PID_Calc(&PID_Current_Loop_Id,foc->Curr_Components.Id,foc->Curr_Components.Id_set);
	PID_Calc(&PID_Current_Loop_Iq,foc->Curr_Components.Iq,foc->Curr_Components.Iq_set);
	foc->Volt_Components.Vd = PID_Current_Loop_Id.out;
	foc->Volt_Components.Vq = PID_Current_Loop_Iq.out;
}


/**
 * ********************************************************************************
 * @fn		  RevPark_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg)
 * @version   V1.0.0
 * @date	  2023/10/20
 * @brief	  逆Park变换
 * @param	  foc: pointer to a FOC_Typedef structure
 * @param	  sensor_msg: pointer to a Sensor_Msg_Typedef structure
 * @def		  Valpha = Vd * cosVal - Vq * sinVal
 * 			  Vbeta = Vd * sinVal + Vq * cosVal
 * ********************************************************************************
 * @attention	None
 * ********************************************************************************
 */
void InvPark_Transform(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg)
{
    fp32 cos_theta = arm_cos_f32(sensor_msg->theta_elec);
    fp32 sin_theta = arm_sin_f32(sensor_msg->theta_elec);
    foc->Volt_Components.Valpha = foc->Volt_Components.Vd * cos_theta - foc->Volt_Components.Vq * sin_theta;
    foc->Volt_Components.Vbeta = foc->Volt_Components.Vd * sin_theta + foc->Volt_Components.Vq * cos_theta;
}


/**
 * ********************************************************************************************
 * @fn		  SVPWM_Calc(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg)
 * @version	  V1.0.0
 * @date	  2023/10/23
 * @brief	  SVPWM变换
 * @param	  foc: pointer to a FOC_Typedef structure
 * @param	  sensor_msg: pointer to a Sensor_Msg_Typedef structure
 * @def		  扇区计算：N = 4C + 2B + A;
 * 			  非零矢量和零矢量作用时间计算：
 * 			  X = \frac{SQRT3 * Ts * Vbeta}{Vdc}
 *            Y = \frac{SQRT3 * Ts}{Vdc} * (\frac{SQRT3 * Valpha}{2} + frac{1}{2} * Vbeta)
 *            Z = \frac{SQRT3 * Ts}{Vdc} * (-\frac{SQRT3 * Valpha}{2} + \frac{1}{2} * Vbeta)
 *                    各扇区作用时间表
 * 			  |  N  | 1 | 2 | 3 | 4 | 5 | 6 |
 *            | T_1 | Z | Y |- Z|- X| X |- Y|
 *            | T_2 | Y |- X| X | Z |- Y|- Z|
 *            | T_0 |   (Ts - T1 - T2) / 2  |
 *                  扇区矢量切换点关系表
 *            |  N  | 1 | 2 | 3 | 4 | 5 | 6 |
 *            |T_cm1| Tb| Ta| Ta| Tc| Tc| Tb|
 *            |T_cm2| Ta| Tc| Tb| Tb| Ta| Tc|
 *            |T_cm3| Tc| Tb| Tc| Ta| Tb| Ta|
 * *********************************************************************************************
 * @attention	输出限幅及过调制
 * *********************************************************************************************
 */ 
void SVPWM_Calc(FOC_Typedef_t *foc, Sensor_Msg_Typedef_t *sensor_msg)
{
	uint8_t A, B, C, N;
	fp32 Uref1, Uref2, Uref3;
	int16_t X, Y, Z;
	int16_t T1, T2;
	int16_t Ta, Tb, Tc;

	Uref1 = foc->Volt_Components.Vbeta;
	Uref2 = SQRT3 / 2 * foc->Volt_Components.Valpha - 1 / 2 * foc->Volt_Components.Vbeta;
	Uref3 = - SQRT3 / 2 * foc->Volt_Components.Valpha - 1 / 2 * foc->Volt_Components.Vbeta;

	if (Uref1 > 0.0f) A = 1; else A = 0;
	if (Uref2 > 0.0f) B = 1; else B = 0;
	if (Uref3 > 0.0f) C = 1; else C = 0;

	N = 4 * C + 2 * B + A;

	X = SQRT3 * foc->Volt_Components.Vbeta * PWM_TS / sensor_msg->Vbus;
	Y = 1.5f * foc->Volt_Components.Valpha * PWM_TS / sensor_msg->Vbus + SQRT3 / 2 * foc->Volt_Components.Vbeta * PWM_TS / sensor_msg ->Vbus;
	Z = -1.5f * foc->Volt_Components.Valpha * PWM_TS / sensor_msg ->Vbus + SQRT3 / 2 * foc->Volt_Components.Vbeta * PWM_TS / sensor_msg ->Vbus;

	switch (N)
	{
		case SECTOR1:
		{
			T1 = Z;
			T2 = Y;
			break;
		}
		case SECTOR2:
		{
			T1 = Y;
			T2 = -X;
			break;
		}
		case SECTOR3:
		{
			T1 = -Z;
			T2 = X;
			break;
		}
		case SECTOR4:
		{
			T1 = -X;
			T2 = Z;
			break;
		}
		case SECTOR5:
		{
			T1 = X;
            T2 = -Y;
			break;
		}
		case SECTOR6:
		{
			T1 = -Y;
            T2 = -Z;
			break;
		}
		default:
		{
			T1 = 0;
            T2 = 0;
            break;
		}
	}

	PWM_Output_Limit(T1);
	PWM_Output_Limit(T2);
	Over_Modulation(T1, T2);
	

	Ta = (PWM_TS - T1 - T2) / 4;
	Tb = Ta + T1 / 2;
	Tc = Tb + T2 / 2;

	switch(N)
	{
		case SECTOR1:
		{
			TIM1->CCR1 = Tb;
			TIM1->CCR2 = Ta;
			TIM1->CCR3 = Tc;
			break;
		}
		case SECTOR2:
		{
			TIM1->CCR1 = Ta;
            TIM1->CCR2 = Tc;
			TIM1->CCR3 = Tb;
            break;
		}
		case SECTOR3:
		{
			TIM1->CCR1 = Ta;
			TIM1->CCR2 = Tb;
			TIM1->CCR3 = Tc;
			break;
		}
		case SECTOR4:
		{
			TIM1->CCR1 = Tc;
            TIM1->CCR2 = Tb;
			TIM1->CCR3 = Ta;
			break;
		}
		case SECTOR5:
		{
			TIM1->CCR1 = Tc;
			TIM1->CCR2 = Ta;
			TIM1->CCR3 = Tb;
			break;
		}
		case SECTOR6:
		{
			TIM1->CCR1 = Tb;
            TIM1->CCR2 = Tc;
            TIM1->CCR3 = Ta;
            break;
		}
		default:
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			break;
		}
	}
}

