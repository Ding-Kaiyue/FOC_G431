/**
	****************************************************************************
	* @file					High_Freq_FOC_Task.c
	* @author				Kaiyue Ding
	* @version  			V1.0.0
	* @date					2023/10/20
	* @brief				高频电机任务: FOC运算
	****************************************************************************
	* @attention		None
	*
	****************************************************************************
	*/
#include "High_Freq_FOC_Task.h"
#include "Bsp_ADC.h"
#include "FOC.h"
#include "arm_math.h"
#include "Bsp_LED.h"
#include "Bsp_CAN.h"

Sensor_Msg_Typedef_t Sensor_Msg;
FOC_Typedef_t FOC;
bool_t cali_flag = FALSE;   //默认电机未校准
uint32_t cali_time = 0; //执行校准的时间
extern recv_msg_fdcan_t recv_msg_fdcan;


void High_Freq_FOC(void)
{
	/* Sensor_Message_Get */
	Motor_Angle_Get(&Sensor_Msg);
	Phase_Current_Get(&Sensor_Msg);
	Vbus_Voltage_Get(&Sensor_Msg);

	/* FOC Calculate*/
	if(recv_msg_fdcan.work_mode != OPEN_TORQUE_MODE) {
		Clarke_Transform(&FOC, &Sensor_Msg);
		Park_Transform(&FOC, &Sensor_Msg);
		PID_Current_Loop_Calc(&FOC);
		Vdq_Normalization(&FOC);
		InvPark_Transform(&FOC, &Sensor_Msg);
		SVPWM_Calc(&FOC, &Sensor_Msg);
	}
	else {
		Vdq_Normalization(&FOC);
		InvPark_Transform(&FOC, &Sensor_Msg);
		SVPWM_Calc(&FOC, &Sensor_Msg);
	}
}

/**
 * ***********************************************************************************
 * @fn        Calibration_Motor(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
 * @version   V1.0.0
 * @date      2023/10/23
 * @brief     电机校准函数：给定电角度为0°，Vq = a, Vd = 0可以让电机旋转至电机零位
 *                         上电时校准一次，校准后方能运行电机
 * @param     FOC: pointer to a FOC_Typedef structure
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure
 * @ref       https://www.zhihu.com/question/53944400/answer/138930785
 * ***********************************************************************************
 * @attention None
 * ***********************************************************************************
 */
void Calibration_Motor(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    FOC->Volt_Components.Vd = 0;
    FOC->Volt_Components.Vq = 1;
    Sensor_Msg->theta_elec = 0.0f;
   /*  InvPark_Transform(FOC,Sensor_Msg);
    SVPWM_Calc(FOC,Sensor_Msg); */
    cali_time ++;
	LED_Light();
    if(cali_time >= CALIBRATION_TIME){
        cali_time = 0;
		Sensor_Msg->theta_mech = 0.0f;
        cali_flag = TRUE;
    }
}
