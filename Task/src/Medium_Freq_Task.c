/**
	****************************************************************************
	* @file					Medium_Freq_Task.c
	* @author				Kaiyue Ding
	* @version  			V1.0.0
	* @date					2023/10/20
	* @brief				��Ƶ�������: ƽ���ٶȼ���
                                         ״̬���л�
	****************************************************************************
	* @attention		None
	*
	****************************************************************************
	*/

#include "Medium_Freq_Task.h"
#include "Bsp_CAN.h"
#include "Bsp_LED.h"
#include "PID.h"
#include "Bsp_Motor.h"
#include "Filter.h"
#include "Safety_Task.h"
#include "main.h"

extern recv_msg_fdcan_t recv_msg_fdcan;
extern FOC_Typedef_t FOC;
extern Sensor_Msg_Typedef_t Sensor_Msg;
extern PID_Typedef_t PID_Speed_Loop,PID_Position_Loop,PID_Slow_Position_Loop, PID_Mix_Loop_Pos, PID_Mix_Loop_Speed;
extern uint8_t error_number;
extern bool_t led_warning_flag;

bool_t position_limit_flag = FALSE;

/**
 * ****************************************************************************
 * @fn           State_Mechine_Switch(void)
 * @version      V1.0.0
 * @date	     2023/10/23
 * @brief        ��Ƶ����ִ��Ƶ�ʣ�1kHz��ִ��״̬���л�����
 * @param        None
 * ****************************************************************************
 * @attention    ֻ��У׼���ĵ�����ܽ���SPEED_MODE��POSITION_MODE��TORQUE_MODE
 * ****************************************************************************
 */
void Medium_Freq_State_Mechine(void)
{
    switch(recv_msg_fdcan.work_mode) {
        case SPEED_MODE:
        {
            Speed_Mode(&FOC, &Sensor_Msg);
            break;
        }
        case SLOW_POSITION_MODE:
        {
            Slow_Position_Mode(&FOC, &Sensor_Msg);
            break;
        }
        case ABS_POSITION_MODE:
        {
            Abs_Position_Mode(&FOC,&Sensor_Msg);
            break;
        }
        case INC_POSITION_MODE:
        {
            Inc_Position_Mode(&FOC,&Sensor_Msg);
            break;
        }
        case TORQUE_MODE:
        {
            Torque_Mode(&FOC,&Sensor_Msg);
            break;
        }
        case OPEN_TORQUE_MODE:
        {
            Open_Torque_Mode(&FOC,&Sensor_Msg);
            break;
        }
        case MIX_MODE:
        {
            Mix_Mode(&FOC,&Sensor_Msg);
            break;
        }

        case POS_LIMIT_MODE:
        {
            position_limit_flag = TRUE;
            break;
        }
        default: break;
    }
}


/**
 * ****************************************************************************
 * @fn        Process_Motor_Ecd(Sensor_Msg_Typedef_t *Sensor_Msg)
 * @version   V1.0.0
 * @date      2023/10/23
 * @brief     �����е�Ƕȹ�������
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure
 * ****************************************************************************
 * @attention None
 * ****************************************************************************
 */
void Process_Motor_Angle(Sensor_Msg_Typedef_t *Sensor_Msg)
{
    #ifdef ENCODER_BIT16
    uint32_t encoder_range = 65536;
    #endif
    #ifdef ENCODER_BIT14
    uint32_t encoder_range = 16384;
    #endif
    #ifdef ENCODER_BIT12
    uint32_t encoder_range = 4096;
    #endif

    Sensor_Msg->last_theta_mech = Sensor_Msg->theta_mech;
    Sensor_Msg->last_series_theta_mech = Sensor_Msg->series_theta_mech;

    Sensor_Msg->last_series_ecd = Sensor_Msg->series_ecd;

    if((Sensor_Msg->theta_mech - Sensor_Msg->last_theta_mech) > (PI)) {
        Sensor_Msg->rounds ++;
    }
    else if((Sensor_Msg->theta_mech - Sensor_Msg->last_theta_mech) < -(PI)){
        Sensor_Msg->rounds --;
    }

    Sensor_Msg->series_theta_mech = Sensor_Msg->rounds * (2*PI) + Sensor_Msg->theta_mech;
    Sensor_Msg->series_ecd = Sensor_Msg->rounds * encoder_range + Sensor_Msg->series_ecd;
}


/**
 * ****************************************************************************
 * @fn        Speed_Calc(Sensor_Msg_Typedef_t *Sensor_Msg)
 * @version   V1.0.1
 * @date      2023/11/02
 * @brief     �ٶȼ��㣺1ms����һ��,�ڶ�ʱ���ж���ִ��,�˲�
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure
 * @def       speed(rad/s) = delta_angle / delta_time
 *            speed(rpm) = speed(rad/s) * 60 / (2*PI)
 * @note      V1.0.1  ����speed_rpmΪ��е�Ƕȼ�����
 * ****************************************************************************
 * @attention None
 * ****************************************************************************
 */
void Speed_Calc(Sensor_Msg_Typedef_t *Sensor_Msg)
{
    #ifdef ENCODER_BIT16
    uint32_t encoder_range = 65536;
    #endif
    #ifdef ENCODER_BIT14
    uint32_t encoder_range = 16384;
    #endif
    #ifdef ENCODER_BIT12
    uint32_t encoder_range = 4096;
    #endif

    fp32 speed_rpm_raw;
    uint32_t delta_theta_meth = Sensor_Msg->series_theta_mech - Sensor_Msg->last_series_theta_mech;
    uint32_t delta_time = 0.001;
    speed_rpm_raw = delta_theta_meth / delta_time * 60 / (2 * PI);

    #ifdef FIRST_ORDER_FILTER
    Sensor_Msg->speed_rpm = First_Order_Filter(speed_rpm_raw);
    #endif
    #ifdef SECOND_ORDER_FILTER
    Sensor_Msg->speed_rpm = Second_Order_Filter(speed_rpm_raw);
    #endif
}


/**
 * ****************************************************************************
 * @fn        Speed_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg) 
 * @version   V1.0.1
 * @date      2023/11/02
 * @brief     ����ٶ�ģʽ����CAN�����ڴ����ٶ�rpm��PID��������õ�Iq_ref��Id_ref��Ϊ0
 * @param     FOC: pointer to a FOC_Typedef structure.
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure.
 * @note      V1.0.1  �����λ����
 * ****************************************************************************
 * @attention None
 * ****************************************************************************
 */
void Speed_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    fp32 speed_set = 0.0f;
    fp32 speed_ref = Sensor_Msg->speed_rpm;

    if(position_limit_flag == FALSE)
    {
        speed_set = recv_msg_fdcan.value;
    }
    else if(position_limit_flag == TRUE)
    {
        if(Sensor_Msg->series_ecd - recv_msg_fdcan.pos_l < DEAD_BAND) {
            speed_set = 0;
        }
        else if(recv_msg_fdcan.pos_h - Sensor_Msg->series_ecd < DEAD_BAND) {
            speed_set = 0;
        }
        else {
            speed_set = recv_msg_fdcan.value;
        }
    }
    
    PID_Calc(&PID_Speed_Loop, speed_ref, speed_set);
    FOC->Curr_Components.Iq_set = PID_Speed_Loop.out;
    FOC->Curr_Components.Id_set = 0;
}


/**
 * ************************************************************************************
 * @fn        Slow_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
 * @version   V1.0.0
 * @date      2023/11/02
 * @brief     �������ģʽ��λ�õ�����
 *            λ�ÿ��Ƶ��ת�ٺ���ʱ����ƽ�����ٷ�������ܴ���ת�Ӳ����򶯵ú�����������
 *            û�����������������䣩
 *            Ϊ�����ٶȻ��ڴ���������λ�ÿ���ʱ����ֻʹ��λ�ú͵�����ɵ�˫�����п��ƣ�
 *            ����λ�û���PID���ƣ�λ�õ�΢�����ٶȣ���Сλ�ÿ��Ƶ��𵴼ӿ�����������������
 *            ��̬��
 * @param     FOC: pointer to a FOC_Typedef structure.
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure.
 * ************************************************************************************
 * @attention None
 * ************************************************************************************
 */
void Slow_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    fp32 position_set = recv_msg_fdcan.value;
    fp32 position_ref = Sensor_Msg->series_ecd;
    fp32 speed_ref = Sensor_Msg->speed_rpm;

    if(position_limit_flag == FALSE)
    {
        PID_Calc(&PID_Slow_Position_Loop, position_ref, position_set);
        FOC->Curr_Components.Iq_set = PID_Slow_Position_Loop.out;
    }
    else if (position_limit_flag == TRUE)
    {
        if(Sensor_Msg->series_ecd - recv_msg_fdcan.pos_l < DEAD_BAND) {
           PID_Calc(&PID_Speed_Loop, speed_ref, 0);
           FOC->Curr_Components.Iq_set = PID_Speed_Loop.out;
        }
        else if(recv_msg_fdcan.pos_h - Sensor_Msg->series_ecd < DEAD_BAND) {
            PID_Calc(&PID_Speed_Loop, speed_ref, 0);
            FOC->Curr_Components.Iq_set = PID_Speed_Loop.out;
        }
        else {
            PID_Calc(&PID_Slow_Position_Loop, position_ref, position_set);
            FOC->Curr_Components.Iq_set = PID_Slow_Position_Loop.out;
        }
    }
    FOC->Curr_Components.Id_set = 0;
}


/**
 * **********************************************************************************
 * @fn        Abs_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg) 
 * @version   V1.0.1
 * @date      2023/11/02
 * @brief     ���Ե��λ��ģʽ����CAN�����ڴ��ĵ������ֵ��PID��������õ��ٶ�Ŀ��ֵ��
 *                            PID��������õ�Iq_ref��Id_ref��Ϊ0
 * @param     FOC: pointer to a FOC_Typedef structure
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure
 * @note      V1.0.1  �����λ����
 * **********************************************************************************
 * @attention None
 * **********************************************************************************
 */
void Abs_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    fp32 position_set = recv_msg_fdcan.value;
    fp32 position_ref = Sensor_Msg->series_ecd;
    fp32 speed_ref = Sensor_Msg->speed_rpm;

    if(position_limit_flag == FALSE)
    {
        PID_Calc(&PID_Position_Loop, position_ref, position_set);
    }
    else if(position_limit_flag == TRUE)
    {
        if(Sensor_Msg->series_ecd - recv_msg_fdcan.pos_l < DEAD_BAND) {
            PID_Position_Loop.out = 0;
        }
        else if(recv_msg_fdcan.pos_h - Sensor_Msg->series_ecd < DEAD_BAND) {
            PID_Position_Loop.out = 0;
        }
        else {
            PID_Calc(&PID_Position_Loop, position_ref, position_set);
        }
    }

    PID_Calc(&PID_Speed_Loop, speed_ref, PID_Position_Loop.out);
    FOC->Curr_Components.Iq_set = PID_Speed_Loop.out;
    FOC->Curr_Components.Id_set = 0;
}


/**
 * **********************************************************************************
 * @fn        Inc_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg) 
 * @version   V1.0.0
 * @date      2023/11/02
 * @brief     �������λ��ģʽ����CAN�������ĵ������ֵ��PID��������õ��ٶ�Ŀ��ֵ��
 *                            PID��������õ�Iq_ref��Id_ref��Ϊ0
 * @param     FOC: pointer to a FOC_Typedef structure
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure
 * **********************************************************************************
 * @attention None
 * **********************************************************************************
 */
void Inc_Position_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    fp32 position_set = recv_msg_fdcan.value + Sensor_Msg->series_ecd;
    fp32 position_ref = Sensor_Msg->series_ecd;
    fp32 speed_ref = Sensor_Msg->speed_rpm;

    if(position_limit_flag == FALSE)
    {
        PID_Calc(&PID_Position_Loop, position_ref, position_set);
    }
    else if(position_limit_flag == TRUE)
    {
        if(Sensor_Msg->series_ecd - recv_msg_fdcan.pos_l < DEAD_BAND) {
            PID_Position_Loop.out = 0;
        }
        else if(recv_msg_fdcan.pos_h - Sensor_Msg->series_ecd < DEAD_BAND) {
            PID_Position_Loop.out = 0;
        }
        else {
            PID_Calc(&PID_Position_Loop, position_ref, position_set);
        }
    }
    PID_Calc(&PID_Speed_Loop, speed_ref, PID_Position_Loop.out);
    FOC->Curr_Components.Iq_set = PID_Speed_Loop.out;
    FOC->Curr_Components.Id_set = 0;
}


/**
 * *****************************************************************************
 * @fn        Torque_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg) 
 * @version   V1.0.1
 * @date      2023/11/02
 * @brief     �������ģʽ����CAN�����ڴ��ĵ���ֵ
 * @param     FOC: pointer to a FOC_Typedef structure.
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure.
 * @note      V1.0.1  �����λ����
 * ****************************************************************************
 * @attention None
 * ****************************************************************************
 */
void Torque_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    if(position_limit_flag == FALSE)
    {
        FOC->Curr_Components.Iq_set = recv_msg_fdcan.value;
    }
    else if(position_limit_flag == TRUE)
    {
        if(Sensor_Msg->series_ecd - recv_msg_fdcan.pos_l < DEAD_BAND) {
            FOC->Curr_Components.Iq_set = PID_Calc(&PID_Speed_Loop, Sensor_Msg->speed_rpm, 0);
        }
        else if(recv_msg_fdcan.pos_h - Sensor_Msg->series_ecd < DEAD_BAND) {
            FOC->Curr_Components.Iq_set = PID_Calc(&PID_Speed_Loop, Sensor_Msg->speed_rpm, 0);
        }
        else {
            FOC->Curr_Components.Iq_set = recv_msg_fdcan.value;
        }
    }
    FOC->Curr_Components.Id_set = 0;
}


/**
 * *********************************************************************************
 * @fn         Open_Torque_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
 * @version    V1.0.0
 * @date       2023/11/03
 * @brief      ��������������CAN�����ڴ��ĵ�ѹֵ
 * @param      FOC: pointer to a FOC_Typedef structure.
 * @param      Sensor_Msg: pointer to a Sensor_Msg_Typedef structure.
 * *********************************************************************************
 * @attention  None
 * *********************************************************************************
 */
void Open_Torque_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    if(position_limit_flag == FALSE)
    {
        FOC->Volt_Components.Vq = recv_msg_fdcan.cmd_vq;
        FOC->Volt_Components.Vd = recv_msg_fdcan.cmd_vd;
    }
}


/**
 * **************************************************************************
 * @fn          Mix_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
 * @version     V1.0.0
 * @date        2023/11/03
 * @brief       �����Ͽ���ģʽ����CAN�����ڴ���λ�ã��ٶȺ�����ֵ
 * @param       FOC: pointer to a FOC_Typedef structure.
 * @param       Sensor_Msg: pointer to a Sensor_Msg structure.
 * **************************************************************************
 * @attention   None
 * **************************************************************************
 */
void Mix_Mode(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg) 
{
    fp32 cmd_pos = recv_msg_fdcan.cmd_pos;
    fp32 cmd_speed = recv_msg_fdcan.cmd_speed;
    fp32 cmd_current = recv_msg_fdcan.cmd_current;

    if(position_limit_flag == FALSE)
    {
        PID_Calc(&PID_Mix_Loop_Pos, Sensor_Msg->series_ecd, cmd_pos);
        PID_Calc(&PID_Mix_Loop_Speed, Sensor_Msg->speed_rpm, cmd_speed);
        FOC->Curr_Components.Iq_set = PID_Mix_Loop_Pos.out + PID_Mix_Loop_Speed.out + cmd_current;
    }
    else if(position_limit_flag == TRUE)
    {
        if(Sensor_Msg->series_ecd - recv_msg_fdcan.pos_l < DEAD_BAND) {
            FOC->Curr_Components.Iq_set = PID_Calc(&PID_Speed_Loop, Sensor_Msg->speed_rpm, 0);
        }
        else if(recv_msg_fdcan.pos_h - Sensor_Msg->series_ecd < DEAD_BAND) {
            FOC->Curr_Components.Iq_set = PID_Calc(&PID_Speed_Loop, Sensor_Msg->speed_rpm, 0);
        }
        else {
            PID_Calc(&PID_Mix_Loop_Pos, Sensor_Msg->series_ecd, cmd_pos);
            PID_Calc(&PID_Mix_Loop_Speed, Sensor_Msg->speed_rpm, cmd_speed);
            FOC->Curr_Components.Iq_set = PID_Mix_Loop_Pos.out + PID_Mix_Loop_Speed.out + cmd_current;
        }
    }
    FOC->Curr_Components.Id_set = 0;
}


/**
 * *****************************************************************************
 * @fn        Position_Limit_Set(Sensor_Msg_Typedef_t *Sensor_Msg) 
 * @version   V1.0.0
 * @date      2023/11/02
 * @brief     �����λ����CAN�������޺�����
 * @param     Sensor_Msg: pointer to a Sensor_Msg_Typedef structure
 * *****************************************************************************
 * @attention None
 * *****************************************************************************
 */
/* void Position_Limit_Set(FOC_Typedef_t *FOC, Sensor_Msg_Typedef_t *Sensor_Msg)
{
    int32_t pos_limit_l = recv_msg_fdcan.pos_l;
    int32_t pos_limit_h = recv_msg_fdcan.pos_h;
    int32_t tmp;

    if(pos_limit_l > pos_limit_h) {
        tmp = pos_limit_h;
        pos_limit_h = pos_limit_l;
        pos_limit_l = tmp;
    }
} */


/**
 * *****************************************************************************
 * @fn        LED_State(uint8_t error_number) 
 * @version   V1.0.1
 * @date      2023/11/02
 * @brief     ����LED����ID��˸�����ϱ�������Ϣ
 * @param     error_number: ������
 * @note      V1.0.0��LED����״̬�뾯��״̬��ͻ
 * *****************************************************************************
 * @attention None
 * *****************************************************************************
*/
void LED_State(uint8_t error_number)
{
    if(led_warning_flag == FALSE) {
        if(error_number != NO_ERROR)
        {
            LED_Error_Toggle();
        }
        else if(error_number == NO_ERROR)
        {
            LED_Toggle(MOTOR_ID1 - MOTOR_CMD_ID);
        }
    }
}
    