#include "Filter.h"

fp32 speed_out;
fp32 alpha;

/**
 * ************************************************************************
 * @fn       Filter_Init(uint8_t type)
 * @version  V1.0.0
 * @date     2023/10/21
 * @brief    初始化滤波器类型
 * @param    type: 滤波器类型
 * ************************************************************************
 * @attention NONE
 * ************************************************************************
 */
void Filter_Init(uint8_t type)
{
    switch(type)
    {
        case FIRST_ORDER:
        case SECOND_ORDER:
        {
            alpha = ALPHA;
            break;
        }        
        case KALMAN_FILTER:
        break;
        default:break;
    }
}
/**
 * ************************************************************************
 * @fn       First_Order_Filter(fp32 speed_in, fp32 alpha)
 * @version  V1.0.0
 * @date     2023/10/21
 * @brief    一阶低通滤波器
 * @param    speed_in：输入速度
 * @return   speed_out：输出速度
 * ***********************************************************************
 * @attention \alpha = Ts / (Ts + \frac{1}{2 * \pi * fc})
 * ***********************************************************************
 */
fp32 First_Order_Filter(fp32 speed_in)
{
    speed_out = speed_in * alpha + (1.0f - alpha) * speed_out;
    return speed_out;
}

/**
 * ************************************************************************
 * @fn       Second_Order_Filter(fp32 speed_in)
 * @version  V1.0.0
 * @date     2023/10/21
 * @brief    二阶低通滤波器
 * @param    speed_in：输入速度
 * @return   speed_out：输出速度
 * ************************************************************************
 * @attention \alpha = Ts / (Ts + \frac{1}{2 * \pi * fc})
 * ************************************************************************
 */
fp32 Second_Order_Filter(fp32 speed_in)
{
    fp32 speed_out_first_order = First_Order_Filter(speed_in);
    speed_out = First_Order_Filter(speed_out_first_order);
    return speed_out;
}

