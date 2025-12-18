#include "sensor.h"
#include "app_user.h" // 为 SENSOR_FIX_VALUE_DWIN、GSS_device 提供声明

// 传感器断开检测相关变量
uint8_t  sensor_disconnect_count = 0;     // 传感器断开计数器
uint8_t  sensor_is_disconnected  = 0;     // 传感器断开状态标志
uint16_t sensor_stable_count     = 0;     // 连续正常计数器（用于解锁显示）

/**
 * @brief  检测传感器是否断开
 * @param  data: 4个通道的ADC数据
 * @retval 1-断开, 0-正常
 */
uint8_t APP_USER_Detect_Sensor_Disconnect(uint16_t data[4])
{
    uint8_t abnormal_count = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (data[i] > SENSOR_DISCONNECT_HIGH_THRESHOLD || data[i] < SENSOR_DISCONNECT_LOW_THRESHOLD)
        {
            abnormal_count++;
        }
    }

    // 如果>=2路通道异常，判定为传感器可能断开
    return (abnormal_count >= 2) ? 1 : 0;
}

/**
 * @brief  修正传感器断开时的数据
 * @param  data: 4个通道的ADC数据
 */
void APP_USER_Fix_Disconnected_Sensor_Data(uint16_t data[4])
{
    for (uint8_t i = 0; i < 4; i++)
    {
        data[i] = SENSOR_FIX_VALUE;
    }
}

/**
 * @brief  获取传感器断开状态
 * @retval 1-断开, 0-正常
 */
uint8_t APP_USER_Is_Sensor_Disconnected(void)
{
    return sensor_is_disconnected;
}

/**
 * @brief  修正DWIN传感器数据（传感器断开时）
 * @param  data: 4个通道的ADC数据
 */
void APP_USER_Fix_Sensor_Data_For_DWIN(uint16_t data[4])
{
    for (uint8_t i = 0; i < 4; i++)
    {
        data[i] = SENSOR_FIX_VALUE_DWIN;
    }
}

/**
 * @brief  更新最新AD数据到GSS_device（并计算电压）
 */
void APP_SENSOR_Update_From_Buffer(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3)
{
    // 原始AD
    GSS_device.hall_ad[0] = ch0;
    GSS_device.hall_ad[1] = ch1;
    GSS_device.hall_ad[2] = ch2;
    GSS_device.hall_ad[3] = ch3;

    // 电压换算（单位：mV）
    GSS_device.hall_v[0] = ((float)GSS_device.hall_ad[0] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[1] = ((float)GSS_device.hall_ad[1] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[2] = ((float)GSS_device.hall_ad[2] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[3] = ((float)GSS_device.hall_ad[3] * 3300.0f) / 4095.0f;
}
