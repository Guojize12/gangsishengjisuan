#include "sensor.h"
#include "app_user.h" // 为 SENSOR_FIX_VALUE_DWIN 宏提供声明

// 传感器断开检测相关变量
uint8_t sensor_disconnect_count = 0;     // 传感器断开计数器
uint8_t sensor_is_disconnected = 0;      // 传感器断开状态标志（导出供其他模块使用）
uint16_t sensor_stable_count = 0;        // 传感器连续正常计数器（用于解锁显示）

/**
 * @brief  检测传感器是否断开
 * @param  data: 4个通道的ADC数据
 * @retval 1-传感器断开, 0-传感器正常
 * @note   判断依据：多个通道同时出现极端值（接近0或4095）
 */
uint8_t APP_USER_Detect_Sensor_Disconnect(uint16_t data[4])
{
    uint8_t abnormal_count = 0;

    // 检查4个通道中有多少个通道数据异常
    for (uint8_t i = 0; i < 4; i++)
    {
        if (data[i] > SENSOR_DISCONNECT_HIGH_THRESHOLD || data[i] < SENSOR_DISCONNECT_LOW_THRESHOLD)
        {
            abnormal_count++;
        }
    }

    // 如果有3个或以上通道数据异常，判定为传感器可能断开
    if (abnormal_count >= 2)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief  修正传感器断开时的数据
 * @param  data: 4个通道的ADC数据
 * @retval None
 * @note   将所有通道数据修正为2048（12位ADC中间值）
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
 * @retval 1-传感器断开, 0-传感器正常
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