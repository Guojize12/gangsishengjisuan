#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

// 传感器断开检测相关变量与阈值
#define SENSOR_DISCONNECT_HIGH_THRESHOLD  4000   // 高阈值（接近满量程4095）
#define SENSOR_DISCONNECT_LOW_THRESHOLD   100    // 低阈值（接近0）
#define SENSOR_DISCONNECT_DETECT_COUNT    50     // 连续检测到50次异常才判定为断开
#define SENSOR_FIX_VALUE                  2048   // 传感器断开时的固定输出值（12位ADC中间值）
#define SENSOR_DISPLAY_LOCK_THRESHOLD     2      // 显示锁定阈值：检测到1次异常就立即锁定
#define SENSOR_DISPLAY_UNLOCK_THRESHOLD   100    // 显示解锁阈值：需要连续100次正常才解锁

// 供其他模块访问的状态量
extern uint8_t sensor_disconnect_count; // 断开检测计数
extern uint8_t sensor_is_disconnected;  // 断开状态
extern uint16_t sensor_stable_count;    // 连续正常计数

// 拆分出的接口（原本为 app_user.c 内部静态函数）
uint8_t APP_USER_Detect_Sensor_Disconnect(uint16_t data[4]);
void    APP_USER_Fix_Disconnected_Sensor_Data(uint16_t data[4]);

// 保持原有对外接口
uint8_t APP_USER_Is_Sensor_Disconnected(void);
void    APP_USER_Fix_Sensor_Data_For_DWIN(uint16_t data[4]);

#endif
