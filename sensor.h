#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

// 传感器断开检测相关变量与阈值
#define SENSOR_DISCONNECT_HIGH_THRESHOLD  4000
#define SENSOR_DISCONNECT_LOW_THRESHOLD   100
#define SENSOR_DISCONNECT_DETECT_COUNT    50
#define SENSOR_FIX_VALUE                  2048
#define SENSOR_DISPLAY_LOCK_THRESHOLD     2
#define SENSOR_DISPLAY_UNLOCK_THRESHOLD   100

// 供其他模块访问的状态量
extern uint8_t  sensor_disconnect_count; // 断开检测计数
extern uint8_t  sensor_is_disconnected;  // 断开状态
extern uint16_t sensor_stable_count;     // 连续正常计数

// 拆分出的接口
uint8_t APP_USER_Detect_Sensor_Disconnect(uint16_t data[4]);
void    APP_USER_Fix_Disconnected_Sensor_Data(uint16_t data[4]);

// 保持原有对外接口
uint8_t APP_USER_Is_Sensor_Disconnected(void);
void    APP_USER_Fix_Sensor_Data_For_DWIN(uint16_t data[4]);

// 新增：更新最新AD数据到GSS_device（并计算电压）
void    APP_SENSOR_Update_From_Buffer(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3);

#endif
