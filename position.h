#ifndef POSITION_H
#define POSITION_H

#include <stdint.h>

// 导出运行时关键变量供其他模块使用
extern uint32_t g_current_position;
extern int32_t  position_diff;
extern uint32_t g_total_meters;        // 供 loadini 初始化
extern float    s_total_distance_m_f;

// 位置/里程/Modbus 相关接口
void     Modbus_Send_ReadCmd(void);
void     Modbus_Rec_Handle(void);
void     Modbus_Process_Position_Data(uint32_t position);

uint32_t Modbus_Get_Current_Position(void);
uint32_t Modbus_Get_Last_Position(void);
uint32_t Modbus_Get_Position_Change_Count(void);
int32_t  Modbus_Get_Position_Diff(void);
void     Modbus_Reset_Position_Change_Count(void);

uint32_t APP_USER_Get_Total_Meters(void);
void     APP_USER_Reset_Total_Meters(void);

float    APP_USER_Get_Relative_Position(void);
void     APP_USER_Set_Zero_Point(uint32_t zero_point);

// 速度获取接口（速度计算在 position 模块）
float    APP_USER_Get_Real_Speed(void);

// 里程保存（跨越10米立即保存；小时兜底保存由内部tick也会处理）
void     APP_USER_Mileage_Flash_Save_Handle(void);
void     FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value);

void     APP_USER_UpdateDevicePositionAndSpeed(void);
void     APP_USER_UpdateRunDirection(void);

void     InitSmartCalibration(void);

#endif
