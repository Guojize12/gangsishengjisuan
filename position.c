#include "app_user.h"
#include "position.h"

#include "bsp_crc16.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "bsp_rtc.h"
#include "params_init.h"   // 采用集中化持久化：PARAMS_SaveAll()

#include <math.h>
#include <string.h>

//==================== 基本状态变量（尽量沿用原有变量名） ====================

// 原始计数（AD）
static uint32_t g_last_position         = 0;
uint32_t        g_current_position      = 0;
int32_t         position_diff           = 0;     // 当前帧相对上一帧的AD差值（有符号）

// 速度（使用RTC按物理时间计算）
static float    g_real_speed            = 0.0f;  // 运行速度（m/s，取绝对值）
static bsp_rtc_def s_last_speed_rtc     = {0};   // 上次用于速度计算的RTC时间
static uint32_t    s_last_speed_ad      = 0;     // 上次用于速度计算的AD

// 里程（毫米）
uint32_t        g_total_meters          = 0;     // 累计里程（mm）
float           s_total_distance_m_f    = 0.0f;  // 浮点里程累计（m），用于避免整数累加精度损失

// 方向
// 0=停止/未知，1=向上（AD增大），2=向下（AD减小）
static uint8_t  s_last_direction        = 0;

// 外部引用
extern uint16_t alarm_button_or_dwin;
extern gss_device  GSS_device;

//==================== 集中化存储触发策略 ====================
// 位置不动时，1小时存一次；里程变动超过3米时，存一次
static uint32_t last_flash_save_time_ms     = 0;
static float    last_flash_saved_distance_m = 0.0f;
static uint32_t last_motion_time_ms         = 0;
// 判定“有运动”的最小位移（m）
static const float MOTION_EPSILON_M         = 0.01f;

// 降低方向检测灵敏度（阈值提高）
#define DIFF_STOP_THRESH   8          // AD微动阈值（越大越不敏感）
#define SPEED_STOP_THRESH  0.02f      // 停止阈值 m/s（越大越不敏感）

//==================== Modbus - 发送读取命令 ====================

void Modbus_Send_ReadCmd(void)
{
    uint8_t  cmd[8];
    uint8_t  dev_addr = 0x01;
    uint16_t reg_addr = 0x0000;   // 32位位置寄存器（两寄存器）
    uint16_t reg_num  = 0x0002;

    cmd[0] = dev_addr;
    cmd[1] = 0x03;
    cmd[2] = (reg_addr >> 8) & 0xFF;
    cmd[3] = (reg_addr     ) & 0xFF;
    cmd[4] = (reg_num  >> 8) & 0xFF;
    cmd[5] = (reg_num      ) & 0xFF;

    uint16_t crc = bsp_crc16(cmd, 6);
    cmd[6] = (crc >> 8) & 0xFF;   // 注意：与BSP约定的CRC大小端一致
    cmd[7] = (crc     ) & 0xFF;

    BSP_GPIO_Set(0xB5, 1);        // RS485发送
    BSP_UART_Transmit(APP_MODBUS_UART, cmd, 8);
    BSP_GPIO_Set(0xB5, 0);        // RS485接收
}

//==================== 基础计算核心 ====================

static inline uint32_t RTC_SecOfDay(const bsp_rtc_def* t)
{
    return (uint32_t)t->hour * 3600u + (uint32_t)t->minute * 60u + (uint32_t)t->second;
}

static void Modbus_Process_Position_Data(uint32_t ad_now)
{
    // 0) 获取当前RTC时间（用于速度计算的绝对时间基准）与毫秒时基
    bsp_rtc_def now_rtc = {0};
    BSP_RTC_Get(&now_rtc);
    uint32_t now_sod = RTC_SecOfDay(&now_rtc);
    uint32_t now_ms  = HAL_GetTick();

    // 1) 原始计数更新与差值（有符号）
    g_last_position    = g_current_position;
    g_current_position = ad_now;
    position_diff      = (int32_t)g_current_position - (int32_t)g_last_position;

    // 2) 实时位置（米，支持正负）
    //    公式：pos_m = slope * (AD - zero_point) + offset
    GSS_device.position_data_ad    = g_current_position;

    // 位置小于0置0
    float pos_calc = (float)GSS_device.position_slope *
                     ((int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point)
                     + (float)GSS_device.position_offset;

    GSS_device.position_data_real  = pos_calc < 0 ? 0.0f : pos_calc;

    // 3) 当前位移（米）
    //    用标定的 slope 将AD差值转米；里程与速度均按位移的绝对值计（速度为标量）
    float delta_m_signed = (float)position_diff * (float)GSS_device.position_slope;
    float delta_m_abs    = fabsf(delta_m_signed);

    // 更新“是否有运动”的时间戳
    if (delta_m_abs > MOTION_EPSILON_M)
        last_motion_time_ms = now_ms;

    // 4) 速度（m/s）——使用RTC按物理时间计算，每满1秒且AD有变化才更新
    if ((s_last_speed_rtc.hour | s_last_speed_rtc.minute | s_last_speed_rtc.second) == 0)
    {
        // 首次初始化：建立速度计算基准
        s_last_speed_rtc = now_rtc;
        s_last_speed_ad  = ad_now;
        g_real_speed     = 0.0f;
    }
    else
    {
        uint32_t last_sod = RTC_SecOfDay(&s_last_speed_rtc);
        // 处理跨午夜：若now比last小，视为加一天
        uint32_t dt_s = (now_sod >= last_sod) ? (now_sod - last_sod)
                                              : (now_sod + 86400u - last_sod);

        if ((dt_s >= 1u) && (s_last_speed_ad != ad_now))
        {
            int32_t ad_diff_for_speed = (int32_t)ad_now - (int32_t)s_last_speed_ad;
            float   delta_m_for_speed = fabsf((float)ad_diff_for_speed * (float)GSS_device.position_slope);
            g_real_speed = delta_m_for_speed / (float)dt_s;

            // 更新速度计算基准
            s_last_speed_rtc = now_rtc;
            s_last_speed_ad  = ad_now;
        }
        // 未满1秒或AD未变化：保持上一次速度不变
    }
    GSS_device.real_speed = g_real_speed;

    // 5) 总里程（mm）
    s_total_distance_m_f      += delta_m_abs;                 // 累计米（浮点）
    g_total_meters             = (uint32_t)(s_total_distance_m_f * 1000.0f); // 转mm整数
    GSS_device.Total_meters    = g_total_meters / 1000;       // 对外显示用“整米”

    // 6) 运行方向（按AD递增递减 + 速度阈值，降低灵敏度）
    uint8_t new_dir = s_last_direction;
    if (position_diff > DIFF_STOP_THRESH) {
        new_dir = 1; // 向上
    } else if (position_diff < -DIFF_STOP_THRESH) {
        new_dir = 2; // 向下
    } else {
        if (g_real_speed < SPEED_STOP_THRESH) {
            new_dir = 0; // 停止
        }
    }
    s_last_direction = new_dir;
    GSS_device.run_direction = new_dir;

    // 7) 根据策略决定是否落盘（集中化）
    APP_USER_Mileage_Flash_Save_Handle();
}

//==================== Modbus接收处理 ====================

void Modbus_Rec_Handle(void)
{
    if (BSP_UART_Rec_Read(APP_MODBUS_UART) == USR_EOK)
    {
        if (APP_MODBUS_UART_BUF.rxLen >= 7)
        {
            uint8_t *rx_data = APP_MODBUS_UART_BUF.rxBuf;
            uint16_t rx_len  = APP_MODBUS_UART_BUF.rxLen;

            if (rx_data[0] == 0x01)
            {
                if (rx_data[1] == 0x03)
                {
                    uint8_t data_len = rx_data[2];
                    if (data_len == 4 && rx_len == (3 + data_len + 2))
                    {
                        uint16_t calc_crc = bsp_crc16(rx_data, rx_len - 2);
                        uint16_t recv_crc = (rx_data[rx_len - 2] << 8) | rx_data[rx_len - 1];

                        if (calc_crc == recv_crc)
                        {
                            uint32_t position = (rx_data[3] << 24) | (rx_data[4] << 16) |
                                                (rx_data[5] << 8)  |  rx_data[6];
                            Modbus_Process_Position_Data(position);
                        }
                        else
                        {
                            // CRC错误
                        }
                    }
                }
                else if (rx_data[1] == 0x83)
                {
                    // 异常响应
                }
            }
        }
    }
}

//==================== 对外接口 ====================

float APP_USER_Get_Real_Speed(void)
{
    return g_real_speed;
}

uint32_t Modbus_Get_Current_Position(void)
{
    return g_current_position;
}

uint32_t Modbus_Get_Last_Position(void)
{
    return g_last_position;
}

uint32_t Modbus_Get_Position_Change_Count(void)
{
    return 0;
}

int32_t Modbus_Get_Position_Diff(void)
{
    return position_diff;
}

void Modbus_Reset_Position_Change_Count(void)
{
}

uint32_t APP_USER_Get_Total_Meters(void)
{
    return g_total_meters;
}

void APP_USER_Reset_Total_Meters(void)
{
    // 仅操作RAM，然后集中化落盘
    g_total_meters        = 0;
    s_total_distance_m_f  = 0.0f;

    // 更新“上次保存的里程”基准，避免紧接着再次触发3m策略
    last_flash_saved_distance_m = 0.0f;
    last_flash_save_time_ms     = HAL_GetTick();
    last_motion_time_ms         = HAL_GetTick();

    PARAMS_SaveAll();
}

// 绝对位置（米，带零点和偏置）
float APP_USER_Get_Relative_Position(void)
{
    int32_t ad_diff = (int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point;
    float pos = (float)(GSS_device.position_slope * ad_diff + GSS_device.position_offset);
    return pos < 0 ? 0.0f : pos;
}

// 标定零点（集中化持久化）
// 第一次按键：斜率不变；将零点设为当前AD，使实时位置立刻归0（offset相应调整）
void APP_USER_Set_Zero_Point(uint32_t zero_point)
{
    // 1) 设置零点并保存到RAM
    GSS_device.position_zero_point = zero_point;

    // 2) 让当前实时位置归零：
    // pos = slope*(AD - zero_point) + offset = 0
    // 由于此时 AD == zero_point，直接令 offset = 0 更直观
    GSS_device.position_offset = 0.0f;

    // 3) 立即更新设备实时位置为0
    GSS_device.position_data_ad   = g_current_position;
    GSS_device.position_data_real = 0.0f;

    // 4) 里程清零（与现场操作预期一致）
    APP_USER_Reset_Total_Meters();

    // 5) 集中化落盘
    PARAMS_SaveAll();
}

// 将当前计算结果同步到GSS_device（如需在其它流程调用）
void APP_USER_UpdateDevicePositionAndSpeed(void)
{
    GSS_device.position_data_real = APP_USER_Get_Relative_Position();
    GSS_device.position_data_ad   = g_current_position;
    GSS_device.real_speed         = APP_USER_Get_Real_Speed();
}

// 独立的方向更新（若需要周期调用）
void APP_USER_UpdateRunDirection(void)
{
    uint8_t new_dir = s_last_direction;
    if (position_diff > DIFF_STOP_THRESH) {
        new_dir = 1;
    } else if (position_diff < -DIFF_STOP_THRESH) {
        new_dir = 2;
    } else {
        if (g_real_speed < SPEED_STOP_THRESH) {
            new_dir = 0;
        }
    }
    s_last_direction = new_dir;
    GSS_device.run_direction = new_dir;
}

// 智能标定初始化（用两点求斜率与偏置）
// 与位置公式 pos = slope*(AD - zero_point) + offset 完全一致：
//   zero_point = x2（下限AD），offset = y2（下限物理值）
void InitSmartCalibration(void)
{
    float    y1 = (float)GSS_device.position_range_upper;
    float    y2 = (float)GSS_device.position_range_lower;
    uint32_t x1 = GSS_device.position_signal_upper;
    uint32_t x2 = GSS_device.position_signal_lower;

    if (x1 != x2) {
        GSS_device.position_slope       = (y1 - y2) / (float)((int32_t)x1 - (int32_t)x2);
        GSS_device.position_zero_point  = x2;   // 锚定到下限AD
        GSS_device.position_offset      = y2;   // 与(AD - zero_point)搭配

        // 立即按新标定刷新实时位置
        GSS_device.position_data_ad   = g_current_position;
        int32_t ad_diff = (int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point;
        GSS_device.position_data_real = (float)GSS_device.position_slope * (float)ad_diff + (float)GSS_device.position_offset;

        // 集中化持久化
        PARAMS_SaveAll();
    }
}

//==================== 集中化里程/位置落盘策略实现 ====================

void APP_USER_Mileage_Flash_Save_Handle(void)
{
    uint32_t now_ms          = HAL_GetTick();
    float    curr_distance_m = s_total_distance_m_f;
    int      need_save       = 0;

    // 1) 里程超过3米
    if ((curr_distance_m - last_flash_saved_distance_m) >= 3.0f) {
        need_save = 1;
    }

    // 2) 位置不动达1小时（无运动判定：last_motion_time_ms距离当前超过1小时）
    if ((now_ms - last_motion_time_ms) >= (3600UL * 1000UL)) {
        // 避免重复每次都写：也要求距离上次保存超过1小时
        if ((now_ms - last_flash_save_time_ms) >= (3600UL * 1000UL)) {
            need_save = 1;
        }
    }

    if (need_save) {
        PARAMS_SaveAll();
        last_flash_save_time_ms     = now_ms;
        last_flash_saved_distance_m = curr_distance_m;
    }
}

//==================== 兼容旧接口（内部转为集中化保存） ====================
// 注意：app_user.c 中仍可能调用该函数（例如二次按键后保存里程）；此处不再分散写Flash。
void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value)
{
    (void)addr;
    (void)value;
    // 统一走集中化保存（PARAMS_SaveAll内有memcmp保护，无变化不写）
    PARAMS_SaveAll();
}

