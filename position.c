#include "app_user.h"
#include "position.h"

#include "bsp_crc16.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_timer.h"

#include <math.h>

//==================== 基本状态变量（尽量沿用原有变量名） ====================

// 原始计数（AD）
static uint32_t g_last_position         = 0;
uint32_t        g_current_position      = 0;
int32_t         position_diff           = 0;     // 当前帧相对上一帧的AD差值（有符号）

// 采样与速度
static uint32_t s_last_tick             = 0;     // 上一帧tick
static float    g_sample_period_s       = 0.2f;  // 默认采样周期兜底（秒）
static float    g_real_speed            = 0.0f;  // 运行速度（m/s，取绝对值）

// 里程（毫米）
uint32_t        g_total_meters          = 0;     // 累计里程（mm）
static float    s_total_distance_m_f    = 0.0f;  // 浮点里程累计（m），用于避免整数累加精度损失

// 方向
// 0=停止/未知，1=向上（AD增大），2=向下（AD减小）
static uint8_t  s_last_direction        = 0;

// 外部引用
extern uint16_t alarm_button_or_dwin;
extern gss_device  GSS_device;

extern uint32_t HAL_GetTick(void);
extern void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value);

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

static void Modbus_Process_Position_Data(uint32_t ad_now)
{
    // 1) 时间间隔
    uint32_t now_tick = HAL_GetTick();
    float dt_s;
    if (s_last_tick == 0) {
        dt_s = g_sample_period_s;   // 首帧兜底
    } else {
        uint32_t dt_ms = now_tick - s_last_tick;
        dt_s = dt_ms > 0 ? (dt_ms / 1000.0f) : g_sample_period_s; // 防止0除
    }
    s_last_tick = now_tick;

    // 2) 原始计数更新与差值（有符号）
    g_last_position    = g_current_position;
    g_current_position = ad_now;
    position_diff      = (int32_t)g_current_position - (int32_t)g_last_position;

    // 3) 实时位置（米，支持正负）
    //    公式：pos_m = slope * (AD - zero_point) + offset
    GSS_device.position_data_ad    = g_current_position;
    GSS_device.position_data_real  = (float)GSS_device.position_slope *
                                     ( (int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point )
                                     + (float)GSS_device.position_offset;

    // 4) 当前位移（米）
    //    用标定的 slope 将AD差值转米；里程与速度均按位移的绝对值计（速度为标量）
    float delta_m_signed = (float)position_diff * (float)GSS_device.position_slope;
    float delta_m_abs    = fabsf(delta_m_signed);

    // 5) 速度（m/s）= |Δm| / Δt
    g_real_speed               = (dt_s > 0.0f) ? (delta_m_abs / dt_s) : 0.0f;
    GSS_device.real_speed      = g_real_speed;

    // 6) 总里程（mm）
    s_total_distance_m_f      += delta_m_abs;                 // 累计米（浮点）
    g_total_meters             = (uint32_t)(s_total_distance_m_f * 1000.0f); // 转mm整数
    GSS_device.Total_meters    = g_total_meters / 1000;       // 对外显示用“整米”

    // 7) 运行方向（按AD递增递减）
    //    简化：只看position_diff的符号；微动时方向保持上一方向或置0
    const int32_t DIFF_STOP_THRESH = 2;        // AD微动阈值（可按实际编码器分辨率调整）
    const float   SPEED_STOP_THRESH= 0.005f;   // 速度停止阈值 m/s

    uint8_t new_dir = s_last_direction;
    if (position_diff > DIFF_STOP_THRESH) {
        new_dir = 1; // 向上
    } else if (position_diff < -DIFF_STOP_THRESH) {
        new_dir = 2; // 向下
    } else {
        // AD微动范围内，看速度是否接近0
        if (g_real_speed < SPEED_STOP_THRESH) {
            new_dir = 0; // 停止
        }
        // 否则保持上一方向（避免抖动）
    }
    s_last_direction = new_dir;
    GSS_device.run_direction = new_dir;
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
                            // CRC错误：此处可加统计或日志
                        }
                    }
                }
                else if (rx_data[1] == 0x83)
                {
                    // 异常响应：此处可加诊断
                }
            }
        }
    }
}

//==================== 对外接口（保持原函数名） ====================

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
    // 简化后不再维护变化计数，按需实现；
    // 若必须返回，可统计“非零差值帧数”，此处返回0占位。
    return 0;
}

int32_t Modbus_Get_Position_Diff(void)
{
    return position_diff;
}

void Modbus_Reset_Position_Change_Count(void)
{
    // 简化：不维护该计数
}

uint32_t APP_USER_Get_Total_Meters(void)
{
    return g_total_meters;
}

void APP_USER_Reset_Total_Meters(void)
{
    g_total_meters        = 0;
    s_total_distance_m_f  = 0.0f;
    FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, 0);
}

// 绝对位置（米，带零点和偏置）
float APP_USER_Get_Relative_Position(void)
{
    int32_t ad_diff = (int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point;
    return (float)(GSS_device.position_slope * ad_diff + GSS_device.position_offset);
}

// 标定零点（写入闪存）
void APP_USER_Set_Zero_Point(uint32_t zero_point)
{
    GSS_device.position_zero_point = zero_point;
    FLASH_WriteU32_WithCheck(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
}

// 将当前计算结果同步到GSS_device（如需在其它流程调用）
void APP_USER_UpdateDevicePositionAndSpeed(void)
{
    // 实时位置（米）
    GSS_device.position_data_real = APP_USER_Get_Relative_Position();
    // 计数（AD）
    GSS_device.position_data_ad   = g_current_position;
    // 速度（m/s）
    GSS_device.real_speed         = APP_USER_Get_Real_Speed();
}

// 独立的方向更新（若需要周期调用）
void APP_USER_UpdateRunDirection(void)
{
    const int32_t DIFF_STOP_THRESH = 2;
    const float   SPEED_STOP_THRESH= 0.005f;

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
void InitSmartCalibration(void)
{
    float    y1 = (float)GSS_device.position_range_upper;
    float    y2 = (float)GSS_device.position_range_lower;
    uint32_t x1 = GSS_device.position_signal_upper;
    uint32_t x2 = GSS_device.position_signal_lower;

    if (x1 != x2) {
        GSS_device.position_slope  = (y1 - y2) / (float)( (int32_t)x1 - (int32_t)x2 );
        GSS_device.position_offset = y1 - GSS_device.position_slope * (float)x1;
    }
}

//==================== 简单的闪存写封装 ====================

void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value)
{
    (void)EEPROM_FLASH_WriteU32(addr, value);
}
