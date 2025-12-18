#include "app_user.h"   // ENCODER_COUNTS_PER_REV、类型定义
#include "position.h"

// 精确匹配 BSP 的声明
#include "bsp_crc16.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_timer.h"

// 位置数据相关变量
static uint32_t g_last_position        = 0;
uint32_t        g_current_position     = 0;
static uint32_t g_position_change_count= 0;
static float    g_postion               = 0.0f;
uint32_t        g_total_meters         = 0;        // 毫米累计

/******** 绝对值编码器的统一周期与系数（仅用于计算，不改DWIN） ********/
static float    g_sample_period_s         = 0.2f;    // 200ms
static float    g_position_scale_m_per_rev= 0.3f;    // 每圈0.3米
static float    g_encoder_max_step_m      = 3.0f;    // 跳变保护阈值（米）
static uint32_t g_mileage_save_step_m     = 10;      // 里程步进保存（米）
static uint32_t g_mileage_save_interval_ms= 60*60*1000; // 至少每小时保存
static uint32_t s_last_save_meters        = 0;
static uint32_t s_last_save_tick          = 0;

float           g_max_position            = 0;
float           g_min_position            = 0;

int32_t         position_diff             = 0;       // 始终计算位置差
static float    g_real_speed              = 0.0f;    // 实时速度（m/s）

extern uint16_t alarm_button_or_dwin;
extern gss_device  GSS_device;

extern uint32_t HAL_GetTick(void);
extern void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value);

// 发送Modbus RTU读取命令
void Modbus_Send_ReadCmd(void)
{
    uint8_t cmd[8];
    uint8_t  dev_addr = 0x01;        // 从机地址
    uint16_t reg_addr = 0x0000;      // 位置寄存器地址（32位）
    uint16_t reg_num  = 0x0002;      // 读取2个寄存器

    cmd[0] = dev_addr;
    cmd[1] = 0x03;
    cmd[2] = (reg_addr >> 8) & 0xFF;
    cmd[3] = (reg_addr     ) & 0xFF;
    cmd[4] = (reg_num  >> 8) & 0xFF;
    cmd[5] = (reg_num      ) & 0xFF;

    uint16_t crc = bsp_crc16(cmd, 6);
    cmd[6] = (crc >> 8) & 0xFF;
    cmd[7] = (crc     ) & 0xFF;

    BSP_GPIO_Set(0xB5, 1);              // RS485发送模式
    BSP_UART_Transmit(APP_MODBUS_UART, cmd, 8);
    BSP_GPIO_Set(0xB5, 0);              // RS485接收模式
}

extern uint32_t stop_time;

// 接收并处理位置数据（里程累计、速度计算、跳变保护）
void Modbus_Process_Position_Data(uint32_t position)
{
    // 更新位置
    g_last_position     = g_current_position;
    g_current_position  = position;

    // 位置差
    position_diff = (int32_t)g_current_position - (int32_t)g_last_position;

    // 变化计数
    if (g_current_position != g_last_position)
    {
        g_position_change_count++;
    }

    // 本次位移（米）
    int32_t abs_diff = (position_diff < 0) ? -position_diff : position_diff;
    float   delta_m  = ((float)abs_diff / (float)ENCODER_COUNTS_PER_REV) * g_position_scale_m_per_rev;

    // 单步跳变保护
    if (delta_m > g_encoder_max_step_m)
    {
        // 本次位移未计入里程，速度也按跳变忽略
        return;
    }

    // 累计总里程（转换为毫米）
    g_total_meters += (uint32_t)(delta_m * 1000.0f);

    // 记录最大/最小位置（米）
    float current_position_m = ((float)g_current_position / (float)ENCODER_COUNTS_PER_REV) * g_position_scale_m_per_rev;
    if (current_position_m > g_max_position) g_max_position = current_position_m;
    if (g_min_position == 0 || current_position_m < g_min_position) g_min_position = current_position_m;

    // 同步显示用总里程（米）
    GSS_device.Total_meters = g_total_meters / 1000;

    // 实时速度（m/s）= delta_m / 采样周期
    g_real_speed = delta_m / g_sample_period_s;

    // 里程保存策略
    APP_USER_Mileage_Flash_Save_Handle();
}

// 获取实时速度
float APP_USER_Get_Real_Speed(void)
{
    return g_real_speed;
}

// Modbus接收处理
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
                else if (rx_data[1] == 0x83) // 异常响应
                {
                    // 异常处理
                }
            }
        }
    }
}

// 获取当前位置值
uint32_t Modbus_Get_Current_Position(void)
{
    return g_current_position;
}

// 获取上次位置值
uint32_t Modbus_Get_Last_Position(void)
{
    return g_last_position;
}

// 获取位置变化次数
uint32_t Modbus_Get_Position_Change_Count(void)
{
    return g_position_change_count;
}

// 获取位置变化差值
int32_t Modbus_Get_Position_Diff(void)
{
    return (int32_t)g_current_position - (int32_t)g_last_position;
}

// 重置位置变化计数
void Modbus_Reset_Position_Change_Count(void)
{
    g_position_change_count = 0;
}

// 获取总里程（毫米）
uint32_t APP_USER_Get_Total_Meters(void)
{
    return g_total_meters;
}

// 重置总里程
void APP_USER_Reset_Total_Meters(void)
{
    g_total_meters = 0;
    g_postion      = 0.0f;

    FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, 0);
}

// 相对位置（米）
float APP_USER_Get_Relative_Position(void)
{
    if (alarm_button_or_dwin == 0)
    {
        return ((int32_t)g_current_position - (int32_t)GSS_device.position_zero_point)
               * (g_position_scale_m_per_rev / (float)ENCODER_COUNTS_PER_REV);
    }
    else if (alarm_button_or_dwin == 1)
    {
        return (float)(GSS_device.position_slope * (float)GSS_device.position_data_ad + GSS_device.position_offset);
    }
    else
    {
        alarm_button_or_dwin = 0;
        return ((int32_t)g_current_position - (int32_t)GSS_device.position_zero_point)
               * (g_position_scale_m_per_rev / (float)ENCODER_COUNTS_PER_REV);
    }
}

// 设置标定零点
void APP_USER_Set_Zero_Point(uint32_t zero_point)
{
    GSS_device.position_zero_point = zero_point;
    FLASH_WriteU32_WithCheck(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
}

/******** 关键参数写入与里程保存策略 ********/
void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value)
{
    (void)EEPROM_FLASH_WriteU32(addr, value);
}

void APP_USER_Mileage_Flash_Save_Handle(void)
{
    uint32_t now        = HAL_GetTick();
    uint32_t meters_now = g_total_meters / 1000; // 转整米比较

    // 步进触发
    if ((meters_now - s_last_save_meters) >= g_mileage_save_step_m)
    {
        FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);
        s_last_save_meters = meters_now;
        s_last_save_tick   = now;
        return;
    }

    // 定时触发（兜底）
    if ((now - s_last_save_tick) >= g_mileage_save_interval_ms)
    {
        FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);
        s_last_save_meters = meters_now;
        s_last_save_tick   = now;
        return;
    }
}
