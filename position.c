#include "app_user.h"
#include "position.h"

#include "bsp_crc16.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "bsp_rtc.h"

#include <math.h>

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
float s_total_distance_m_f = 0.0f; // 浮点里程累计（m），用于避免整数累加精度损失

// 方向
// 0=停止/未知，1=向上（AD增大），2=向下（AD减小）
static uint8_t  s_last_direction        = 0;

// 外部引用
extern uint16_t alarm_button_or_dwin;
extern gss_device  GSS_device;

extern void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value);

static uint8_t first_run = 1;

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
	
	  LOGT("采样前: g_total_meters=%lu, s_total_distance_m_f=%.3f\n", g_total_meters, s_total_distance_m_f);
    // 0) 获取当前RTC时间（用于速度计算的绝对时间基准）
    bsp_rtc_def now_rtc = {0};
    BSP_RTC_Get(&now_rtc);
    uint32_t now_sod = RTC_SecOfDay(&now_rtc);
		
			if (first_run) 
		{
			g_last_position = ad_now;  // 首帧把last和current都设为实际AD
			g_current_position = ad_now;
			position_diff = 0;
			first_run = 0;
			// 之后的逻辑直接return，也可以不累计
			return;
		}
		
    // 1) 原始计数更新与差值（有符号）
    g_last_position    = g_current_position;
    g_current_position = ad_now;
    position_diff      = (int32_t)g_current_position - (int32_t)g_last_position;

    // 2) 实时位置（米，支持正负）
    //    公式：pos_m = slope * (AD - zero_point) + offset
    GSS_device.position_data_ad    = g_current_position;
		
		//位置小于0置0
		float pos_calc = (float)GSS_device.position_slope *
                                     ( (int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point )
                                     + (float)GSS_device.position_offset;
		
    GSS_device.position_data_real  = pos_calc < 0 ? 0.0f : pos_calc;

    // 3) 当前位移（米）
    //    用标定的 slope 将AD差值转米；里程与速度均按位移的绝对值计（速度为标量）
    float delta_m_signed = (float)position_diff * (float)GSS_device.position_slope;
    float delta_m_abs    = fabsf(delta_m_signed);

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

    // 6) 运行方向（按AD递增递减 + 速度阈值）
    const int32_t DIFF_STOP_THRESH = 2;        // AD微动阈值（可按实际编码器分辨率调整）
    const float   SPEED_STOP_THRESH= 0.005f;   // 速度停止阈值 m/s

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
		
//		LOGT("Modbus采样: AD_now=%lu, delta_m_abs=%.5f, 累计米=%.5f, 总里程mm=%lu, AD_diff=%ld, slope=%.7f, 偏置=%.3f\n",
//    g_current_position,           // 当前AD
//    delta_m_abs,                  // 单帧物理位移（m）
//    s_total_distance_m_f,         // 累加（m）
//    g_total_meters,               // 累加（mm）
//    position_diff,                // AD增量
//    GSS_device.position_slope,    // 最新斜率
//    GSS_device.position_offset    // 最新偏置
		
		LOGT("采样后: g_total_meters=%lu, s_total_distance_m_f=%.3f, 新增里程delta=%.5f\n",
     g_total_meters, s_total_distance_m_f, delta_m_abs);

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
    g_total_meters        = 0;
    s_total_distance_m_f  = 0.0f;
    FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, 0);
}

// 绝对位置（米，带零点和偏置）
float APP_USER_Get_Relative_Position(void)
{
    int32_t ad_diff = (int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point;
    float pos = (float)(GSS_device.position_slope * ad_diff + GSS_device.position_offset);
    return pos < 0 ? 0.0f : pos;
}

// 标定零点（写入闪存）
// 第一次按键：斜率不变；将零点设为当前AD，使实时位置立刻归0（offset相应调整）
void APP_USER_Set_Zero_Point(uint32_t zero_point)
{
    // 1) 设置零点并保存
    GSS_device.position_zero_point = zero_point;
    FLASH_WriteU32_WithCheck(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);

    // 2) 让当前实时位置归零：
    // pos = slope*(AD - zero_point) + offset = 0
    // 由于此时 AD == zero_point，直接令 offset = 0 更直观（与下面计算等价）
    int32_t ad_diff_now = (int32_t)g_current_position - (int32_t)GSS_device.position_zero_point;
    (void)ad_diff_now; // AD应为零
    GSS_device.position_offset = 0.0f;

    // 3) 立即更新设备实时位置为0
    GSS_device.position_data_ad   = g_current_position;
    GSS_device.position_data_real = 0.0f;
	
   	APP_USER_Reset_Total_Meters();
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
// 斜率允许为负；对上下限AD值无特殊要求（仅避免除零）
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
        FLASH_WriteU32_WithCheck(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
        GSS_device.position_offset      = y2;   // 与(AD - zero_point)搭配

        // 立即按新标定刷新实时位置
        GSS_device.position_data_ad   = g_current_position;
        int32_t ad_diff = (int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point;
        GSS_device.position_data_real = (float)GSS_device.position_slope * (float)ad_diff + (float)GSS_device.position_offset;
    }
}

//==================== 简单的闪存写封装 ====================

void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value)
{
    (void)EEPROM_FLASH_WriteU32(addr, value);
	  LOGT("保存到Flash: addr=0x%04X, value=%lu\n", addr, value);
	
}
