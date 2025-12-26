#include "app_user.h"
#include "position.h"

#include "bsp_crc16.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "bsp_rtc.h"

#include <math.h>
#include <string.h>

//==================== 配置参数（可按实际设备调整） ====================

// 启停滞回阈值（降低从0开始的灵敏度）
#define START_AD_DIFF_THRESH        5        // 启动时AD差阈值（更高，避免轻微抖动就判定为运动）
#define STOP_AD_DIFF_THRESH         2        // 停止时AD差阈值（更低，避免频繁抖动）

#define START_SPEED_THRESH          0.020f   // 启动时速度阈值（m/s）
#define STOP_SPEED_THRESH           0.005f   // 停止时速度阈值（m/s）

// 速度平滑（IIR）系数：new = alpha*old + (1-alpha)*measured
#define SPEED_FILTER_ALPHA          0.30f

// 里程累计防抖与异常防护
#define MIN_VALID_AD_STEP           2        // 小于该AD步数认为是抖动，不计入里程
#define MAX_VALID_AD_STEP           200000u  // 单帧最大允许AD步数（防止异常跳变），按实际传感器配置

// 静止强制归零与里程定时保存
#define STOP_IDLE_TIMEOUT_S         3u       // 超过该秒数未检测到有效位移，强制速度/方向归零
#define MILEAGE_SAVE_UNIT_MM        10000u   // 每10米（10000mm）保存一次
#define MILEAGE_SAVE_IDLE_S         (3600u)  // 未达10米，1小时也保存一次

//==================== 基本状态变量 ====================

// 原始计数（AD）
static uint32_t g_last_position         = 0;
uint32_t        g_current_position      = 0;
int32_t         position_diff           = 0;     // 当前帧相对上一帧的AD差值（有符号）

// 速度（使用RTC按物理时间计算）
static float        g_real_speed        = 0.0f;  // 运行速度（m/s，标量，经过平滑）
static bsp_rtc_def  s_last_speed_rtc    = {0};   // 上次用于速度计算的RTC时间
static uint32_t     s_last_speed_ad     = 0;     // 上次用于速度计算的AD

// 里程（毫米/米）
uint32_t        g_total_meters          = 0;     // 累计里程（mm）
float           s_total_distance_m_f    = 0.0f;  // 浮点里程累计（m），避免整数累加精度损失

// 方向：0=停止/未知，1=向上（AD增大），2=向下（AD减小）
static uint8_t  s_last_direction        = 0;

// 动作/保存相关的时间戳（秒/日）
static uint32_t s_last_move_sod         = 0;     // 最近一次检测到“有效位移”的秒计数
static uint32_t s_last_mileage_save_sod = 0;     // 最近一次里程写Flash的秒计数
static uint32_t s_last_saved_meters_mm  = 0;     // 最近一次写Flash时的里程（mm）

// 外部引用
extern uint16_t  alarm_button_or_dwin;
extern gss_device GSS_device;

extern void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value);

// 首帧标记
static uint8_t first_run = 1;

//==================== 内部工具函数 ====================

static inline uint32_t RTC_SecOfDay(const bsp_rtc_def* t)
{
    return (uint32_t)t->hour * 3600u + (uint32_t)t->minute * 60u + (uint32_t)t->second;
}

// 统一获取当前秒计数
static inline uint32_t Now_SecOfDay(void)
{
    bsp_rtc_def now = {0};
    BSP_RTC_Get(&now);
    return RTC_SecOfDay(&now);
}

// 静止与里程保存后台处理（应被周期调用；本模块在接收处理末尾也会主动调用）
static void Position_Idle_Tick(void)
{
    uint32_t now_sod = Now_SecOfDay();

    // 强制静止判定：长时间未检测到有效位移，速度/方向清零
    if (s_last_move_sod != 0 && (uint32_t)(now_sod - s_last_move_sod) >= STOP_IDLE_TIMEOUT_S) {
        if (g_real_speed != 0.0f || s_last_direction != 0) {
            g_real_speed = 0.0f;
            s_last_direction = 0;
            GSS_device.real_speed = 0.0f;
            GSS_device.run_direction = 0;
            // LOGT("静止超时: 强制速度与方向归零\n");
        }
    }

    // 里程保存后台处理
    // 需求：每10米保存一次；不到10米或静止不动1小时保存一次
    // 实现：若总里程跨越10米边界则保存；否则若距离上次保存已超过1小时也保存一次（哪怕未改变或不足10米）
    if (s_last_mileage_save_sod == 0) {
        s_last_mileage_save_sod = now_sod; // 初始化
    }

    // 每小时兜底保存
    if ((uint32_t)(now_sod - s_last_mileage_save_sod) >= MILEAGE_SAVE_IDLE_S) {
        if (g_total_meters != s_last_saved_meters_mm) {
            FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);
            s_last_saved_meters_mm  = g_total_meters;
        } else {
            // 即便数值未变化，也按需求记录一次
            FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);
        }
        s_last_mileage_save_sod = now_sod;
    }
}

// 里程保存：在每次计算后调用，满足10米就写一次；小时兜底在 Position_Idle_Tick 处理
void APP_USER_Mileage_Flash_Save_Handle(void)
{
    uint32_t now_sod = Now_SecOfDay();

    // 首次调用初始化保存参考
    if (s_last_mileage_save_sod == 0) {
        s_last_mileage_save_sod = now_sod;
    }
    if (s_last_saved_meters_mm == 0 && g_total_meters != 0) {
        // 如果外部已从Flash恢复了g_total_meters，这里同步一次
        s_last_saved_meters_mm = g_total_meters;
    }

    // 每跨越10米边界，保存一次
    uint32_t cur_decade = g_total_meters / MILEAGE_SAVE_UNIT_MM;
    uint32_t last_decade = s_last_saved_meters_mm / MILEAGE_SAVE_UNIT_MM;

    if (cur_decade != last_decade) {
        if (g_total_meters != s_last_saved_meters_mm) {
            FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);
            s_last_saved_meters_mm  = g_total_meters;
            s_last_mileage_save_sod = now_sod;
            // LOGT("总里程跨越%u0米，写入Flash: %lu mm\n", (unsigned)MILEAGE_SAVE_UNIT_MM/1000, g_total_meters);
        }
    }
}

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

static void Update_Direction_With_Hysteresis(int32_t ad_diff, float speed_now)
{
    uint8_t new_dir = s_last_direction;

    if (s_last_direction == 0) {
        // 当前静止：需要达到启动门槛才置方向
        if (ad_diff > START_AD_DIFF_THRESH && speed_now >= START_SPEED_THRESH) {
            new_dir = 1;
        } else if (ad_diff < -START_AD_DIFF_THRESH && speed_now >= START_SPEED_THRESH) {
            new_dir = 2;
        } else {
            new_dir = 0;
        }
    } else {
        // 当前运动：达到停止门槛才置0；否则根据较高的启动门槛判断是否切换方向
        if (fabsf((float)ad_diff) <= (float)STOP_AD_DIFF_THRESH && speed_now <= STOP_SPEED_THRESH) {
            new_dir = 0;
        } else {
            if (ad_diff > START_AD_DIFF_THRESH) {
                new_dir = 1;
            } else if (ad_diff < -START_AD_DIFF_THRESH) {
                new_dir = 2;
            } else {
                // 保持原方向，降低敏感度
            }
        }
    }

    s_last_direction = new_dir;
    GSS_device.run_direction = new_dir;
}

void Modbus_Process_Position_Data(uint32_t ad_now)
{
    // 0) 获取当前RTC时间（用于速度/里程保存/静止判断）
    bsp_rtc_def now_rtc = {0};
    BSP_RTC_Get(&now_rtc);
    uint32_t now_sod = RTC_SecOfDay(&now_rtc);

    // 首帧初始化
    if (first_run)
    {
        g_last_position     = ad_now;
        g_current_position  = ad_now;
        position_diff       = 0;

        // 速度计算基准
        s_last_speed_rtc    = now_rtc;
        s_last_speed_ad     = ad_now;
        g_real_speed        = 0.0f;

        // 动作/保存基准
        s_last_move_sod         = now_sod;
        s_last_mileage_save_sod = now_sod;

        // 将已有的累计里程（可能由loadini从Flash恢复）映射到浮点累计
        s_total_distance_m_f    = (float)g_total_meters / 1000.0f;
        s_last_saved_meters_mm  = g_total_meters;

        // 实时位置刷新
        GSS_device.position_data_ad = g_current_position;
        float pos_calc = (float)GSS_device.position_slope *
                         ((int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point)
                         + (float)GSS_device.position_offset;
        GSS_device.position_data_real = pos_calc < 0 ? 0.0f : pos_calc;

        GSS_device.real_speed = 0.0f;
        GSS_device.run_direction = 0;

        first_run = 0;
        return;
    }

    // 1) 原始计数更新与差值（有符号）
    g_last_position    = g_current_position;
    g_current_position = ad_now;
    position_diff      = (int32_t)g_current_position - (int32_t)g_last_position;

    // 2) 实时位置（米，支持正负，若<0则钳制为0）
    GSS_device.position_data_ad = g_current_position;
    float pos_calc = (float)GSS_device.position_slope *
                     ((int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point)
                     + (float)GSS_device.position_offset;
    GSS_device.position_data_real = pos_calc < 0 ? 0.0f : pos_calc;

    // 3) 有效AD差值（用于里程累计/方向判定）
    int32_t eff_ad_diff = position_diff;

    // 抖动过滤
    if (labs(eff_ad_diff) < MIN_VALID_AD_STEP) {
        eff_ad_diff = 0;
    }
    // 异常限幅
    if ((uint32_t)labs(position_diff) > MAX_VALID_AD_STEP) {
        // 异常跳变直接忽略此次增量
        // LOGT("AD异常跳变: diff=%ld, 已忽略\n", position_diff);
        eff_ad_diff = 0;
    }

    // 4) 速度（m/s）——使用RTC按物理时间计算，每满1秒更新，并做平滑与滞回处理
    if ((s_last_speed_rtc.hour | s_last_speed_rtc.minute | s_last_speed_rtc.second) == 0)
    {
        // 极端情况下的二次保护（一般不会走到）
        s_last_speed_rtc = now_rtc;
        s_last_speed_ad  = ad_now;
        g_real_speed     = 0.0f;
    }
    else
    {
        uint32_t last_sod = RTC_SecOfDay(&s_last_speed_rtc);
        uint32_t dt_s = (now_sod >= last_sod) ? (now_sod - last_sod) : (now_sod + 86400u - last_sod);

        if (dt_s >= 1u) 
				{
            float measured_speed = 0.0f;

            int32_t ad_diff_for_speed = (int32_t)ad_now - (int32_t)s_last_speed_ad;
            if (labs(ad_diff_for_speed) >= MIN_VALID_AD_STEP &&
                (uint32_t)labs(ad_diff_for_speed) <= MAX_VALID_AD_STEP) {
                float delta_m_for_speed = fabsf((float)ad_diff_for_speed * (float)GSS_device.position_slope);
                measured_speed = delta_m_for_speed / (float)dt_s;
            } else {
                measured_speed = 0.0f;
            }

            // IIR 平滑，避免从0开始过于敏感
            g_real_speed = SPEED_FILTER_ALPHA * g_real_speed + (1.0f - SPEED_FILTER_ALPHA) * measured_speed;

            // 更新速度计算基准（每秒刷新一次，无论是否有位移）
            s_last_speed_rtc = now_rtc;
            s_last_speed_ad  = ad_now;
        }
        // 未满1秒：保留当前平滑速度
    }
    GSS_device.real_speed = g_real_speed;

    // 5) 总里程（mm）：仅对有效位移累加绝对值
    float delta_m_abs = fabsf((float)eff_ad_diff * (float)GSS_device.position_slope);
    s_total_distance_m_f += delta_m_abs;                               // 累计米（浮点）
    g_total_meters        = (uint32_t)(s_total_distance_m_f * 1000.0f); // 转mm整数
    GSS_device.Total_meters = g_total_meters / 1000;                    // 显示整米

    // 6) 运行方向（带滞回）
    Update_Direction_With_Hysteresis(position_diff, g_real_speed);

    // 7) 若存在有效位移，刷新“最近动作”时间
    if (eff_ad_diff != 0 || g_real_speed >= START_SPEED_THRESH) {
        s_last_move_sod = now_sod;
    }

    // 8) 里程保存策略：跨越10米即保存；小时兜底在后台tick中处理
    APP_USER_Mileage_Flash_Save_Handle();

    // 9) 静止与小时兜底保存后台处理（本函数末尾也可调用一次）
    Position_Idle_Tick();

    // LOG（如需）
    // LOGT("采样: AD_now=%lu, AD_diff=%ld, eff_diff=%ld, delta_m=%.5f, 累计m=%.3f, 总里程mm=%lu, 速度=%.3f, 方向=%u\n",
    //      g_current_position, position_diff, eff_ad_diff, delta_m_abs, s_total_distance_m_f, g_total_meters, g_real_speed, s_last_direction);
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

    // 无论是否收到数据，都执行一次后台tick（用于超时静止/小时保存）
    Position_Idle_Tick();
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
    s_last_saved_meters_mm = 0; // 清除已保存参考
    FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, 0);

    // 复位后也更新保存时间，避免瞬间触发小时兜底
    s_last_mileage_save_sod = Now_SecOfDay();
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
    // 由于此时 AD == zero_point，直接令 offset = 0 更直观
    GSS_device.position_offset = 0.0f;

    // 3) 立即更新设备实时位置为0
    GSS_device.position_data_ad   = g_current_position;
    GSS_device.position_data_real = 0.0f;

    // 4) 里程清零（按现有逻辑）
    APP_USER_Reset_Total_Meters();
}

// 将当前计算结果同步到GSS_device（如需在其它流程调用）
void APP_USER_UpdateDevicePositionAndSpeed(void)
{
    GSS_device.position_data_real = APP_USER_Get_Relative_Position();
    GSS_device.position_data_ad   = g_current_position;
    GSS_device.real_speed         = APP_USER_Get_Real_Speed();
}

// 独立的方向更新（若需要周期调用：已带滞回）
void APP_USER_UpdateRunDirection(void)
{
    Update_Direction_With_Hysteresis(position_diff, g_real_speed);
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
        FLASH_WriteU32_WithCheck(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
        GSS_device.position_offset      = y2;   // 与(AD - zero_point)搭配

        // 立即按新标定刷新实时位置
        GSS_device.position_data_ad   = g_current_position;
        int32_t ad_diff = (int32_t)GSS_device.position_data_ad - (int32_t)GSS_device.position_zero_point;
        GSS_device.position_data_real = (float)GSS_device.position_slope * (float)ad_diff + (float)GSS_device.position_offset;

        if (GSS_device.position_data_real < 0.0f) {
            GSS_device.position_data_real = 0.0f;
        }
    }
}

//==================== 简单的闪存写封装 ====================

void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value)
{
    (void)EEPROM_FLASH_WriteU32(addr, value);
//    LOGT("保存到Flash: addr=0x%04X, value=%lu\n", addr, value);
}
