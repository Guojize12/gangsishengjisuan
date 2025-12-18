#include "app_user.h"   // 包含 ENCODER_COUNTS_PER_REV、类型定义与外部依赖声明
#include "position.h"

// 位置数据相关变量
static uint32_t g_last_position = 0;        // 上次位置值
uint32_t g_current_position = 0;     // 当前位置值
static uint32_t g_position_change_count = 0; // 位置变化次数
static float g_postion = 0.0f;          // 上次高度值（旧逻辑，已不用于累计里程）
uint32_t g_total_meters = 0;         // 总里程累计值（毫米）

/******** 绝对值编码器的统一周期与系数（仅用于计算，不改DWIN） ********/
static float g_sample_period_s = 0.2f;        // 采样与计算统一周期（秒）：200ms
static float g_position_scale_m_per_rev = 0.3f;  // 每圈对应的米数系数（默认0.3米/圈）
static float g_encoder_max_step_m = 3.0f;        // 单步位移最大允许值（米），用于跳变保护
static uint32_t g_mileage_save_step_m = 10;      // 里程写入步进阈值（米）
static uint32_t g_mileage_save_interval_ms = 60*60*1000; // 里程写入定时间隔（毫秒）：1小时
static uint32_t s_last_save_meters = 0;         // 上次写入的整米值
static uint32_t s_last_save_tick = 0;           // 上次写入时间戳（毫秒）

float g_max_position = 0;        // 最大位置值（米，显示用途）
float g_min_position = 0;        // 最小位置值（米，显示用途）

int32_t position_diff = 0;          // 初始化为0，避免启动时显示异常值

extern uint8_t alarm_button_or_dwin;
extern gss_device  GSS_device;

extern uint32_t HAL_GetTick(void);
extern uint16_t bsp_crc16(const uint8_t* data, uint16_t len);

extern void BSP_GPIO_Set(uint16_t pin, uint8_t val);
extern void BSP_UART_Transmit(uint8_t uart, uint8_t *buf, uint16_t len);
extern usr_err_t BSP_UART_Rec_Read(uint8_t uart);

extern uart_buf_def g_uart_buf[];
extern Timer* BSP_TIMER_Init(Timer* t, void (*cb)(void), uint32_t reload, uint32_t start);
extern void BSP_TIMER_Start(Timer* t);

extern void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value);

// 发送Modbus RTU读取命令
// 用于读取AMT50S8M10U10-RC0.5绝对值编码器位置
void Modbus_Send_ReadCmd(void)
{
    uint8_t cmd[8];
    uint8_t dev_addr = 0x01;        // AMT50编码器从机地址
    uint16_t reg_addr = 0x0000;     // 位置寄存器地址（32位位置）
    uint16_t reg_num = 0x0002;      // 读取2个寄存器（32位）

    // 构建Modbus RTU帧
    cmd[0] = dev_addr;                  // 从机地址
    cmd[1] = 0x03;                      // 功能码：读保持寄存器
    cmd[2] = (reg_addr >> 8) & 0xFF;    // 寄存器地址高字节
    cmd[3] = reg_addr & 0xFF;           // 寄存器地址低字节
    cmd[4] = (reg_num >> 8) & 0xFF;     // 寄存器数量高字节
    cmd[5] = (reg_num & 0xFF);            // 寄存器数量低字节

    // 计算CRC16校验
    uint16_t crc = bsp_crc16(cmd, 6);
    cmd[6] = (crc >> 8) & 0xFF;                 // CRC低字节
    cmd[7] = crc & 0xFF;                       // CRC高字节

    // 发送数据
    BSP_GPIO_Set(0XB5, 1);              // RS485发送模式
    BSP_UART_Transmit(APP_MODBUS_UART, cmd, 8);
    BSP_GPIO_Set(0XB5, 0);              // RS485接收模式

    // 【重要】已移除在此处累计总里程的旧逻辑，改为在接收并解析位置后处理
    // 原逻辑依赖于旧的g_postion变量，容易与接收解析时序不一致。
}

extern uint32_t stop_time;
// 位置数据处理函数（新：在此进行里程累计与跳变保护）
void Modbus_Process_Position_Data(uint32_t position)
{
    // 更新位置数据
    g_last_position = g_current_position;
    g_current_position = position;

    // 始终计算位置差值，无论位置是否变化
    position_diff = (int32_t)g_current_position - (int32_t)g_last_position;

    // 检查位置变化（仅用于计数）
    if (g_current_position != g_last_position)
    {
        g_position_change_count++;
    }

    // 计算本次位移（米）：|Δ计数| / 每圈计数 × 圈长系数
    int32_t abs_diff = (position_diff < 0) ? -position_diff : position_diff;
    float delta_m = ((float)abs_diff / (float)ENCODER_COUNTS_PER_REV) * g_position_scale_m_per_rev;

    // 单步跳变保护：超过设定阈值则不累计
    if (delta_m > g_encoder_max_step_m)
    {
        // 日志位置预留：单步跳变过大，本次位移未计入里程
        return;
    }

    // 累计总里程（内部以毫米为单位）
    g_total_meters += (uint32_t)(delta_m * 1000.0f);

    // 记录最大/最小位置（米）用于显示
    float current_position_m = ((float)g_current_position / (float)ENCODER_COUNTS_PER_REV) * g_position_scale_m_per_rev;
    if (current_position_m > g_max_position) g_max_position = current_position_m;
    if (g_min_position == 0 || current_position_m < g_min_position) g_min_position = current_position_m;

    // 同步对外显示的总里程（米）
    GSS_device.Total_meters = g_total_meters / 1000;

    // 触发里程保存策略（步进+定时）
    APP_USER_Mileage_Flash_Save_Handle();
}

// Modbus接收处理函数
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
                                                (rx_data[5] << 8) | rx_data[6];
                            Modbus_Process_Position_Data(position);
                        }
                        else
                        {
                            // 日志位置预留：CRC错误
                        }
                    }
                }
                else if (rx_data[1] == 0x83) // 错误响应
                {
                    // 日志位置预留：Modbus异常响应
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

// 获取总里程（毫米）- 内部存储仍以毫米为单位
uint32_t APP_USER_Get_Total_Meters(void)
{
    return g_total_meters;
}

// 重置总里程
void APP_USER_Reset_Total_Meters(void)
{
    g_total_meters = 0;
    g_postion = 0.0f;

    // 保存到FLASH（关键参数写入走包裹接口）
    FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, 0);

    // 日志位置预留：总里程清零
}

// 获取相对于零点的位置（智能标定模式判断）
float  APP_USER_Get_Relative_Position(void)
{
    // 情况1：按钮标定模式（alarm_button_or_dwin = 0）
    if (alarm_button_or_dwin == 0)
    {
        // 相对位置（米）= (当前计数-零点计数)/每圈计数 × 圈长系数
        return ((int32_t)g_current_position - (int32_t)GSS_device.position_zero_point)
               * (g_position_scale_m_per_rev / (float)ENCODER_COUNTS_PER_REV);
     }
    // 情况2：DWIN标定模式（alarm_button_or_dwin = 1）
    else if (alarm_button_or_dwin == 1)
    {
        // 保持原有线性映射，不改动DWIN流程
        return (float)(GSS_device.position_slope * (float)GSS_device.position_data_ad + GSS_device.position_offset) ;
    }
    // 情况3：未知模式或异常情况
    else
    {
        alarm_button_or_dwin = 0;
        return ((int32_t)g_current_position - (int32_t)GSS_device.position_zero_point)
               * (g_position_scale_m_per_rev / (float)ENCODER_COUNTS_PER_REV);
    }
}

// 手动设置标定零点（用于外部设置）
void APP_USER_Set_Zero_Point(uint32_t zero_point)
{
    GSS_device.position_zero_point = zero_point;

    // 保存到FLASH（关键参数写入走包裹接口）
    FLASH_WriteU32_WithCheck(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
}

/******** 关键参数写入包裹与里程保存策略 ********/
/**
 * @brief  关键参数写入包裹函数（预留冗余校验位置）
 * @param  addr: Flash地址
 * @param  value: 要写入的数值
 * @note   未来可在此增加镜像地址写入、CRC校验、版本号等冗余机制
 */
void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value)
{
    (void)EEPROM_FLASH_WriteU32(addr, value);
}

/**
 * @brief  里程保存策略处理（步进+定时）
 * @note   条件1：总里程每增加≥10米写一次Flash；
 *         条件2：若总里程变化不大，则至少每1小时写一次。
 */
void APP_USER_Mileage_Flash_Save_Handle(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t meters_now = g_total_meters / 1000; // 转为整米比较

    // 步进触发
    if ((meters_now - s_last_save_meters) >= g_mileage_save_step_m)
    {
        FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);
        s_last_save_meters = meters_now;
        s_last_save_tick = now;
        return;
    }

    // 定时触发（兜底）
    if ((now - s_last_save_tick) >= g_mileage_save_interval_ms)
    {
        FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);
        s_last_save_meters = meters_now;
        s_last_save_tick = now;
        return;
    }
}