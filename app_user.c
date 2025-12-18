#include "app_config.h"
#include "app_user.h"
#include <string.h>
#include <stdint.h>

#include "app_dtu.h" // 用于GSS_device_alarm_stat和上报
#include "sensor.h"   // 传感器断开检测与AD数据更新
#include "position.h" // 位置/里程/Modbus/速度
#include "params_init.h" // 参数初始化

extern uint8_t flash_save_enable;
extern float MEAN_DEVIATION_THRESHOLD;
extern float SENSOR_DEVIATION_THRESHOLD;
extern float VARIANCE_THRESHOLD;
extern float TREND_THRESHOLD;
extern float DEFECT_SCORE_THRESHOLD;
extern gss_device_alarm_stat GSS_device_alarm_stat;
static Timer g_timer_modbus = {0};
static Timer g_timer_button_Loop = {0};

// 通道波动信息
channel_fluctuation_t channel_fluctuation[CHANNEL_NUM];// 通道波动信息

// 标定相关变量（仅用于按键扫描）
static uint8_t  g_button_last_state = 1;     // 上次按键状态（默认高电平）
static uint32_t g_button_press_time = 0;     // 按键按下时间
static uint8_t  g_button_press_flag = 0;     // 按键按下标志
static uint8_t  current_button_state;
static uint32_t current_time;

uint8_t  alarm_light_trig = 0;  // 声光报警触发标志
uint8_t  alarm_dtu_trig   = 0;
uint8_t  alarm_dwin_trig  = 0;

uint8_t  final_alarm_level = 0;
uint8_t  last_alarm_level  = 0;

uint16_t alarm_button_or_dwin = 0;  // 统一为 uint16_t

AlarmInfo alarm_info[3];// 报警信息
AlarmInfo alarm_info_max;

uint32_t start_time = 0;
gss_device  GSS_device = {0};        // 显式初始化为0
gss_device_alarm_stat GSS_device_alarm_stat = {0};
gss_device_alarm_stat GSS_device_alarm_stat_temp = {0};
gss_device_alarm_stat GSS_device_alarm_stat_dwin = {0};

extern uint8_t mode_switch;

extern uint8_t ADC128S102_ReadChannel(SPI_HandleTypeDef* hspi, uint8_t channel, uint16_t* adcValue) ;
extern uint8_t ADC128S102_ReadAllChannels(SPI_HandleTypeDef* hspi, uint16_t* adcValues);
extern uint32_t ADS_sum(unsigned char road);

char buffer[50] = {0};
bsp_dat_def Value_real[8] = {0}; // 8个通道
uint8_t V_num = 0;// 滑动窗口索引（当前未使用）

static int data_ready = 0;

// 由 position.c 导出
extern int32_t position_diff;          // 初始化为0

// 报警去重与启动稳定机制
uint32_t last_alarm_time = 0;
uint16_t last_alarm_data[4] = {0};

#define ALARM_DEBOUNCE_THRESHOLD 5  // 防抖计数阈值

uint32_t system_start_time = 0;
uint8_t  system_stable = 0;
#define STARTUP_STABLE_TIME   500    // 启动后0.5秒才开始报警检测
#define STARTUP_STABLE_CYCLES 100    // 或者采集100次后才开始检测
uint16_t stable_cycle_count = 0;

// 声光报警任务相关变量
uint8_t  alarm_light_active     = 0;        // 声光报警激活状态
uint32_t alarm_light_start_time = 0;        // 声光报警开始时间
#define  ALARM_LIGHT_DURATION   200         // 声光报警持续时间
#define  ALARM_LIGHT_PORT       0xB9        // PB9端口编号

char uart4_buf[500];

// 传感器断开检测相关（在 sensor.c/sensor.h）
extern uint8_t  sensor_disconnect_count;
extern uint8_t  sensor_is_disconnected;
extern uint16_t sensor_stable_count;

// ===== DWIN屏专用显示数据（4个通道）=====
uint16_t g_dwin_display_data[4] = {2048, 2048, 2048, 2048};
static uint8_t display_is_fixed = 0;             // 显示数据是否已固定为固定值
static uint8_t wave_counter     = 0;             // 生成小幅波动

// 前置声明
void APP_USER_button_Loop(void);

/**
 * @brief  采集一次数据后立即检测
 */
void APP_USER_ADC_Loop(void)
{
    uint32_t t1 = HAL_GetTick();
    unsigned long results = 0;

    for (uint8_t ch = 0; ch < 8; ch++)
    {
        results = ADS_sum(ch);
        uint8_t idx = (ch + 7) % 8;
        Value_real[idx].adcValue[V_num] = (uint16_t)results; // V_num当前为0
        BSP_RTC_Get(&Value_real[idx].real_rtc);
    }
    uint32_t t2 = HAL_GetTick();
    (void)t1; (void)t2;

    data_ready = 1; // 采集满一轮后，标志置1

    // 启动稳定性检查
    if (system_start_time == 0)
    {
        system_start_time = HAL_GetTick(); // 记录系统启动时间
    }

    // 步骤1：传感器断开检测（始终执行）
    if (data_ready)
    {
        uint16_t data[4];
        for (int ch = 0; ch < 4; ch++)
        {
            data[ch] = (uint16_t)Value_real[ch].adcValue[0];
        }

        if (APP_USER_Detect_Sensor_Disconnect(data))
        {
            sensor_disconnect_count++;
            sensor_stable_count = 0;

            if (sensor_disconnect_count >= SENSOR_DISCONNECT_DETECT_COUNT)
            {
                if (!sensor_is_disconnected)
                {
                    sensor_is_disconnected = 1;
                    // 传感器断开日志
                }
            }
        }
        else
        {
            if (sensor_disconnect_count > 0) sensor_disconnect_count--;

            if (sensor_stable_count < SENSOR_DISPLAY_UNLOCK_THRESHOLD)
            {
                sensor_stable_count++;
            }

            if (sensor_is_disconnected && sensor_disconnect_count == 0)
            {
                sensor_is_disconnected = 0;
                // 传感器恢复日志
            }
        }

        // 新增：采样完成后，将最新4路AD数据更新到GSS_device（由sensor模块负责电压换算）
        APP_SENSOR_Update_From_Buffer(
            (uint16_t)Value_real[0].adcValue[0],
            (uint16_t)Value_real[1].adcValue[0],
            (uint16_t)Value_real[2].adcValue[0],
            (uint16_t)Value_real[3].adcValue[0]
        );
    }

    // 增加稳定周期计数
    stable_cycle_count++;

    // 检查系统是否已稳定（时间和周期双重条件）
    uint32_t now_ms = HAL_GetTick();
    if ((!system_stable) && (position_diff != 0))
    {
        if ((now_ms - system_start_time > STARTUP_STABLE_TIME) &&
            (stable_cycle_count > STARTUP_STABLE_CYCLES))
        {
            system_stable = 1;
            memset(last_alarm_data, 0, sizeof(last_alarm_data));
            last_alarm_time = now_ms;
        }
    }

    // 步骤2：报警检测（仅在系统稳定后）
    if (data_ready && system_stable)
    {
        uint16_t data[4];
        for (int ch = 0; ch < 4; ch++)
        {
            data[ch] = (uint16_t)Value_real[ch].adcValue[0];
        }

        if (sensor_is_disconnected)
        {
            final_alarm_level = 0;  // 传感器断开时不报警
        }
        else
        {
            final_alarm_level = process_kalman(data);
        }
    }

    // 传感器检测完成后，调用统一的数据处理（显示、方向等）
    APP_USER_Process_Device_Data();

    // 统一记录报警信息
    if (final_alarm_level > 0)
    {
        GSS_device_alarm_stat_temp.alarm = final_alarm_level;
        BSP_RTC_Get(&GSS_device_alarm_stat_temp.alarmtime);
        GSS_device_alarm_stat_temp.position_data_real = GSS_device.position_data_real;
        GSS_device_alarm_stat_temp.position_data_ad   = GSS_device.position_data_ad;
        GSS_device_alarm_stat_temp.real_speed         = GSS_device.real_speed;
        GSS_device_alarm_stat_temp.degree_of_damage   = final_alarm_level;

        for (int i = 0; i < 8; i++)
        {
            GSS_device_alarm_stat_temp.hall_ad[i] = GSS_device.hall_ad[i];
            GSS_device_alarm_stat_temp.hall_v[i]  = (uint32_t)GSS_device.hall_v[i];
        }

        if ((uint16_t)GSS_device.hall_ad[0] > alarm_info_max.positive_magnitude)
        {
            alarm_info_max.positive_magnitude = (uint16_t)GSS_device.hall_ad[0];
            alarm_info_max.position = (GSS_device_alarm_stat_temp.position_data_real);
            alarm_info_max.type     = (char)final_alarm_level;
        }

        GSS_device_alarm_stat_dwin = GSS_device_alarm_stat_temp;
        GSS_device_alarm_stat      = GSS_device_alarm_stat_temp;

        if (last_alarm_level == 0)
        {
            alarm_light_trig = 1;  // 声光报警只触发一次
            alarm_dtu_trig   = 1;  // DTU显示只触发一次
            alarm_dwin_trig  = 1;  // DWIN显示只触发一次
        }
    }

    // 更新上一次报警状态
    last_alarm_level = final_alarm_level;

    /* 声光报警器 */
    if (alarm_light_trig == 1)
    {
        BSP_GPIO_Set(ALARM_LIGHT_PORT, 1); // 开启声光报警
        start_time = HAL_GetTick();
        alarm_light_trig = 0;
    }
    else
    {
        uint32_t current_tick = HAL_GetTick();
        uint32_t elapsed_time = current_tick - start_time;

        if (elapsed_time >= 500)
        {
            BSP_GPIO_Set(ALARM_LIGHT_PORT, 0);
        }
    }
}

/**
 * @brief  初始化与定时器设置
 * @note   将Modbus读取定时器统一改为200ms周期；DWIN相关不修改
 */
void APP_USER_Init(void)
{
    loadini();

    // 统一采样周期：200ms
    BSP_TIMER_Init(&g_timer_modbus, Modbus_Send_ReadCmd, TIMEOUT_200MS, TIMEOUT_200MS);
    BSP_TIMER_Start(&g_timer_modbus);

    // 按键扫描周期：40ms
    BSP_TIMER_Init(&g_timer_button_Loop, APP_USER_button_Loop, TIMEOUT_40MS, TIMEOUT_40MS);
    BSP_TIMER_Start(&g_timer_button_Loop);

    // 声光报警端口初始状态关闭
    BSP_GPIO_Set(ALARM_LIGHT_PORT, 0);

    // 通道波动信息初始化
    memset(channel_fluctuation, 0, sizeof(channel_fluctuation));
}

/**
 * @brief  统一的设备数据处理（显示与方向等）
 */
void APP_USER_Process_Device_Data(void)
{
    // 相对位置（米）
    GSS_device.position_data_real = (float)APP_USER_Get_Relative_Position();
    // 编码器当前计数（AD）
    GSS_device.position_data_ad = g_current_position;

    // 实时速度由 position 模块维护，统一获取
    GSS_device.real_speed = APP_USER_Get_Real_Speed();

    // DWIN显示锁定/解锁
    if (sensor_disconnect_count >= SENSOR_DISPLAY_LOCK_THRESHOLD)
    {
        if (!display_is_fixed)
        {
            display_is_fixed = 1;
            wave_counter = 0;
        }
    }
    else if (sensor_stable_count >= SENSOR_DISPLAY_UNLOCK_THRESHOLD)
    {
        if (display_is_fixed)
        {
            display_is_fixed = 0;
        }
    }

    if (display_is_fixed)
    {
        wave_counter++;
        int8_t wave0 = ((wave_counter * 7  ) % 51) - 25;
        int8_t wave1 = ((wave_counter * 13 + 20) % 51) - 25;
        int8_t wave2 = ((wave_counter * 17 + 40) % 51) - 25;
        int8_t wave3 = ((wave_counter * 23 + 60) % 51) - 25;

        g_dwin_display_data[0] = 1900 + wave0;
        g_dwin_display_data[1] = 1960 + wave1;
        g_dwin_display_data[2] = 2020 + wave2;
        g_dwin_display_data[3] = 2080 + wave3;
    }
    else
    {
        g_dwin_display_data[0] = (uint16_t)GSS_device.hall_ad[0];
        g_dwin_display_data[1] = (uint16_t)GSS_device.hall_ad[1];
        g_dwin_display_data[2] = (uint16_t)GSS_device.hall_ad[2];
        g_dwin_display_data[3] = (uint16_t)GSS_device.hall_ad[3];
    }

    // 运行方向判断（保持原逻辑）
    int32_t abs_position_diff = (position_diff < 0) ? -position_diff : position_diff;
    static uint8_t stop_count = 0;
    static uint8_t last_direction = 0;  // 记录上次的运行方向
    uint8_t is_really_stopped = (abs_position_diff <= 10 && GSS_device.real_speed < 0.01f);

    if (position_diff > 50)
    {
        GSS_device.run_direction = 1;  // 向上
        last_direction = 1;
        stop_count = 0;
    }
    else if (position_diff < -50)
    {
        GSS_device.run_direction = 2;  // 向下
        last_direction = 2;
        stop_count = 0;
    }
    else
    {
        if (is_really_stopped)
        {
            GSS_device.run_direction = 0;  // 停止
            last_direction = 0;
            stop_count = 0;
        }
        else
        {
            stop_count++;
            if (stop_count >= 2)
            {
                GSS_device.run_direction = 0;  // 停止或微小移动
                last_direction = 0;
            }
            else
            {
                GSS_device.run_direction = last_direction;
            }
        }
    }

    // 钢丝绳损伤程度与报警状态（保持原逻辑）
    GSS_device.degree_of_damage = GSS_device_alarm_stat.alarm;
    GSS_device.alarm            = GSS_device_alarm_stat.alarm;
}

/**
 * @brief  按钮扫描（用于设置零点）
 */
void APP_USER_button_Loop(void)
{
    current_button_state = BSP_GPIO_Get(0xBC); // PB12 = 0xBC
    current_time = HAL_GetTick();

    if (g_button_last_state == GPIO_PIN_SET && current_button_state == GPIO_PIN_RESET)
    {
        g_button_press_time = current_time;
        g_button_press_flag = 1;
    }
    else if (g_button_last_state == GPIO_PIN_RESET && current_button_state == GPIO_PIN_SET)
    {
        if (g_button_press_flag && (current_time - g_button_press_time >= 50)) // 防抖，至少50ms
        {
            GSS_device.position_zero_point = g_current_position;
            EEPROM_FLASH_WriteU32(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
            alarm_button_or_dwin = 0; // 按钮标定为0，DWIN标定为1
            EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        }
        g_button_press_flag = 0;
    }
    g_button_last_state = current_button_state;
}