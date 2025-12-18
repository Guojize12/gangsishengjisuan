#include "app_config.h"
#include "app_user.h"
#include <string.h>   // 为 memset 提供声明

#include "app_dtu.h" // 用于GSS_device_alarm_stat和上报
#include "sensor.h"   // 拆分：传感器断开检测
#include "position.h" // 拆分：位置/里程/Modbus
#include "params_init.h" // 拆分：参数初始化

extern uint8_t flash_save_enable;
extern float MEAN_DEVIATION_THRESHOLD;
extern float SENSOR_DEVIATION_THRESHOLD;
extern float VARIANCE_THRESHOLD;
extern float TREND_THRESHOLD;
extern float DEFECT_SCORE_THRESHOLD;
extern gss_device_alarm_stat GSS_device_alarm_stat;
static Timer g_timer_modbus = {0};
static Timer g_timer_button_Loop = {0};

// 位置数据相关变量（已拆分至 position.c）
// static uint32_t g_last_position = 0;        // 上次位置值
// uint32_t g_current_position = 0;     // 当前位置值
// static uint32_t g_position_change_count = 0; // 位置变化次数
// static float g_postion = 0.0f;          // 上次高度值（旧逻辑，已不用于累计里程）
// static uint32_t g_total_meters = 0;         // 总里程累计值（毫米）

/******** 新增：绝对值编码器的统一周期与系数（仅用于计算，不改DWIN） ********/
// 已拆分至 position.c
// static float g_sample_period_s = 0.2f;        // 采样与计算统一周期（秒）：200ms
// static float g_position_scale_m_per_rev = 0.3f;  // 每圈对应的米数系数（默认0.3米/圈）
// static float g_encoder_max_step_m = 3.0f;        // 单步位移最大允许值（米），用于跳变保护
// static uint32_t g_mileage_save_step_m = 10;      // 里程写入步进阈值（米）
// static uint32_t g_mileage_save_interval_ms = 60*60*1000; // 里程写入定时间隔（毫秒）：1小时
// static uint32_t s_last_save_meters = 0;         // 上次写入的整米值
// static uint32_t s_last_save_tick = 0;           // 上次写入时间戳（毫秒）

// float g_max_position = 0;        // 最大位置值（米，显示用途）
// float g_min_position = 0;        // 最小位置值（米，显示用途）

uint8_t anomaly_count = 0;
uint16_t anomaly_positions[2] = {0, 0};
uint16_t ch0_positive_magnitude  = 0;

// 标定相关变量
static uint8_t  g_button_last_state = 1;     // 上次按键状态（默认高电平）
static uint32_t g_button_press_time = 0;    // 按键按下时间
static uint8_t  g_button_press_flag = 0;     // 按键按下标志
static uint8_t  current_button_state;
static uint32_t current_time;

// 报警变量已拆分到 alarm.c
// uint8_t final_alarm_level = 0;
// uint8_t last_alarm_level = 0;  // 上一次的报警状态，用于检测报警状态变化
uint8_t  alarm_light_trig = 0;  // 声光报警触发标志，供其他模块访问
uint8_t alarm_dtu_trig = 0;
uint8_t alarm_dwin_trig = 0;

uint8_t final_alarm_level = 0;
uint8_t last_alarm_level = 0;

uint8_t  alarm_button_or_dwin = 0;

AlarmInfo alarm_info[3];// 报警信息
AlarmInfo alarm_info_max;
channel_fluctuation_t channel_fluctuation[CHANNEL_NUM];// 通道波动信息

uint32_t start_time = 0;
gss_device  GSS_device = {0};        // 显式初始化为0，确保所有字段初始值正确
gss_device_alarm_stat GSS_device_alarm_stat = {0};      // 显式初始化为0
gss_device_alarm_stat GSS_device_alarm_stat_temp = {0};  // 显式初始化为0
gss_device_alarm_stat GSS_device_alarm_stat_dwin = {0}; // 显式初始化为0

extern uint8_t mode_switch ;

// FLASH存储地址定义已移到 app_user.h 中

// static uint32_t    nucSysIDNo = 0;          /*设备ID-8034     2025-05-19*/ // 已移至 params_init.c

extern uint8_t ADC128S102_ReadChannel(SPI_HandleTypeDef* hspi, uint8_t channel, uint16_t* adcValue) ;
extern uint8_t ADC128S102_ReadAllChannels(SPI_HandleTypeDef* hspi, uint16_t* adcValues);
extern uint32_t ADS_sum(unsigned char road);

char buffer[50] = {0};
bsp_dat_def Value_real[8] = {0}; // 8个通道，每个通道32组历史数据
uint8_t V_num = 0;// 滑动窗口索引

static int data_ready = 0;

// 由 position.c 导出
extern int32_t position_diff;          // 初始化为0，避免启动时显示异常值
// ========= 补充：报警去重机制的全局变量定义（之前缺失） =========
uint32_t last_alarm_time = 0;
uint16_t last_alarm_data[4] = {0};

#define ALARM_DEBOUNCE_THRESHOLD 5  // 防抖计数阈值

// 启动稳定机制
uint32_t system_start_time = 0;
uint8_t system_stable = 0;
#define STARTUP_STABLE_TIME 500    // 启动后0.5秒才开始报警检测
#define STARTUP_STABLE_CYCLES 100  // 或者采集100次后才开始检测
uint16_t stable_cycle_count = 0;

// 声光报警任务相关变量
uint8_t alarm_light_active = 0;        // 声光报警激活状态
uint32_t alarm_light_start_time = 0;   // 声光报警开始时间
#define ALARM_LIGHT_DURATION 200             // 声光报警持续时间5秒
#define ALARM_LIGHT_PORT 0xB9                 // PB9端口编号

char uart4_buf[500];

// 传感器断开检测相关宏/变量已移至 sensor.h/sensor.c

// ===== DWIN屏专用显示数据（4个通道）=====
// 传感器正常时：与GSS_device.hall_ad[]相同
// 传感器断开时：显示固定值（1900/2000/2100/2200）附近小幅波动，便于区分四个通道
uint16_t g_dwin_display_data[4] = {2048, 2048, 2048, 2048};
static uint8_t display_is_fixed = 0;             // 显示数据是否已固定为固定值
// sensor_stable_count 已移至 sensor.c
static uint8_t wave_counter = 0;                // 波动计数器（用于生成小幅波动）

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
        Value_real[idx].adcValue[V_num] = results;//V_num一直为0
        BSP_RTC_Get(&Value_real[idx].real_rtc);
    }
    uint32_t t2 = HAL_GetTick();

    data_ready = 1; // 采集满一轮后，标志置1

    // 启动稳定性检查
    if (system_start_time == 0)
    {
        system_start_time = HAL_GetTick(); // 记录系统启动时间
    }
    // ===== 【步骤1】传感器断开检测（始终执行，不受system_stable限制）=====
    if (data_ready)
    {
        uint16_t data[4];  // 定点数类型
        for (int ch = 0; ch < 4; ch++)
        {
            data[ch] = (uint16_t)Value_real[ch].adcValue[0];
        }

        // 传感器断开检测与状态更新
        if (APP_USER_Detect_Sensor_Disconnect(data))
        {
            // 检测到异常
            sensor_disconnect_count++;
            sensor_stable_count = 0;  // 检测到异常，稳定计数器清零

            // 连续检测到多次异常，确认传感器断开
            if (sensor_disconnect_count >= SENSOR_DISCONNECT_DETECT_COUNT)
            {
                if (!sensor_is_disconnected)
                {
                    sensor_is_disconnected = 1;
                    // 日志位置预留：传感器断开
                }
            }
        }
        else
        {
            // 传感器数据正常
            if (sensor_disconnect_count > 0)
            {
                sensor_disconnect_count--;
            }

            // 累计连续正常次数
            if (sensor_stable_count < SENSOR_DISPLAY_UNLOCK_THRESHOLD)
            {
                sensor_stable_count++;
            }

            if (sensor_is_disconnected && sensor_disconnect_count == 0)
            {
                sensor_is_disconnected = 0;
                // 日志位置预留：传感器恢复
            }
        }
    }

    // 增加稳定周期计数
    stable_cycle_count++;
    // 检查系统是否已稳定（时间和周期双重条件）
    uint32_t current_time = HAL_GetTick();
    if ((!system_stable) && (position_diff != 0))
    {
        if ((current_time - system_start_time > STARTUP_STABLE_TIME) &&
                (stable_cycle_count > STARTUP_STABLE_CYCLES))
        {
            system_stable = 1;
            // 系统稳定后，清空历史报警数据，避免启动数据干扰
            memset(last_alarm_data, 0, sizeof(last_alarm_data));
            last_alarm_time = current_time;
        }
    }

    // ===== 【步骤2】报警检测（只在系统稳定后才进行）=====
    if (data_ready && system_stable)
    {
        uint16_t data[4];  // 定点数类型
        for (int ch = 0; ch < 4; ch++)
        {
            data[ch] = (uint16_t)Value_real[ch].adcValue[0];
        }

        // 传感器断开时不进行报警检测，直接设置报警级别为0
        if (sensor_is_disconnected)
        {
            final_alarm_level = 0;  // 传感器断开时不报警
        }
        else
        {
            final_alarm_level = process_kalman(data);
        }
    }

    // ===== 在传感器检测完成后，调用统一的数据处理函数 =====
    APP_USER_Process_Device_Data();

    // 统一记录报警信息（略：保留原逻辑）
    if (final_alarm_level > 0)
    {
        GSS_device_alarm_stat_temp.alarm = final_alarm_level;  // 记录报警状态
        BSP_RTC_Get(&GSS_device_alarm_stat_temp.alarmtime);    // 获取当前时间
        GSS_device_alarm_stat_temp.position_data_real = GSS_device.position_data_real;    // 获取当前位置
        GSS_device_alarm_stat_temp.position_data_ad = GSS_device.position_data_ad;      // 获取当前位置ad
        GSS_device_alarm_stat_temp.real_speed = GSS_device.real_speed;
        GSS_device_alarm_stat_temp.degree_of_damage = final_alarm_level;    // 记录损伤程度
        for (int i = 0; i < 8; i++)// 记录当前通道信息
        {
            GSS_device_alarm_stat_temp.hall_ad[i] = GSS_device.hall_ad[i];
            GSS_device_alarm_stat_temp.hall_v[i] = (uint32_t)GSS_device.hall_v[i];
        }
        if (ch0_positive_magnitude > alarm_info_max.positive_magnitude)
        {
            alarm_info_max.positive_magnitude = ch0_positive_magnitude;
            alarm_info_max.position = (GSS_device_alarm_stat_temp.position_data_real);
            alarm_info_max.type = (char)final_alarm_level;
        }
        GSS_device_alarm_stat_dwin = GSS_device_alarm_stat_temp;
        GSS_device_alarm_stat =      GSS_device_alarm_stat_temp;

        if (last_alarm_level == 0)
        {
            alarm_light_trig = 1;  // 报警灯 - 只触发一次
            alarm_dtu_trig = 1;    // DTU显示 - 只触发一次
            alarm_dwin_trig = 1;   // DWIN显示 - 只触发一次
        }
    }

    // 更新上一次报警状态
    last_alarm_level = final_alarm_level;

    /*声光报警器*/
    if (alarm_light_trig == 1)
    {
        BSP_GPIO_Set(ALARM_LIGHT_PORT, 1); // 开启声光报警
        start_time = HAL_GetTick();
        alarm_light_trig = 0; // 重置触发标志，避免重复触发
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

void APP_USER_Init(void)
{
//    BSP_CONFIG_Init();
    loadini();

    // 将Modbus读取定时器周期改为200ms（要求统一采样周期）
    BSP_TIMER_Init(&g_timer_modbus, Modbus_Send_ReadCmd, TIMEOUT_200MS, TIMEOUT_200MS);
    BSP_TIMER_Start(&g_timer_modbus);

    // 保持按键扫描周期不变
    BSP_TIMER_Init(&g_timer_button_Loop, APP_USER_button_Loop, TIMEOUT_40MS, TIMEOUT_40MS);
    BSP_TIMER_Start(&g_timer_button_Loop);

    // 初始化声光报警端口PB9为输出模式
    BSP_GPIO_Set(ALARM_LIGHT_PORT, 0); // 初始状态为关闭

    // 初始化通道波动信息数组
    memset(channel_fluctuation, 0, sizeof(channel_fluctuation));
    // 日志位置预留：通道波动信息初始化
}

void APP_USER_Process_Device_Data(void)
{
    // 1. 霍尔传感器AD值处理（保持原始数据不变）
    GSS_device.hall_ad[0] = Value_real[0].adcValue[0];
    GSS_device.hall_ad[1] = Value_real[1].adcValue[0];
    GSS_device.hall_ad[2] = Value_real[2].adcValue[0];
    GSS_device.hall_ad[3] = Value_real[3].adcValue[0];

    // ===== DWIN显示锁定/解锁（保持原逻辑，不改DWIN）=====
    if (sensor_disconnect_count >= SENSOR_DISPLAY_LOCK_THRESHOLD)
    {
        if (!display_is_fixed)
        {
            display_is_fixed = 1;
            wave_counter = 0;  // 重置波动计数器
            // 日志位置预留：显示锁定
        }
    }
    else if (sensor_stable_count >= SENSOR_DISPLAY_UNLOCK_THRESHOLD)
    {
        if (display_is_fixed)
        {
            display_is_fixed = 0;
            // 日志位置预留：显示解锁
        }
    }

    if (display_is_fixed)
    {
        wave_counter++;
        int8_t wave0 = ((wave_counter * 7) % 51) - 25;
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
        g_dwin_display_data[0] = GSS_device.hall_ad[0];
        g_dwin_display_data[1] = GSS_device.hall_ad[1];
        g_dwin_display_data[2] = GSS_device.hall_ad[2];
        g_dwin_display_data[3] = GSS_device.hall_ad[3];
    }

    // 2. 霍尔传感器电压值计算
    GSS_device.hall_v[0] = ((float)GSS_device.hall_ad[0] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[1] = ((float)GSS_device.hall_ad[1] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[2] = ((float)GSS_device.hall_ad[2] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[3] = ((float)GSS_device.hall_ad[3] * 3300.0f) / 4095.0f;

    // 3. 位置数据处理（相对位置按新增系数计算；DWIN模式不改）
    GSS_device.position_data_real = (float)APP_USER_Get_Relative_Position() ;
    GSS_device.position_data_ad = g_current_position; // 编码器当前计数

    // 4. 实时速度计算（统一用200ms周期与系数）
    int32_t abs_position_diff = (position_diff < 0) ? -position_diff : position_diff;
    GSS_device.real_speed = ((float)abs_position_diff / (float)ENCODER_COUNTS_PER_REV)
                            * g_position_scale_m_per_rev
                            / g_sample_period_s; // 单位：m/s

    // 5. 运行方向判断（保持原逻辑）
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

    // 6. 报警状态数据（保持原逻辑）
    GSS_device.degree_of_damage = GSS_device_alarm_stat.alarm; // 钢丝绳损伤程度
    GSS_device.alarm = GSS_device_alarm_stat.alarm;

}
