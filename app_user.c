#include "app_config.h"
#include "app_user.h"
#include "bsp_timer.h"
#include "bsp_gpio.h"
#include "string.h"

#include "app_dtu.h"     // 用于GSS_device_alarm_stat和上报
#include "sensor.h"      // 传感器断开检测与AD数据更新
#include "position.h"    // 位置/里程/Modbus/速度
#include "params_init.h" // 参数初始化
#include "app_kalman.h"  // 总预热超时计时器重置

/* 外部参数与状态 */
extern uint8_t  flash_save_enable;
extern float    MEAN_DEVIATION_THRESHOLD;
extern float    SENSOR_DEVIATION_THRESHOLD;
extern float    VARIANCE_THRESHOLD;
extern float    TREND_THRESHOLD;
extern float    DEFECT_SCORE_THRESHOLD;
extern gss_device_alarm_stat GSS_device_alarm_stat;

/* 定时器 */
static Timer g_timer_modbus      = {0};
static Timer g_timer_button_Loop = {0};

/* 预热状态 */
extern uint8_t warmup_init_flag;

/* 通道波动信息（用于显示/统计） */
channel_fluctuation_t channel_fluctuation[CHANNEL_NUM];

/* 报警触发标志 */
uint8_t  alarm_light_trig = 0;
uint8_t  alarm_dtu_trig   = 0;
uint8_t  alarm_dwin_trig  = 0;

/* 报警等级与去重 */
uint8_t  final_alarm_level = 0;
uint8_t  last_alarm_level  = 0;

/* 统一按键来源控制（旧逻辑兼容） */
uint16_t alarm_button_or_dwin = 1;

AlarmInfo alarm_info[3];
AlarmInfo alarm_info_max;

/* 设备与报警状态 */
uint32_t start_time = 0;
gss_device                GSS_device                 = {0};
gss_device_alarm_stat     GSS_device_alarm_stat      = {0};
gss_device_alarm_stat     GSS_device_alarm_stat_temp = {0};
gss_device_alarm_stat     GSS_device_alarm_stat_dwin = {0};

extern uint8_t mode_switch;

/* 采集相关 */
extern uint8_t  ADC128S102_ReadChannel(SPI_HandleTypeDef* hspi, uint8_t channel, uint16_t* adcValue);
extern uint8_t  ADC128S102_ReadAllChannels(SPI_HandleTypeDef* hspi, uint16_t* adcValues);
extern uint32_t ADS_sum(unsigned char road);

char        buffer[50] = {0};
bsp_dat_def Value_real[8] = {0}; // 8个通道
uint8_t     V_num = 0;           // 滑动窗口索引（当前未使用）

static volatile int data_ready = 0;

/* 来自 position.c */
extern int32_t position_diff;

/* 启动稳定机制 */
uint32_t last_alarm_time    = 0;
uint16_t last_alarm_data[4] = {0};

#define ALARM_DEBOUNCE_THRESHOLD 5

uint32_t system_start_time = 0;
uint8_t  system_stable     = 0;
#define STARTUP_STABLE_TIME   500    // 启动后0.5秒才开始报警检测
#define STARTUP_STABLE_CYCLES 100    // 或者采集100次后才开始检测
uint16_t stable_cycle_count = 0;

/* 声光报警状态机/继电器 */
uint8_t  alarm_light_active     = 0;
uint32_t alarm_light_start_time = 0;

/* 按键提示蜂鸣器状态机 */
static BeepState g_beep_state     = BEEP_IDLE;
static uint8_t   g_beep_remain    = 0;           // 剩余鸣叫次数
static uint32_t  g_beep_last_tick = 0;

#define BEEP_ON_TIME   100    // ms，单次鸣叫时长
#define BEEP_OFF_TIME  150    // ms，间隔时长

/* 传感器断开检测计数 */
extern uint8_t  sensor_disconnect_count;
extern uint8_t  sensor_is_disconnected;
extern uint16_t sensor_stable_count;

/* DWIN显示数据与锁定 */
uint16_t g_dwin_display_data[4] = {2048, 2048, 2048, 2048};
static uint8_t display_is_fixed = 0;
static uint8_t wave_counter     = 0;

/* 前置声明 */
void APP_USER_button_Loop(void);

/* ========= 采样 ========= */
/* 采集一次数据后置 data_ready 标志，供后续步骤使用 */
static void APP_USER_Sample_ADC(void)
{
    uint32_t t1 = HAL_GetTick();
    unsigned long results = 0;
    for (uint8_t ch = 0; ch < 8; ch++) {
        results = ADS_sum(ch);
        uint8_t idx = (ch + 7) % 8;
        Value_real[idx].adcValue[V_num] = (uint16_t)results;
        BSP_RTC_Get(&Value_real[idx].real_rtc);
    }
    (void)t1;
    data_ready = 1;
}

/* ========= 断线检测与数据同步 ========= */
static void APP_USER_Check_Sensor_Disconnect(void)
{
    if (system_start_time == 0)
        system_start_time = HAL_GetTick();

    if (data_ready) {
        uint16_t data[4];
        for (int ch = 0; ch < 4; ch++) data[ch] = (uint16_t)Value_real[ch].adcValue[0];

        if (APP_USER_Detect_Sensor_Disconnect(data)) {
            sensor_disconnect_count++;
            sensor_stable_count = 0;
            if (sensor_disconnect_count >= SENSOR_DISCONNECT_DETECT_COUNT) {
                if (!sensor_is_disconnected) {
                    sensor_is_disconnected = 1;
                    // TODO: add log if needed
                }
            }
        } else {
            if (sensor_disconnect_count > 0) sensor_disconnect_count--;
            if (sensor_stable_count < SENSOR_DISPLAY_UNLOCK_THRESHOLD) sensor_stable_count++;
            if (sensor_is_disconnected && sensor_disconnect_count == 0) {
                sensor_is_disconnected = 0;
                // TODO: add log if needed
            }
        }

        // 更新最新4路AD数据到GSS_device
        APP_SENSOR_Update_From_Buffer(
            (uint16_t)Value_real[0].adcValue[0],
            (uint16_t)Value_real[1].adcValue[0],
            (uint16_t)Value_real[2].adcValue[0],
            (uint16_t)Value_real[3].adcValue[0]
        );
    }
    stable_cycle_count++;
}

/* ========= 系统稳定判定（开机抑制误报） ========= */
static void APP_USER_Check_System_Stability(void)
{
    uint32_t now_ms = HAL_GetTick();
    if ((!system_stable) && (position_diff != 0)) {
        if ((now_ms - system_start_time > STARTUP_STABLE_TIME) &&
            (stable_cycle_count > STARTUP_STABLE_CYCLES)) {
            system_stable = 1;
            memset(last_alarm_data, 0, sizeof(last_alarm_data));
            last_alarm_time = now_ms;
        }
    }
}

/* ========= 报警等级计算（卡尔曼 + 判据） ========= */
static void APP_USER_Check_Alarm_Level(void)
{
    if (!data_ready) return;

    uint16_t data[4];
    for (int ch = 0; ch < 4; ch++) {
        data[ch] = (uint16_t)Value_real[ch].adcValue[0];
    }

    // 始终运行卡尔曼主流程，以便预热超时自动切换能生效
    uint8_t computed = process_kalman(data);

    // 仅在系统稳定且传感器未断开时，输出报警等级；其他情况下不报警
    if (system_stable && !sensor_is_disconnected) {
        final_alarm_level = computed;
    } else {
        final_alarm_level = 0;
    }
}

/* ========= 报警信息记录与触发 ========= */
static void APP_USER_Record_Alarm_Info(void)
{
    if (final_alarm_level > 0) {
        GSS_device_alarm_stat_temp.alarm = final_alarm_level;
        BSP_RTC_Get(&GSS_device_alarm_stat_temp.alarmtime);
        GSS_device_alarm_stat_temp.position_data_real = GSS_device.position_data_real;
        GSS_device_alarm_stat_temp.position_data_ad   = GSS_device.position_data_ad;
        GSS_device_alarm_stat_temp.real_speed         = GSS_device.real_speed;
        GSS_device_alarm_stat_temp.degree_of_damage   = final_alarm_level;

        for (int i = 0; i < 8; i++) {
            GSS_device_alarm_stat_temp.hall_ad[i] = GSS_device.hall_ad[i];
            GSS_device_alarm_stat_temp.hall_v[i]  = (uint32_t)GSS_device.hall_v[i];
        }

        if ((uint16_t)GSS_device.hall_ad[0] > alarm_info_max.positive_magnitude) {
            alarm_info_max.positive_magnitude = (uint16_t)GSS_device.hall_ad[0];
            alarm_info_max.position           = GSS_device_alarm_stat_temp.position_data_real;
            alarm_info_max.type               = (char)final_alarm_level;
        }

        GSS_device_alarm_stat_dwin = GSS_device_alarm_stat_temp;
        GSS_device_alarm_stat      = GSS_device_alarm_stat_temp;

        if (last_alarm_level == 0) {
            alarm_light_trig = 1;
            alarm_dtu_trig   = 1;
            alarm_dwin_trig  = 1;
        }
    }
    last_alarm_level = final_alarm_level;
}

/* ========= 声光报警继电器输出（非阻塞时序） ========= */
static void APP_USER_Handle_Alarm_Output(void)
{
    if (alarm_light_trig == 1) {
        BSP_GPIO_Set(ALARM_LIGHT_PORT, 1); // 开启声光报警
        start_time       = HAL_GetTick();
        alarm_light_trig = 0;
    } else {
        uint32_t elapsed = HAL_GetTick() - start_time;
        if (elapsed >= ALARM_LIGHT_DURATION)
            BSP_GPIO_Set(ALARM_LIGHT_PORT, 0);
    }
}

/* ========= 提示蜂鸣：启动N次鸣叫（非阻塞状态机） ========= */
static inline void Relay_Beep_N_Times(uint8_t n)
{
    if (n == 0) return;
    g_beep_remain    = n;
    g_beep_state     = BEEP_ON;
    g_beep_last_tick = HAL_GetTick();
    BSP_GPIO_Set(ALARM_LIGHT_PORT, 1); // 首声，继电器吸合
}

/* 每次主循环调用，驱动鸣叫状态机 */
static inline void Relay_Beep_Task(void)
{
    if (g_beep_state == BEEP_IDLE) return;

    uint32_t now = HAL_GetTick();
    if (g_beep_state == BEEP_ON)
    {
        if (now - g_beep_last_tick >= BEEP_ON_TIME)
        {
            BSP_GPIO_Set(ALARM_LIGHT_PORT, 0); // 拉低，进入间隔
            g_beep_state     = BEEP_OFF;
            g_beep_last_tick = now;
        }
    }
    else if (g_beep_state == BEEP_OFF)
    {
        if (now - g_beep_last_tick >= BEEP_OFF_TIME)
        {
            if (g_beep_remain > 0) g_beep_remain--;
            if (g_beep_remain == 0)
            {
                g_beep_state = BEEP_IDLE;
                BSP_GPIO_Set(ALARM_LIGHT_PORT, 0); // 保证最终拉低
            }
            else
            {
                BSP_GPIO_Set(ALARM_LIGHT_PORT, 1); // 下一声吸合
                g_beep_state     = BEEP_ON;
                g_beep_last_tick = now;
            }
        }
    }
}

/* ========= 主业务循环 ========= */
void APP_USER_ADC_Loop(void)
{
    APP_USER_Sample_ADC();
    APP_USER_Check_Sensor_Disconnect();
    APP_USER_Check_System_Stability();
    APP_USER_Check_Alarm_Level();
    APP_USER_Process_Device_Data();
    APP_USER_Record_Alarm_Info();
    APP_USER_Handle_Alarm_Output();

    // 非阻塞提示鸣叫状态机
    Relay_Beep_Task();

    // 重置本次采样标志，避免重复处理
    data_ready = 0;
}

/* ========= 业务初始化与定时器设置 ========= */
void APP_USER_Init(void)
{
    loadini();

    // Modbus采样周期：200ms
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

/* ========= 显示锁定/解锁判断与切换 ========= */
static void APP_USER_UpdateDisplayLockState(void)
{
    if (sensor_disconnect_count >= SENSOR_DISPLAY_LOCK_THRESHOLD)
    {
        if (!display_is_fixed)
        {
            display_is_fixed = 1;
            wave_counter     = 0;
        }
    }
    else if (sensor_stable_count >= SENSOR_DISPLAY_UNLOCK_THRESHOLD)
    {
        if (display_is_fixed)
        {
            display_is_fixed = 0;
        }
    }
}

/* ========= DWIN显示数据维护 ========= */
static void APP_USER_UpdateDwinDisplayData(void)
{
    if (display_is_fixed)
    {
        wave_counter++;
        int8_t wave0 = ((wave_counter *  7     ) % 51) - 25;
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
}

/* ========= 钢丝绳报警与损伤赋值 ========= */
static void APP_USER_UpdateAlarmState(void)
{
    GSS_device.degree_of_damage = GSS_device_alarm_stat.alarm;
    GSS_device.alarm            = GSS_device_alarm_stat.alarm;
}

/* ========= 主调度：设备数据处理（位置/显示/方向/报警） ========= */
void APP_USER_Process_Device_Data(void)
{
    APP_USER_UpdateDevicePositionAndSpeed();
    APP_USER_UpdateDisplayLockState();
    APP_USER_UpdateDwinDisplayData();
    APP_USER_UpdateRunDirection();
    APP_USER_UpdateAlarmState();
}

/* ========= 按钮检测（防抖） ========= */
ButtonEvent Button_DetectEvent(void)
{
    static uint8_t  last_state  = GPIO_PIN_SET;
    static uint32_t press_time  = 0;
    static uint8_t  press_flag  = 0;

    uint8_t   current_state = BSP_GPIO_Get(0xBC);
    uint32_t  current_time  = HAL_GetTick();
    ButtonEvent event       = BUTTON_EVENT_NONE;

    if (last_state == GPIO_PIN_SET && current_state == GPIO_PIN_RESET)
    {
        press_time = current_time;
        press_flag = 1;
        event      = BUTTON_EVENT_PRESSED;
    }
    else if (last_state == GPIO_PIN_RESET && current_state == GPIO_PIN_SET)
    {
        if (press_flag && (current_time - press_time >= 50)) // 防抖
        {
            event = BUTTON_EVENT_RELEASED;
        }
        press_flag = 0;
    }
    last_state = current_state;
    return event;
}

/* ========= 按键两步标定状态机 + 1分钟自动切检测 + 10分钟超时回退 ========= */
#define CALIBRATION_SECOND_PRESS_TIMEOUT_MS   (600000UL) // 10分钟
#define PREHEAT_AFTER_FIRST_PRESS_TIMEOUT_MS  (60000UL)  // 1分钟

typedef enum 
{
    CAL_WAIT_FIRST = 0,   // 等待第一次按键
    CAL_WAIT_SECOND       // 已完成第一次，等待第二次
} CalibButtonState;

void APP_USER_button_Loop(void)
{
    static CalibButtonState cal_state          = CAL_WAIT_FIRST;
    static uint32_t         first_press_tick   = 0;     // 第一次按键释放时间戳
    static uint8_t          one_min_switch_done= 0;     // 1分钟自动切换是否已执行

    // 若处于等待第二次状态，优先检查是否达到超时
    if (cal_state == CAL_WAIT_SECOND && first_press_tick != 0)
    {
        uint32_t now     = HAL_GetTick();
        uint32_t elapsed = now - first_press_tick;

        // 10分钟超时：回到初始
        if (elapsed >= CALIBRATION_SECOND_PRESS_TIMEOUT_MS)
        {
            cal_state           = CAL_WAIT_FIRST;
            first_press_tick    = 0;
            one_min_switch_done = 0;
        }
        // 1分钟：仅切到检测模式，不回到初始；只执行一次
        else if (!one_min_switch_done && elapsed >= PREHEAT_AFTER_FIRST_PRESS_TIMEOUT_MS)
        {
            mode_switch = 1;
            EEPROM_FLASH_WriteU16(FLASH_MODE_SWITCH, mode_switch);
            Relay_Beep_N_Times(2);
            one_min_switch_done = 1;
            // 不修改 cal_state / first_press_tick，继续等待第二次按键
        }
    }

    ButtonEvent event = Button_DetectEvent();

    // 保留：任何“按下”事件都重置“总预热超时计时器”
    if (event == BUTTON_EVENT_PRESSED) {
        warmup_reset_timeout_counter();
    }

    if (event != BUTTON_EVENT_RELEASED)
        return;

    // 仅在“释放”事件上处理一次
    if (cal_state == CAL_WAIT_FIRST)
    {
        // 第一次：设置零点为当前AD，实时位置归零；保存下限信号；模式置0
        APP_USER_Set_Zero_Point(g_current_position);
        GSS_device.position_signal_lower = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_BUT_DATA, GSS_device.position_signal_lower); // 保存位置信号下限
        mode_switch = 0;
        EEPROM_FLASH_WriteU16(FLASH_MODE_SWITCH, mode_switch);
        warmup_init_flag = 0; // 让预热重新初始化

        // 重置总预热超时计时器，表示有用户交互
        warmup_reset_timeout_counter();

        // 按键提示：响一声
        Relay_Beep_N_Times(1);

        // 进入等待第二次状态，并记录时间戳（用于1分钟与10分钟判断）
        first_press_tick    = HAL_GetTick();
        one_min_switch_done = 0;
        cal_state           = CAL_WAIT_SECOND;
    }
    else // CAL_WAIT_SECOND
    {
        // 第二次：保存上限信号，计算斜率与偏置，模式置1
        GSS_device.position_signal_upper = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_TOP_DATA, GSS_device.position_signal_upper); // 保存位置信号上限
        InitSmartCalibration();
        mode_switch = 1;
        EEPROM_FLASH_WriteU16(FLASH_MODE_SWITCH, mode_switch);

        // 累计里程设为当前位置实际米数
        float curr_pos_m      = APP_USER_Get_Relative_Position();
        g_total_meters        = (uint32_t)(curr_pos_m * 1000.0f);
        s_total_distance_m_f  = curr_pos_m;
        FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);

        // 提示：响两声
        Relay_Beep_N_Times(2);

        // 一轮完成，回到等待第一次
        first_press_tick    = 0;
        one_min_switch_done = 0;
        cal_state           = CAL_WAIT_FIRST;
    }
}

/* ========= 预留：外设接收处理（按需实现） ========= */
void APP_USER_Rx_Handle(void)
{
    /* 占位：如需UART或其它外设接收额外处理，可在此实现 */
}
