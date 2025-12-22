#include "app_config.h"
#include "app_user.h"
#include <string.h>
#include <math.h>

#include "app_dtu.h" // 用于GSS_device_alarm_stat和上报
#include "sensor.h"   // 传感器断开检测与AD数据更新
#include "position.h" // 位置/里程/Modbus/速度
#include "params_init.h" // 参数初始化
#include "app_kalman.h"  // 新增：用于总预热超时计时器重置

extern uint8_t flash_save_enable;
extern float MEAN_DEVIATION_THRESHOLD;
extern float SENSOR_DEVIATION_THRESHOLD;
extern float VARIANCE_THRESHOLD;
extern float TREND_THRESHOLD;
extern float DEFECT_SCORE_THRESHOLD;
extern gss_device_alarm_stat GSS_device_alarm_stat;
static Timer g_timer_modbus = {0};
static Timer g_timer_button_Loop = {0};
extern uint8_t warmup_init_flag;

// 通道波动信息
channel_fluctuation_t channel_fluctuation[CHANNEL_NUM];// 通道波动信息

// 旧的按键扫描变量未使用，移除以优化结构

uint8_t  alarm_light_trig = 0;  // 声光报警触发标志
uint8_t  alarm_dtu_trig   = 0;
uint8_t  alarm_dwin_trig  = 0;

uint8_t  final_alarm_level = 0;
uint8_t  last_alarm_level  = 0;


/*取消alarm_button_or_dwin判断所有都为1*/
uint16_t alarm_button_or_dwin = 1;  // 统一为 uint16_t

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

// ========= 按键提示继电器响声（非阻塞，参考声光报警器） =========
static BeepState g_beep_state    = BEEP_IDLE;
static uint8_t   g_beep_remain   = 0;           // 剩余鸣叫次数
static uint32_t  g_beep_last_tick= 0;

// 调整为注释中所述：单声100ms，间隔150ms
#define BEEP_ON_TIME    100    // ms，单次鸣叫时长
#define BEEP_OFF_TIME   150    // ms，间隔时长

// 启动N次提示鸣叫（非阻塞）
static inline void Relay_Beep_N_Times(uint8_t n)
{
    if (n == 0) return;
    g_beep_remain    = n;
    g_beep_state     = BEEP_ON;
    g_beep_last_tick = HAL_GetTick();
    BSP_GPIO_Set(ALARM_LIGHT_PORT, 1); // 首声，继电器吸合
}

// 每次主循环调用，驱动鸣叫状态机
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
    uint32_t t2 = HAL_GetTick();
    (void)t1; (void)t2;
    data_ready = 1;
}

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
                    // log
                }
            }
        } else {
            if (sensor_disconnect_count > 0) sensor_disconnect_count--;
            if (sensor_stable_count < SENSOR_DISPLAY_UNLOCK_THRESHOLD) sensor_stable_count++;
            if (sensor_is_disconnected && sensor_disconnect_count == 0) {
                sensor_is_disconnected = 0;
                // log
            }
        }
        // 将最新4路AD数据更新到GSS_device
        APP_SENSOR_Update_From_Buffer(
            (uint16_t)Value_real[0].adcValue[0],
            (uint16_t)Value_real[1].adcValue[0],
            (uint16_t)Value_real[2].adcValue[0],
            (uint16_t)Value_real[3].adcValue[0]
        );
    }
    stable_cycle_count++;
}

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

static void APP_USER_Check_Alarm_Level(void)
{
    if (data_ready && system_stable) {
        uint16_t data[4];
        for (int ch = 0; ch < 4; ch++) data[ch] = (uint16_t)Value_real[ch].adcValue[0];
        if (sensor_is_disconnected) {
            final_alarm_level = 0;
        } else {
            final_alarm_level = process_kalman(data);
        }
    }
}

static void APP_USER_Record_Alarm_Info(void)
{
    if (final_alarm_level > 0) {
        GSS_device_alarm_stat_temp.alarm = final_alarm_level;
        BSP_RTC_Get(&GSS_device_alarm_stat_temp.alarmtime);
        GSS_device_alarm_stat_temp.position_data_real = GSS_device.position_data_real;
        GSS_device_alarm_stat_temp.position_data_ad = GSS_device.position_data_ad;
        GSS_device_alarm_stat_temp.real_speed = GSS_device.real_speed;
        GSS_device_alarm_stat_temp.degree_of_damage = final_alarm_level;
        for (int i = 0; i < 8; i++) {
            GSS_device_alarm_stat_temp.hall_ad[i] = GSS_device.hall_ad[i];
            GSS_device_alarm_stat_temp.hall_v[i]  = (uint32_t)GSS_device.hall_v[i];
        }
        if ((uint16_t)GSS_device.hall_ad[0] > alarm_info_max.positive_magnitude) {
            alarm_info_max.positive_magnitude = (uint16_t)GSS_device.hall_ad[0];
            alarm_info_max.position = GSS_device_alarm_stat_temp.position_data_real;
            alarm_info_max.type = (char)final_alarm_level;
        }
        GSS_device_alarm_stat_dwin = GSS_device_alarm_stat_temp;
        GSS_device_alarm_stat = GSS_device_alarm_stat_temp;
        if (last_alarm_level == 0) {
            alarm_light_trig = 1;
            alarm_dtu_trig = 1;
            alarm_dwin_trig = 1;
        }
    }
    last_alarm_level = final_alarm_level;
}

static void APP_USER_Handle_Alarm_Output(void)
{
    /* 声光报警器 */
    if (alarm_light_trig == 1) {
        BSP_GPIO_Set(ALARM_LIGHT_PORT, 1); // 开启声光报警
        start_time = HAL_GetTick();
        alarm_light_trig = 0;
    } else {
        uint32_t current_tick = HAL_GetTick();
        uint32_t elapsed_time = current_tick - start_time;
        if (elapsed_time >= 500)
            BSP_GPIO_Set(ALARM_LIGHT_PORT, 0);
    }
}


void APP_USER_ADC_Loop(void)
{
    APP_USER_Sample_ADC();
    APP_USER_Check_Sensor_Disconnect();
    APP_USER_Check_System_Stability();
    APP_USER_Check_Alarm_Level();
    APP_USER_Process_Device_Data();
    APP_USER_Record_Alarm_Info();
    APP_USER_Handle_Alarm_Output();

    // 非阻塞提示鸣叫状态机（放在报警输出之后，避免抢占）
    Relay_Beep_Task();
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


// 2. 显示锁定、解锁状态判断与切换
static void APP_USER_UpdateDisplayLockState(void)
{
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
}

// 3. DWIN显示数据维护
static void APP_USER_UpdateDwinDisplayData(void)
{
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
}

// 5. 钢丝绳报警与损伤赋值
static void APP_USER_UpdateAlarmState(void)
{
    GSS_device.degree_of_damage = GSS_device_alarm_stat.alarm;
    GSS_device.alarm            = GSS_device_alarm_stat.alarm;
}

// 主调度函数
void APP_USER_Process_Device_Data(void)
{
    APP_USER_UpdateDevicePositionAndSpeed();
    APP_USER_UpdateDisplayLockState();
    APP_USER_UpdateDwinDisplayData();
    APP_USER_UpdateRunDirection();
    APP_USER_UpdateAlarmState();
}

/**
 * @brief  按钮扫描（用于设置零点）
 */

ButtonEvent Button_DetectEvent(void)
{
    static uint8_t last_state = GPIO_PIN_SET;
    static uint32_t press_time = 0;
    static uint8_t press_flag = 0;

    uint8_t current_state = BSP_GPIO_Get(0xBC);
    uint32_t current_time = HAL_GetTick();
    ButtonEvent event = BUTTON_EVENT_NONE;

    if (last_state == GPIO_PIN_SET && current_state == GPIO_PIN_RESET)
    {
        press_time = current_time;
        press_flag = 1;
        event = BUTTON_EVENT_PRESSED;
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

// ========= 新增：按键两步标定状态机 + 1分钟自动切换检测 + 10分钟超时回退 =========
#define CALIBRATION_SECOND_PRESS_TIMEOUT_MS   (600000UL) // 10分钟
#define PREHEAT_AFTER_FIRST_PRESS_TIMEOUT_MS  (60000UL)  // 1分钟

typedef enum {
    CAL_WAIT_FIRST = 0,   // 等待第一次按键
    CAL_WAIT_SECOND       // 已完成第一次，等待第二次
} CalibButtonState;

void APP_USER_button_Loop(void)
{
    static CalibButtonState cal_state = CAL_WAIT_FIRST;
    static uint32_t first_press_tick = 0; // 记录第一次按键释放的时间

    // 若处于等待第二次状态，优先检查是否达到1分钟自动进入检测模式
    if (cal_state == CAL_WAIT_SECOND && first_press_tick != 0)
    {
        uint32_t now = HAL_GetTick();

        // 1分钟后自动切换到检测模式（不依赖样本数，不保存参数）
        if (now - first_press_tick >= PREHEAT_AFTER_FIRST_PRESS_TIMEOUT_MS)
        {
            mode_switch = 1;
            EEPROM_FLASH_WriteU16(FLASH_MODE_SWITCH, mode_switch);
            // 提示：两声（可选）
            Relay_Beep_N_Times(2);

            // 完成一次切换后，回到等待第一次
            first_press_tick = 0;
            cal_state = CAL_WAIT_FIRST;
        }
        else if (now - first_press_tick >= CALIBRATION_SECOND_PRESS_TIMEOUT_MS)
        {
            // 10分钟超时：回到等待第一次状态（原有逻辑保留）
            cal_state = CAL_WAIT_FIRST;
            first_press_tick = 0;
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

        // 按键提示：响一声（100ms），间隔150ms）
        Relay_Beep_N_Times(1);

        // 进入等待第二次状态，并记录时间戳（用于1分钟自动切换）
        first_press_tick = HAL_GetTick();
        cal_state = CAL_WAIT_SECOND;
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
        float curr_pos_m = APP_USER_Get_Relative_Position();
        g_total_meters = (uint32_t)(curr_pos_m * 1000.0f);
        s_total_distance_m_f = curr_pos_m;
        FLASH_WriteU32_WithCheck(FLASH_TOTAL_METERS, g_total_meters);

        // 提示：响两声
        Relay_Beep_N_Times(2);

        // 一轮完成，回到等待第一次
        first_press_tick = 0;
        cal_state = CAL_WAIT_FIRST;
    }
}
