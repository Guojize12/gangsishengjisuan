#include "app_config.h"
#include "app_user.h"
#include <string.h>

#include "app_dtu.h" // 用于GSS_device_alarm_stat和上报
extern uint8_t flash_save_enable;
extern float MEAN_DEVIATION_THRESHOLD;
extern float SENSOR_DEVIATION_THRESHOLD;
extern float VARIANCE_THRESHOLD;
extern float TREND_THRESHOLD;
extern float DEFECT_SCORE_THRESHOLD;
extern gss_device_alarm_stat GSS_device_alarm_stat;
static Timer g_timer_modbus = {0};
static Timer g_timer_button_Loop = {0};
// 位置数据相关变量
static uint32_t g_last_position = 0;        // 上次位置值
uint32_t g_current_position = 0;     // 当前位置值
static uint32_t g_position_change_count = 0; // 位置变化次数
static float g_postion = 0.0f;          // 上次高度值
static uint32_t g_total_meters = 0;         // 总里程累计值

float g_max_position = 0;        // 最大位置值
float g_min_position = 0;        // 最小位置值

uint8_t anomaly_count = 0;
uint16_t anomaly_positions[2] = {0, 0};
uint16_t ch0_positive_magnitude  = 0;

// 标定相关变量
static uint8_t  g_button_last_state = 1;     // 上次按键状态（默认高电平）
static uint32_t g_button_press_time = 0;    // 按键按下时间
static uint8_t  g_button_press_flag = 0;     // 按键按下标志
static uint8_t  current_button_state;
static uint32_t current_time;

uint8_t final_alarm_level = 0;
uint8_t last_alarm_level = 0;  // 上一次的报警状态，用于检测报警状态变化
uint8_t  alarm_light_trig = 0;  // 声光报警触发标志，供其他模块访问
uint8_t alarm_dtu_trig = 0;
uint8_t alarm_dwin_trig = 0;

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

static uint32_t    nucSysIDNo = 0;          /*设备ID-8034     2025-05-19*/

extern uint8_t ADC128S102_ReadChannel(SPI_HandleTypeDef* hspi, uint8_t channel, uint16_t* adcValue) ;
extern uint8_t ADC128S102_ReadAllChannels(SPI_HandleTypeDef* hspi, uint16_t* adcValues);
extern uint32_t ADS_sum(unsigned char road);

char buffer[50] = {0};
bsp_dat_def Value_real[8] = {0}; // 8个通道，每个通道32组历史数据
uint8_t V_num = 0;// 滑动窗口索引

static int data_ready = 0;

int32_t position_diff = 0;          // 初始化为0，避免启动时显示异常值
// 报警去重机制
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

// 传感器断开检测相关变量
#define SENSOR_DISCONNECT_HIGH_THRESHOLD  4000   // 高阈值（接近满量程4095）
#define SENSOR_DISCONNECT_LOW_THRESHOLD   100    // 低阈值（接近0）
#define SENSOR_DISCONNECT_DETECT_COUNT    50      // 连续检测到50次异常才判定为断开
#define SENSOR_FIX_VALUE                  2048   // 传感器断开时的固定输出值（12位ADC中间值）
#define SENSOR_DISPLAY_LOCK_THRESHOLD     2      // 显示锁定阈值：检测到1次异常就立即锁定
#define SENSOR_DISPLAY_UNLOCK_THRESHOLD   100    // 显示解锁阈值：需要连续100次正常才解锁

static uint8_t sensor_disconnect_count = 0;     // 传感器断开计数器
uint8_t sensor_is_disconnected = 0;             // 传感器断开状态标志（导出供其他模块使用）

// ===== DWIN屏专用显示数据（4个通道）=====
// 传感器正常时：与GSS_device.hall_ad[]相同
// 传感器断开时：显示固定值（1900/2000/2100/2200）附近小幅波动，便于区分四个通道
uint16_t g_dwin_display_data[4] = {2048, 2048, 2048, 2048};
static uint8_t display_is_fixed = 0;             // 显示数据是否已固定为固定值
static uint16_t sensor_stable_count = 0;        // 传感器连续正常计数器（用于解锁显示）
static uint8_t wave_counter = 0;                // 波动计数器（用于生成小幅波动）

/**
 * @brief  检测传感器是否断开
 * @param  data: 4个通道的ADC数据
 * @retval 1-传感器断开, 0-传感器正常
 * @note   判断依据：多个通道同时出现极端值（接近0或4095）
 */
static uint8_t APP_USER_Detect_Sensor_Disconnect(uint16_t data[4])
{
    uint8_t abnormal_count = 0;

    // 检查4个通道中有多少个通道数据异常
    for (uint8_t i = 0; i < 4; i++)
    {
        if (data[i] > SENSOR_DISCONNECT_HIGH_THRESHOLD || data[i] < SENSOR_DISCONNECT_LOW_THRESHOLD)
        {
            abnormal_count++;
        }
    }

    // 如果有3个或以上通道数据异常，判定为传感器可能断开
    if (abnormal_count >= 2)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief  修正传感器断开时的数据
 * @param  data: 4个通道的ADC数据
 * @retval None
 * @note   将所有通道数据修正为2048（12位ADC中间值）
 */
static void APP_USER_Fix_Disconnected_Sensor_Data(uint16_t data[4])
{
    for (uint8_t i = 0; i < 4; i++)
    {
        data[i] = SENSOR_FIX_VALUE;
    }
}

// 采集一次数据后立即检测
void APP_USER_ADC_Loop(void)
{
    uint32_t t1 = HAL_GetTick();
//    LOG("Function execution111111111111: %lu ms\r\n", t1);
    unsigned long results = 0;

    for (uint8_t ch = 0; ch < 8; ch++)
    {
        results = ADS_sum(ch);
        uint8_t idx = (ch + 7) % 8;
        Value_real[idx].adcValue[V_num] = results;//V_num一直为0
        BSP_RTC_Get(&Value_real[idx].real_rtc);
    }
    uint32_t t2 = HAL_GetTick();
#if 0
    snprintf(uart4_buf, sizeof(uart4_buf),
             "{plotter}%d,%d,%d,%d, %d  \n", Value_real[0].adcValue[V_num], Value_real[1].adcValue[V_num], Value_real[2].adcValue[V_num], Value_real[3].adcValue[V_num], t2 - t1);
//只有0-3可用  后四路未接
    BSP_UART_Transmit(BSP_UART4, (uint8_t*)uart4_buf, strlen(uart4_buf));
#endif
    // 采集索引循环
//    V_num++;
//    if (V_num >= DATA_BUF_LEN)
//    {
//        V_num = 0;
    data_ready = 1; // 采集满一轮后，标志置1
//    }
    // TODO: 可优化——滑动窗口长度、参数建议支持在线配置
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
                    LOG("!!! Sensor Disconnected Detected !!!\n");
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
                LOG("Sensor Reconnected\n");
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

#if 0
        LOG("data[0]: %d, data[1]: %d, data[2]: %d, data[3]: %d\n", data[0], data[1], data[2], data[3]);
#endif

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
//   LOG("final_alarm_level: %d\n", final_alarm_level);

    // ===== 在传感器检测完成后，调用统一的数据处理函数 =====
    // 这样GSS_device.hall_ad[]使用的是最新的sensor_is_disconnected状态
    // 注意：必须在if块外面调用，确保每次采集都更新GSS_device数据
    APP_USER_Process_Device_Data();

    // 统一记录报警信息
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
        // ===== 记录ch0的正幅度和alarm_info_max =====
        //  ch0_positive_magnitude = APP_USER_Get_Channel_Positive_Magnitude(0);
        if (ch0_positive_magnitude > alarm_info_max.positive_magnitude)
        {
            // 记录alarm_info_max中的最大正幅度信息
            alarm_info_max.positive_magnitude = ch0_positive_magnitude;
            alarm_info_max.position = (GSS_device_alarm_stat_temp.position_data_real);  // 转换为实际位置
            alarm_info_max.type = (char)final_alarm_level;
        }
        GSS_device_alarm_stat_dwin = GSS_device_alarm_stat_temp;
        GSS_device_alarm_stat =      GSS_device_alarm_stat_temp;

        // 只在报警状态从0变为非0时才触发（新报警产生）
        if (last_alarm_level == 0)
        {
            alarm_light_trig = 1;  // 报警灯 - 只触发一次
            alarm_dtu_trig = 1;    // DTU显示 - 只触发一次
            alarm_dwin_trig = 1;   // DWIN显示 - 只触发一次
        }
    }

    // 更新上一次报警状态
    last_alarm_level = final_alarm_level;
//  LOG("alarm_light_active = %d\n", alarm_light_active);
    /*声光报警器*/
    if (alarm_light_trig == 1)
    {
        BSP_GPIO_Set(ALARM_LIGHT_PORT, 1); // 开启声光报警
        start_time = HAL_GetTick();
        alarm_light_trig = 0; // 重置触发标志，避免重复触发
    }
    else
    {
        // 检查是否需要关闭声光报警（溢出安全）
        uint32_t current_tick = HAL_GetTick();
        uint32_t elapsed_time = current_tick - start_time;

        if (elapsed_time >= 500)
        {
            // 关闭声光报警
            BSP_GPIO_Set(ALARM_LIGHT_PORT, 0);
            // 注意：这里不需要重置alarm_light_trig，因为它已经是0了
        }
    }
}
extern uint16_t damage_degree;
extern uint16_t alarm_real_position[5];
extern uint16_t small_alarm_count;
extern uint16_t big_alarm_count;
extern uint16_t last_small_alarm_count;
extern uint16_t last_big_alarm_count;
void loadini(void)
{
    // 显式初始化关键数据，确保系统启动时数据正确
    // 初始化位置相关变量
    position_diff = 0;
    g_current_position = 0;

    // 初始化GSS_device的关键字段
    GSS_device.real_speed = 0.0f;         // 运行速度初始为0
    GSS_device.run_direction = 0;         // 运行方向初始为0（停止）
    GSS_device.position_data_real = 0.0f; // 当前位置初始为0
    GSS_device.position_data_ad = 0;      // 位置AD值初始为0
    GSS_device.alarm = 0;                 // 报警状态初始为0
    GSS_device.degree_of_damage = 0;      // 损伤程度初始为0

    // 初始化线性标定参数
    GSS_device.position_slope = 0.0f;             // 线性标定斜率初始为0
    GSS_device.position_offset = 0.0f;            // 线性标定截距初始为0
    nucSysIDNo  = EEPROM_FLASH_ReadU16(FLASHID);
    //     LOG("nucSysIDNo = %d\n", nucSysIDNo);
    GSS_device.position_range_upper    = EEPROM_FLASH_ReadU32(FLASH_POS_TOP_DATA);       /*位置量程上限*/
    GSS_device.position_range_lower = EEPROM_FLASH_ReadU32(FLASH_POS_BUT_DATA);       /*位置量程下限*/
    GSS_device.position_signal_upper = EEPROM_FLASH_ReadU32(FLASH_SIG_TOP_DATA);       /*位置信号上限*/
    GSS_device.position_signal_lower = EEPROM_FLASH_ReadU32(FLASH_SIG_BUT_DATA);       /*位置信号下限*/
    GSS_device.Threshold_set1 = EEPROM_FLASH_ReadU16(FLASH_THRESHOLD_1);       /*阈值1*/
    GSS_device.Threshold_set2 = EEPROM_FLASH_ReadU16(FLASH_THRESHOLD_2);       /*阈值2*/
    GSS_device.Threshold_set3 = EEPROM_FLASH_ReadU16(FLASH_THRESHOLD_3);       /*阈值3*/
    alarm_button_or_dwin = EEPROM_FLASH_ReadU16(FLASH_BUTTON_OR_DWIN);       /*按钮标定为0，DWIN标定为1*/
    GSS_device.position_zero_point = EEPROM_FLASH_ReadU32(FLASH_POSITION_ZERO_POINT);   // 读取位置标定数据
    for (int i = 0; i < 5; i++)
    {
        alarm_real_position[i] = EEPROM_FLASH_ReadU16(FLASH_ALARM_POSITION_COUNT1 + i * 2);/*热力图报警位置*/
    }
    damage_degree  = EEPROM_FLASH_ReadU16(FLASH_DAMAGE_DEGREE);/*损伤程度*/
    small_alarm_count = EEPROM_FLASH_ReadU16(FLASH_SMALL_ALARM_COUNT);/*轻微损伤次数*/
    last_small_alarm_count = small_alarm_count;/*轻微损伤次数*/
    big_alarm_count = EEPROM_FLASH_ReadU16(FLASH_BIG_ALARM_COUNT);/*严重损伤次数*/
    last_big_alarm_count = big_alarm_count;/*严重损伤次数*/
    g_total_meters = EEPROM_FLASH_ReadU32(FLASH_TOTAL_METERS);    /*总里程*/
//    mode_switch = EEPROM_FLASH_ReadU16(FLASH_MODE_SWITCH);    /*模式开关*/
    MEAN_DEVIATION_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_MEAN_DEVIATION_THRESHOLD) / 10;
    SENSOR_DEVIATION_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_SENSOR_DEVIATION_THRESHOLD) / 10;
    VARIANCE_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_VARIANCE_THRESHOLD) / 10;
    TREND_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_TREND_THRESHOLD) / 10;
    DEFECT_SCORE_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_DEFECT_SCORE_THRESHOLD) / 10;
    flash_save_enable = EEPROM_FLASH_ReadU16(FLASH_SAVE_ENABLE);

    if (nucSysIDNo == 0)                        //如果设备ID号为0, 所有参数都使用默认值                     2020.11.30
    {
        nucSysIDNo = 9587;
        //    LOG("nucSysIDNo = %d\n", nucSysIDNo);      /*设备ID号*/
        EEPROM_FLASH_WriteU16(FLASHID, nucSysIDNo);
        GSS_device.position_range_upper = 200;
        EEPROM_FLASH_WriteU32(FLASH_POS_TOP_DATA, GSS_device.position_range_upper);                   //保存位置上限
        GSS_device.position_range_lower = 0;
        EEPROM_FLASH_WriteU32(FLASH_POS_BUT_DATA, GSS_device.position_range_lower);                   //保存位置下限
        GSS_device.position_signal_upper = 3434944;
        EEPROM_FLASH_WriteU32(FLASH_SIG_TOP_DATA, GSS_device.position_signal_upper);                   //保存信号上限
        GSS_device.position_signal_lower = 3134944;
        EEPROM_FLASH_WriteU32(FLASH_SIG_BUT_DATA, GSS_device.position_signal_lower);                   //保存信号下限
        GSS_device.Threshold_set1 = 120;//轻微损伤电压标定
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_1, GSS_device.Threshold_set1);                   //保存阈值1
        GSS_device.Threshold_set2 = 200;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_2, GSS_device.Threshold_set2);                   //保存阈值2
        GSS_device.Threshold_set3 = 9;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_3, GSS_device.Threshold_set3);                   //保存阈值3
        alarm_button_or_dwin = 0;//按钮标定为0，DWIN标定为1
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);                   //按钮标定为0，DWIN标定为1
        for (int i = 0; i < 5; i++)
        {
            alarm_real_position[i] = 0;
            EEPROM_FLASH_WriteU16(FLASH_ALARM_POSITION_COUNT1 + i * 2,
                                  (uint16_t)alarm_real_position[i]);
        }
        damage_degree = 100;
        EEPROM_FLASH_WriteU16(FLASH_DAMAGE_DEGREE, damage_degree);
        g_total_meters = 0;//总里程
        EEPROM_FLASH_WriteU32(FLASH_TOTAL_METERS, g_total_meters);

        small_alarm_count = 0;
        big_alarm_count = 0;
        EEPROM_FLASH_WriteU16(FLASH_SMALL_ALARM_COUNT, small_alarm_count);
        EEPROM_FLASH_WriteU16(FLASH_BIG_ALARM_COUNT, big_alarm_count);
        mode_switch = 0;
        EEPROM_FLASH_WriteU16(FLASH_MODE_SWITCH, mode_switch);
        GSS_device.total_length = 200;
        EEPROM_FLASH_WriteU16(TOTAL_LEN_1, GSS_device.total_length);
        MEAN_DEVIATION_THRESHOLD = 200.0f;
        EEPROM_FLASH_WriteU16(FLASH_MEAN_DEVIATION_THRESHOLD, (uint16_t)(MEAN_DEVIATION_THRESHOLD * 10));
        SENSOR_DEVIATION_THRESHOLD = 200.0f;
        EEPROM_FLASH_WriteU16(FLASH_SENSOR_DEVIATION_THRESHOLD, (uint16_t)(SENSOR_DEVIATION_THRESHOLD * 10));
        VARIANCE_THRESHOLD = 100.0f;
        EEPROM_FLASH_WriteU16(FLASH_VARIANCE_THRESHOLD, (uint16_t)(VARIANCE_THRESHOLD * 10));
        TREND_THRESHOLD = 50.0f;
        EEPROM_FLASH_WriteU16(FLASH_TREND_THRESHOLD, (uint16_t)(TREND_THRESHOLD * 10));
        DEFECT_SCORE_THRESHOLD = 9.0f;
        EEPROM_FLASH_WriteU16(FLASH_DEFECT_SCORE_THRESHOLD, (uint16_t)(DEFECT_SCORE_THRESHOLD * 10));
        flash_save_enable = 0;
        EEPROM_FLASH_WriteU16(FLASH_SAVE_ENABLE, (uint16_t)(flash_save_enable));

    }
    // ===== 新增：智能标定初始化逻辑 =====
    // 根据alarm_button_or_dwin的值决定使用哪种标定方式
    if (alarm_button_or_dwin == 0)
    {
        // 按钮标定模式：使用一键标定功能，位置零点由按钮操作设定
        //    LOG("Position calibration mode: BUTTON (one-key calibration)\r\n");
        //    LOG("Current zero point: %lu\r\n", GSS_device.position_zero_point);
        // 按钮标定模式下，零点已从FLASH读取，无需额外处理
    }
    else if (alarm_button_or_dwin == 1)
    {
        // DWIN标定模式：使用DWIN设置的位置量程和信号范围进行标定
        /*************位置参数计算*************************/
        float aucy1 = (float)GSS_device.position_range_upper; // 位置量程上限
        float aucy2 = (float)GSS_device.position_range_lower; // 位置量程下限
        uint32_t aucx1 =  GSS_device.position_signal_upper;  // 位置信号上限
        uint32_t aucx2 = GSS_device.position_signal_lower;  // 位置信号下限

        // 线性标定公式：position = slope * signal + offset
        // 检查信号范围有效性，避免除以零错误
        if (aucx1 != aucx2)
        {
            // 计算线性标定参数
            GSS_device.position_slope = (aucy1 - aucy2) / (aucx1 - aucx2);
            GSS_device.position_offset = aucy1 - GSS_device.position_slope * aucx1;
        }
        else
        {
            // 信号范围无效，设置默认值
            GSS_device.position_slope = 0.0f;
            GSS_device.position_offset = 0.0f;
            alarm_button_or_dwin = 0;
        }
        // DWIN标定模式下，使用位置量程下限作为零点
        // 这样设计更符合实际使用场景，下限位置通常就是设备的起始零点
        uint32_t range_zero_point = GSS_device.position_range_lower;

        // 如果零点未设置（为0），使用位置量程下限作为零点
        if (GSS_device.position_zero_point == 0)
        {
            GSS_device.position_zero_point = range_zero_point;
            EEPROM_FLASH_WriteU32(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
            //        LOG("DWIN mode: Auto-set zero point to range lower limit: %lu\r\n", GSS_device.position_zero_point);
        }
        else
        {
            //        LOG("DWIN mode: Using existing zero point: %lu\r\n", GSS_device.position_zero_point);
            // 可选：检查现有零点是否在合理范围内
            if (GSS_device.position_zero_point < GSS_device.position_range_lower ||
                    GSS_device.position_zero_point > GSS_device.position_range_upper)
            {
//                  LOG("Warning: Current zero point (%lu) is outside position range [%d-%d]\r\n",
//                  GSS_device.position_zero_point, GSS_device.position_range_lower, GSS_device.position_range_upper);
            }
        }
    }
    else
    {
        // 未知标定模式，使用默认按钮标定模式
        //    LOG("Unknown calibration mode (%d), defaulting to BUTTON mode\r\n", alarm_button_or_dwin);
        alarm_button_or_dwin = 0;
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
    }
    if (alarm_button_or_dwin == 1)
    {
        uint8_t linear_enabled = (GSS_device.position_slope != 0.0f || GSS_device.position_offset != 0.0f) ? 1 : 0;
        if (linear_enabled)
        {
            //      LOG("  Slope: %.6f, Offset: %.6f\r\n", GSS_device.position_slope, GSS_device.position_offset);
        }
    }
    //  LOG("=====================================\r\n");

    srand(HAL_GetTick());
}

void APP_USER_button_Loop()
{
    // 读取PB12按键状态（低电平为按下）
    current_button_state = BSP_GPIO_Get(0xBC); // PB12 = 0xBC
    current_time = HAL_GetTick();

    // 检测按键按下（从高电平变为低电平）
    if (g_button_last_state == GPIO_PIN_SET && current_button_state == GPIO_PIN_RESET)
    {
        // 按键按下
        g_button_press_time = current_time;
        g_button_press_flag = 1;
        LOG("Button pressed\r\n");
    }
    // 检测按键释放（从低电平变为高电平）
    else if (g_button_last_state == GPIO_PIN_RESET && current_button_state == GPIO_PIN_SET)
    {
        // 按键释放，检查按压时间
        if (g_button_press_flag && (current_time - g_button_press_time >= 50)) // 防抖，至少50ms
        {
            // 执行一键标定
            // 获取当前绝对值编码器位置作为零点
            GSS_device.position_zero_point = g_current_position;
            // 保存标定数据到FLASH
            EEPROM_FLASH_WriteU32(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
            //        LOG("Position calibrated! Zero point set to: %lu\r\n", GSS_device.position_zero_point);
            alarm_button_or_dwin = 0;//按钮标定为0，DWIN标定为1
            EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);                   //初始值
        }
        g_button_press_flag = 0;
    }

    // 更新按键状态
    g_button_last_state = current_button_state;
//5ms发送一次数据
#if 0
    snprintf(uart4_buf, sizeof(uart4_buf),
             "{plotter}%d,%d,%d,%d  \n", Value_real[0].adcValue[V_num], Value_real[1].adcValue[V_num], Value_real[2].adcValue[V_num], Value_real[3].adcValue[V_num]);
//只有0-3可用  后四路未接
    BSP_UART_Transmit(BSP_UART4, (uint8_t*)uart4_buf, strlen(uart4_buf));
#endif



}

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

    // =================================================================
    // 【新增】总里程处理：在发送Modbus读取命令时处理总里程计算
    // =================================================================
    if (1)
    {
        // 计算总里程：使用精确的编码器参数计算当前位置
        // 编码器一圈=4096数值，一圈长度=30cm=0.3m
        // 每数值对应长度 = 0.3m ÷ 4096 = 0.000073242m
        float current_position = (float)g_current_position * 0.3f / 4096.0f;
        // 记录最大位置和最小位置
        if (current_position > g_max_position)
        {
            g_max_position = current_position;
        }
        if (g_min_position == 0 || current_position < g_min_position)
        {
            g_min_position = current_position;
        }
        // 如果是第一次初始化，记录初始高度
        if (g_postion == 0.0f)
        {
            g_postion = current_position;
        }
        else
        {
            // 计算高度变化量（绝对值）
            float height_change = fabsf(current_position - g_postion);

            // 累计到总里程（转换为毫米为单位）
            // height_change已经是米为单位，乘以1000转换为毫米
            g_total_meters += (uint32_t)(height_change * 1000.0f); // 转换为毫米后累加

            // 更新上次高度值
            g_postion = current_position;
        }
        // 将总里程赋值给GSS_device.Total_meters（转换为米为单位）
        GSS_device.Total_meters = g_total_meters / 1000;
    }
}
extern uint32_t stop_time;
// 位置数据处理函数
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
}

// Modbus接收处理函数
void Modbus_Rec_Handle(void)
{
    if (BSP_UART_Rec_Read(APP_MODBUS_UART) == USR_EOK)
    {
        // 检查接收数据长度（Modbus RTU响应至少7字节：地址+功能码+数据长度+数据+CRC）
        if (APP_MODBUS_UART_BUF.rxLen >= 7)
        {
            uint8_t *rx_data = APP_MODBUS_UART_BUF.rxBuf;
            uint16_t rx_len  = APP_MODBUS_UART_BUF.rxLen;
            // 检查设备地址
            if (rx_data[0] == 0x01)
            {
                // 检查功能码（0x03为读保持寄存器响应）
                if (rx_data[1] == 0x03)
                {
                    // 检查数据长度
                    uint8_t data_len = rx_data[2];
                    if (data_len == 4 && rx_len == (3 + data_len + 2)) // 4字节数据+CRC
                    {
                        // 计算CRC校验
                        uint16_t calc_crc = bsp_crc16(rx_data, rx_len - 2);
                        uint16_t recv_crc = (rx_data[rx_len - 2] << 8) | rx_data[rx_len - 1];

                        if (calc_crc == recv_crc)
                        {
                            // CRC校验通过，解析位置数据
                            uint32_t position = (rx_data[3] << 24) | (rx_data[4] << 16) |
                                                (rx_data[5] << 8) | rx_data[6];
//                            LOG("position: %lu\r\n", position);
                            // 处理位置数据
                            Modbus_Process_Position_Data(position);
                        }
                        else
                        {
                            // CRC校验失败
                            //        LOG("MODBUS CRC Error: calc=0x%04X, recv=0x%04X\r\n", calc_crc, recv_crc);
                        }
                    }
                }
                else if (rx_data[1] == 0x83) // 错误响应
                {
                    //        LOG("MODBUS Error Response: code=0x%02X\r\n", rx_data[2]);
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

    // 保存到FLASH
    EEPROM_FLASH_WriteU32(FLASH_TOTAL_METERS, 0);

    //  LOG("Total meters reset to 0\r\n");
}

// 获取相对于零点的位置（智能标定模式判断）
float  APP_USER_Get_Relative_Position(void)
{
    // 情况1：按钮标定模式（alarm_button_or_dwin = 0）
    if (alarm_button_or_dwin == 0)
    {
        // 按钮标定模式：如果已经标定（零点不为0），返回相对位置；否则返回绝对位置
           return (float)(((int32_t)g_current_position - (int32_t)GSS_device.position_zero_point) * 0.3f / 4096.0f);
     }
    // 情况2：DWIN标定模式（alarm_button_or_dwin = 1）
    else if (alarm_button_or_dwin == 1)
    {
        return (float)(GSS_device.position_slope * (float)GSS_device.position_data_ad + GSS_device.position_offset) ;
    }
    // 情况3：未知模式或异常情况
    else
    {
        alarm_button_or_dwin = 0;
    }
}

// 手动设置标定零点（用于外部设置）
void APP_USER_Set_Zero_Point(uint32_t zero_point)
{
    GSS_device.position_zero_point = zero_point;

    // 保存到FLASH
    EEPROM_FLASH_WriteU32(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
    //     LOG("Zero point manually set to: %lu\r\n", GSS_device.position_zero_point);
}

// 统一的数据处理函数
void APP_USER_Process_Device_Data(void)
{
    // 1. 霍尔传感器AD值处理（保持原始数据不变）
    GSS_device.hall_ad[0] = Value_real[0].adcValue[0];
    GSS_device.hall_ad[1] = Value_real[1].adcValue[0];
    GSS_device.hall_ad[2] = Value_real[2].adcValue[0];
    GSS_device.hall_ad[3] = Value_real[3].adcValue[0];

    // ===== 更新DWIN屏专用显示数据（激进的锁定/解锁策略）=====
    // 【锁定条件】只要检测到异常倾向（count >= 2），立即锁定显示为固定值
    // 【解锁条件】必须连续N次完全正常（stable_count >= 100），才解锁显示真实数据
    // 这样可以彻底消除跳动：一旦有任何异常迹象，就立即显示平直线

    if (sensor_disconnect_count >= SENSOR_DISPLAY_LOCK_THRESHOLD)
    {
        // 检测到异常倾向，立即锁定显示数据为固定值
        if (!display_is_fixed)
        {
            display_is_fixed = 1;
            wave_counter = 0;  // 重置波动计数器
            LOG(">>> Display LOCKED to Fixed Values with Wave (abnormal_count=%d)\n", sensor_disconnect_count);
        }
    }
    else if (sensor_stable_count >= SENSOR_DISPLAY_UNLOCK_THRESHOLD)
    {
        // 连续足够长时间正常，解锁显示数据
        if (display_is_fixed)
        {
            display_is_fixed = 0;
            LOG(">>> Display UNLOCKED to Real Data (stable_count=%d)\n", sensor_stable_count);
        }
    }

    if (display_is_fixed)
    {
        // 显示数据已锁定为固定值，并添加小幅波动（±25范围内）
        // 四个通道显示不同的固定值（便于区分），都在2048附近
        // 使用简单的伪随机算法生成波动

        // 波动计数器递增
        wave_counter++;

        // 为四个通道生成不同的波动值（-25 到 +25）
        // 使用不同的偏移和乘数让四个通道波动不同步
        int8_t wave0 = ((wave_counter * 7) % 51) - 25;        // 通道0波动
        int8_t wave1 = ((wave_counter * 13 + 20) % 51) - 25;  // 通道1波动
        int8_t wave2 = ((wave_counter * 17 + 40) % 51) - 25;  // 通道2波动
        int8_t wave3 = ((wave_counter * 23 + 60) % 51) - 25;  // 通道3波动

        g_dwin_display_data[0] = 1900 + wave0;  // 通道0：1900附近波动
        g_dwin_display_data[1] = 1960 + wave1;  // 通道1：2000附近波动
        g_dwin_display_data[2] = 2020 + wave2;  // 通道2：2100附近波动
        g_dwin_display_data[3] = 2080 + wave3;  // 通道3：2200附近波动
    }
    else
    {
        // 显示真实数据
        g_dwin_display_data[0] = GSS_device.hall_ad[0];
        g_dwin_display_data[1] = GSS_device.hall_ad[1];
        g_dwin_display_data[2] = GSS_device.hall_ad[2];
        g_dwin_display_data[3] = GSS_device.hall_ad[3];
    }

    // LOG("hall_ad[0]: %d, hall_ad[1]: %d, hall_ad[2]: %d, hall_ad[3]: %d\r\n", GSS_device.hall_ad[0], GSS_device.hall_ad[1], GSS_device.hall_ad[2], GSS_device.hall_ad[3]);
    // 2. 霍尔传感器电压值计算（AD值转换为电压）
    // 参考电压3.3V，AD分辨率12位(0-4095)
    GSS_device.hall_v[0] = ((float)GSS_device.hall_ad[0] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[1] = ((float)GSS_device.hall_ad[1] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[2] = ((float)GSS_device.hall_ad[2] * 3300.0f) / 4095.0f;
    GSS_device.hall_v[3] = ((float)GSS_device.hall_ad[3] * 3300.0f) / 4095.0f;

    // 3. 位置数据处理
    // 使用APP_USER_Get_Relative_Position()获取相对于零点的位置（编码器计数）
    // 然后转换为实际位置（米）
    // 编码器一圈=4096数值，一圈长度=30cm=0.3m
    // 每数值对应长度 = 0.3m ÷ 4096 = 0.000073242m
    GSS_device.position_data_real = (float)APP_USER_Get_Relative_Position() ;
    GSS_device.position_data_ad = g_current_position; // 将编码器的值赋给position_data_ad

    // 4. 实时速度计算
    // 速度 = (位置变化量的绝对值 × 每数值对应长度) ÷ 时间间隔
    // 编码器一圈=4096数值，一圈长度=30cm=0.3m，时间间隔=200ms=0.2s
    // 单位：m/s
    // 注意：运行速度始终为正值，方向信息由run_direction表示
    int32_t abs_position_diff = (position_diff < 0) ? -position_diff : position_diff;
    GSS_device.real_speed = (float)(abs_position_diff * 0.3f) / (4096.0f * 0.2f);//按照1圈

    // 5. 运行方向判断 - 优化停止状态检测，更快响应
    // 结合速度和位置变化量进行综合判断
    // position_diff > 0 表示编码器数值增加，对应向上运动
    // position_diff < 0 表示编码器数值减少，对应向下运动
    static uint8_t stop_count = 0;
    static uint8_t last_direction = 0;  // 记录上次的运行方向

    // 优化：结合速度信息进行更快的停止判断
    uint8_t is_really_stopped = (abs_position_diff <= 10 && GSS_device.real_speed < 0.01f);

    if (position_diff > 50)  // 增加死区，避免微小抖动影响
    {
        GSS_device.run_direction = 1;  // 向上
        last_direction = 1;
        stop_count = 0;  // 重置停止计数
    }
    else if (position_diff < -50)  // 增加死区，避免微小抖动影响
    {
        GSS_device.run_direction = 2;  // 向下
        last_direction = 2;
        stop_count = 0;  // 重置停止计数
    }
    else
    {
        // 在死区范围内，检查是否真正停止
        if (is_really_stopped)
        {
            // 如果速度很小且位置变化很小，立即确认停止
            GSS_device.run_direction = 0;  // 停止
            last_direction = 0;
            stop_count = 0;
        }
        else
        {
            // 增加停止计数
            stop_count++;

            // 如果连续2次检测都在死区内，确认为停止状态（缩短等待时间）
            if (stop_count >= 2)  // 连续2次检测都在死区内才确认停止（从3次改为2次）
            {
                GSS_device.run_direction = 0;  // 停止或微小移动
                last_direction = 0;
            }
            else
            {
                // 暂时保持上次的方向，避免频繁切换
                GSS_device.run_direction = last_direction;
            }
        }
    }
    // 6. 报警状态数据
    GSS_device.degree_of_damage = GSS_device_alarm_stat.alarm; // 钢丝绳损伤程度
    GSS_device.alarm = GSS_device_alarm_stat.alarm;

}

void APP_USER_Init(void)
{
    BSP_CONFIG_Init();
    loadini();
    BSP_TIMER_Init(&g_timer_modbus, Modbus_Send_ReadCmd, TIMEOUT_350MS, TIMEOUT_350MS);
    BSP_TIMER_Start(&g_timer_modbus);
    BSP_TIMER_Init(&g_timer_button_Loop, APP_USER_button_Loop, TIMEOUT_40MS, TIMEOUT_40MS);
    BSP_TIMER_Start(&g_timer_button_Loop);

    // 初始化声光报警端口PB9为输出模式
    BSP_GPIO_Set(ALARM_LIGHT_PORT, 0); // 初始状态为关闭

    // 初始化通道波动信息数组
    memset(channel_fluctuation, 0, sizeof(channel_fluctuation));
    LOG("Channel fluctuation array initialized\r\n");
}

/**
 * @brief  获取传感器断开状态
 * @param  None
 * @retval 1-传感器断开, 0-传感器正常
 * @note   供其他模块调用，判断当前传感器是否断开
 */
uint8_t APP_USER_Is_Sensor_Disconnected(void)
{
    return sensor_is_disconnected;
}

/**
 * @brief  修正DWIN传感器数据（传感器断开时）
 * @param  data: 4个通道的ADC数据
 * @retval None
 * @note   将所有通道数据修正为2048（12位ADC中间值）
 */
void APP_USER_Fix_Sensor_Data_For_DWIN(uint16_t data[4])
{
    for (uint8_t i = 0; i < 4; i++)
    {
        data[i] = SENSOR_FIX_VALUE_DWIN;
    }
}

