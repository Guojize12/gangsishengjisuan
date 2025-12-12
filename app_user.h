#ifndef __APP_USER_H
#define __APP_USER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"
#include "app_config.h"

// FLASH存储地址定义
#if 0
// 旧的FLASH存储地址定义

#else

#define     FLASHID                   (2*30) //dtu ip 
#define     FLASH_POS_TOP_DATA        (2*32)  /*位置量程下限-8000   2020-03-22*/
#define     FLASH_POS_BUT_DATA        (2*34)  /*位置信号下限-8002   2020-03-22*/
#define     FLASH_SIG_TOP_DATA        (2*36)  /*位置量程上限-8004   2020-03-22*/
#define     FLASH_SIG_BUT_DATA        (2*38)  /*位置信号上限-8006   2020-03-22*/

#define     FLASH_POSITION_ZERO_POINT   (2*40)  /*位置标定零点-8010   2025-01-XX*/
#define     FLASH_TOTAL_METERS          (2*42)  /*总里程累计值-8014   2025-01-XX*/

#define     FLASH_THRESHOLD_1           (2*44)  /*轻微损伤电压标定-8022   2025-01-XX*/
#define     FLASH_THRESHOLD_2           (2*46)  /*严重损伤电压标定-8024   2025-01-XX*/
#define     FLASH_BUTTON_OR_DWIN            (2*48)  /*按钮或DWIN 标定-8026   2025-01-XX*/
#define     FLASH_ALARM_POSITION_COUNT1     (2*50)  /*报警位置统计计数-8028   2025-01-XX*/
#define     FLASH_ALARM_POSITION_COUNT2     (2*52)  /*报警位置统计计数-8028   2025-01-XX*/
#define     FLASH_ALARM_POSITION_COUNT3     (2*54)  /*报警位置统计计数-8028   2025-01-XX*/
#define     FLASH_ALARM_POSITION_COUNT4     (2*56)  /*报警位置统计计数-8028   2025-01-XX*/
#define     FLASH_ALARM_POSITION_COUNT5     (2*58)  /*报警位置统计计数-8028   2025-01-XX*/
#define     FLASH_DAMAGE_DEGREE             (2*60)  /*损伤度数据-8034   aaaaaa2025-01-XX*/
#define     FLASH_SMALL_ALARM_COUNT         (2*62)  /*轻微损伤次数-8036   aaaaa2025-01-XX*/
#define     FLASH_BIG_ALARM_COUNT           (2*64)  /*严重损伤次数-8038   aaaaa2025-01-XX*/
#define     FLASH_THRESHOLD_3               (2*66)  /*宽度标定-8038   2025-01-XX*/
#define     TOTAL_LEN_1                     (2*68)  /*绳总长-8040   2025-01-XX*/
#define     FLASH_MODE_SWITCH               (2*70)  /*模式开关-8042   2025-01-XX*/
#define     FLASH_SAVE_ENABLE               (2*72)  /*保存开关*/
// 卡尔曼滤波配置参数存储地址
#define     FLASH_KALMAN_CONFIG_MAGIC_NUMBER       (2*80)  /*卡尔曼配置魔术数      2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_BASELINE_MEAN_0       (2*82)  /*卡尔曼配置基线均值0   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_BASELINE_MEAN_1       (2*84)  /*卡尔曼配置基线均值1   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_BASELINE_MEAN_2       (2*86)  /*卡尔曼配置基线均值2   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_BASELINE_MEAN_3       (2*88)  /*卡尔曼配置基线均值3   2025-01-XX*/

#define     FLASH_KALMAN_CONFIG_BASELINE_STD_0       (2*90)  /*卡尔曼配置基线标准差0   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_BASELINE_STD_1       (2*92)  /*卡尔曼配置基线标准差1   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_BASELINE_STD_2       (2*94)  /*卡尔曼配置基线标准差2   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_BASELINE_STD_3       (2*96)  /*卡尔曼配置基线标准差3   2025-01-XX*/

#define     FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_0       (2*98)  /*卡尔曼配置自适应阈值0   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_1       (2*100)  /*卡尔曼配置自适应阈值1   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_2       (2*102)  /*卡尔曼配置自适应阈值2   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_3       (2*104)  /*卡尔曼配置自适应阈值3   2025-01-XX*/

#define     FLASH_KALMAN_CONFIG_PROCESS_NOISE_SCALE       (2*106)  /*卡尔曼配置过程噪声尺度   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_MEASUREMENT_NOISE_SCALE       (2*108)  /*卡尔曼配置测量噪声尺度   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_WARMUP_SAMPLE_COUNT       (2*110)  /*卡尔曼配置预热样本数   2025-01-XX*/
#define     FLASH_KALMAN_CONFIG_CHECKSUM        (2*112)  /*卡尔曼配置校验和   2025-01-XX*/

#define     FLASH_MEAN_DEVIATION_THRESHOLD      (2*116)  /*均值偏差阈值   2025-01-XX*/
#define     FLASH_SENSOR_DEVIATION_THRESHOLD    (2*118)  /*单个传感器异常检测阈值   2025-01-XX*/
#define     FLASH_VARIANCE_THRESHOLD            (2*120)  /*方差异常检测阈值   2025-01-XX*/
#define     FLASH_TREND_THRESHOLD               (2*122)  /*趋势异常检测阈值   2025-01-XX*/
#define     FLASH_DEFECT_SCORE_THRESHOLD        (2*124)  /*缺陷得分阈值   2025-01-XX*/

#define     APP_IAP_ADDR_STATUS_OTA                (2*126)  /*OTA升级标志位   2025-01-XX*/
// OTA标志值定义
#define     OTA_FLAG_MAGIC_NUMBER         1  /*OTA升级魔术数，Bootloader检测到此值则进入升级模式*/

#endif

typedef struct
{
    uint16_t adcValue[50];  // 更新为新的滑动窗口长度
    bsp_rtc_def real_rtc;

} bsp_dat_def;



// Modbus RTU UART1 定义
#define APP_MODBUS_UART          BSP_UART1
#define APP_MODBUS_UART_BUF   g_uart_buf[APP_MODBUS_UART]

#define CHANNEL_NUM 8
#define DATA_BUF_LEN 50


void loadini(void);
void APP_DTU_Remote_Head_Init(void);
extern void ADC128S052_CS_DISABLE(void);
extern void ADC128S052_CS_ENABLE(void);
extern char uart4_buf[500];
// #define APP_4G_UART          BSP_UART3
//#define   APP_4G_UART_BUF          g_uart_buf[APP_4G_UART]

//定义结构体数组，数组元素为报警时间，报警位置，报警类型
typedef struct
{
    char time[64];
    float position;//报警位置对应的时报警时对应的GSS_device_alarm_stat.position_data
    char type;  //报警类型只有 0 1 2，0表示无损伤，1表示轻微损伤，2表示严重损伤
    uint16_t positive_magnitude; //报警的幅度positive_magnitude
} AlarmInfo;


#define LED1_toggle()          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9)
#define LED_toggle()           HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8)

void APP_USER_Init(void);
void APP_USER_ADC_Loop(void);
void APP_USER_Rx_Handle(void);
void Modbus_Rec_Handle(void);  // Modbus接收处理函数

// 位置数据处理函数
void Modbus_Process_Position_Data(uint32_t position);

// 位置数据获取函数
uint32_t Modbus_Get_Current_Position(void);
uint32_t Modbus_Get_Last_Position(void);
uint32_t Modbus_Get_Position_Change_Count(void);
int32_t Modbus_Get_Position_Diff(void);
uint32_t Modbus_Get_Max_Position(void);
uint32_t Modbus_Get_Min_Position(void);
uint8_t Modbus_Is_Position_Initialized(void);
void Modbus_Reset_Position_Change_Count(void);
void Modbus_Reset_Position_Initialized(void);

void APP_USER_Ip_Set(void);


// 位置标定相关函数

uint8_t APP_USER_Is_Position_Calibrated(void);
float APP_USER_Get_Relative_Position(void);
void APP_USER_Set_Zero_Point(uint32_t zero_point);
void detect_alarm(uint16_t data_filt[8][50]);  // 更新为新的滑动窗口长度

// 总里程相关函数
uint32_t APP_USER_Get_Total_Meters(void);
void APP_USER_Reset_Total_Meters(void);
void    APP_USER_Process_Device_Data(void);

// 新增：每路通道极值统计函数
uint16_t APP_USER_Get_Channel_Max_Value(uint8_t channel);
uint16_t APP_USER_Get_Channel_Min_Value(uint8_t channel);
void APP_USER_Reset_Channel_Extremes(void);
void APP_USER_Get_All_Channel_Stats(void);

// 新增：每路通道宽度统计函数
uint16_t APP_USER_Get_Channel_Current_Width(uint8_t channel);
uint16_t APP_USER_Get_Channel_Max_Width(uint8_t channel);
uint32_t APP_USER_Get_Channel_Width_Update_Count(uint8_t channel);

// 新增：钢丝绳探伤结果获取函数
uint16_t APP_USER_Get_Channel_Positive_Magnitude(uint8_t channel);
uint16_t APP_USER_Get_Channel_Negative_Magnitude(uint8_t channel);
uint16_t APP_USER_Get_Channel_Fluctuation_Width(uint8_t channel);
uint16_t APP_USER_Get_Channel_Baseline_Value(uint8_t channel);
uint16_t APP_USER_Get_Channel_Peak_Positive(uint8_t channel);
uint16_t APP_USER_Get_Channel_Peak_Negative(uint8_t channel);
uint8_t APP_USER_Get_Channel_Anomaly_Detected(uint8_t channel);
uint8_t APP_USER_Get_Current_Alarm_Level(void);

// 新增：探伤统计信息获取函数
void APP_USER_Get_All_Channel_Damage_Stats(void);

// 传感器断开检测相关函数和定义
#define SENSOR_FIX_VALUE_DWIN  2048   // DWIN屏传感器断开时的固定输出值
uint8_t APP_USER_Is_Sensor_Disconnected(void);  // 获取传感器断开状态
void APP_USER_Fix_Sensor_Data_For_DWIN(uint16_t data[4]);  // 修正DWIN传感器数据

// DWIN屏专用显示数据（全局变量）
extern uint16_t g_dwin_display_data[4];  // DWIN屏曲线显示专用数据


#ifdef __cplusplus
}
#endif
#endif  /* __APP_USER_H */

