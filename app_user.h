#ifndef __APP_USER_H
#define __APP_USER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 功能概览（业务层）
   - APP_USER_Init: 业务初始化，启动Modbus与按键定时器，加载参数，初始化通道状态与声光报警端口
   - APP_USER_ADC_Loop: 采样→断线检测→系统稳定判定→报警等级计算→设备数据处理→报警记录与输出→蜂鸣任务
   - 按键两步标定状态机（APP_USER_button_Loop）：
       第一次按键：设零点、模式置预热、重置总预热超时计时器、响一声；
       1分钟未二次按键：自动切检测但不保存参数、响两声；
       10分钟未二次按键：超时回退等待第一次；
       第二次按键：保存上限、计算斜率/偏置、模式置检测、更新里程、响两声。
   - 显示与方向：锁定/解锁显示、DWIN显示数据维护、运行方向更新
   - 位置/速度/里程：位置与速度在 position.c 计算；此处统一同步与保存
*/

typedef struct AlarmInfo AlarmInfo;

#include "bsp_config.h"
#include "app_config.h"

#define     FLASHID                   (2*30)
#define     FLASH_POS_TOP_DATA        (2*32)
#define     FLASH_POS_BUT_DATA        (2*34)
#define     FLASH_SIG_TOP_DATA        (2*36)
#define     FLASH_SIG_BUT_DATA        (2*38)

#define     FLASH_POSITION_ZERO_POINT (2*40)
#define     FLASH_TOTAL_METERS        (2*42)

#define     FLASH_THRESHOLD_1         (2*44)
#define     FLASH_THRESHOLD_2         (2*46)
#define     FLASH_BUTTON_OR_DWIN      (2*48)
#define     FLASH_ALARM_POSITION_COUNT1 (2*50)
#define     FLASH_ALARM_POSITION_COUNT2 (2*52)
#define     FLASH_ALARM_POSITION_COUNT3 (2*54)
#define     FLASH_ALARM_POSITION_COUNT4 (2*56)
#define     FLASH_ALARM_POSITION_COUNT5 (2*58)
#define     FLASH_DAMAGE_DEGREE       (2*60)
#define     FLASH_SMALL_ALARM_COUNT   (2*62)
#define     FLASH_BIG_ALARM_COUNT     (2*64)
#define     FLASH_THRESHOLD_3         (2*66)
#define     TOTAL_LEN_1               (2*68)
#define     FLASH_MODE_SWITCH         (2*70)
#define     FLASH_SAVE_ENABLE         (2*72)

#define     FLASH_KALMAN_CONFIG_MAGIC_NUMBER           (2*80)
#define     FLASH_KALMAN_CONFIG_BASELINE_MEAN_0        (2*82)
#define     FLASH_KALMAN_CONFIG_BASELINE_MEAN_1        (2*84)
#define     FLASH_KALMAN_CONFIG_BASELINE_MEAN_2        (2*86)
#define     FLASH_KALMAN_CONFIG_BASELINE_MEAN_3        (2*88)

#define     FLASH_KALMAN_CONFIG_BASELINE_STD_0         (2*90)
#define     FLASH_KALMAN_CONFIG_BASELINE_STD_1         (2*92)
#define     FLASH_KALMAN_CONFIG_BASELINE_STD_2         (2*94)
#define     FLASH_KALMAN_CONFIG_BASELINE_STD_3         (2*96)

#define     FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_0   (2*98)
#define     FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_1   (2*100)
#define     FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_2   (2*102)
#define     FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_3   (2*104)

#define     FLASH_KALMAN_CONFIG_PROCESS_NOISE_SCALE    (2*106)
#define     FLASH_KALMAN_CONFIG_MEASUREMENT_NOISE_SCALE (2*108)
#define     FLASH_KALMAN_CONFIG_WARMUP_SAMPLE_COUNT    (2*110)
#define     FLASH_KALMAN_CONFIG_CHECKSUM               (2*112)

#define     FLASH_MEAN_DEVIATION_THRESHOLD             (2*116)
#define     FLASH_SENSOR_DEVIATION_THRESHOLD           (2*118)
#define     FLASH_VARIANCE_THRESHOLD                   (2*120)
#define     FLASH_TREND_THRESHOLD                      (2*122)
#define     FLASH_DEFECT_SCORE_THRESHOLD               (2*124)

#define     APP_IAP_ADDR_STATUS_OTA                    (2*126)
#define     OTA_FLAG_MAGIC_NUMBER                      1

/* 声光报警器端口与时序配置（统一常量） */
#define  ALARM_LIGHT_PORT       0xB9        // PB9端口编号
#define  ALARM_LIGHT_DURATION   200         // 声光报警持续时间(ms)

typedef struct
{
    uint16_t   adcValue[50];
    bsp_rtc_def real_rtc;
} bsp_dat_def;

typedef enum 
{
    BEEP_IDLE = 0,
    BEEP_ON,        // 当前声：继电器拉高
    BEEP_OFF,       // 间隔期：继电器拉低
} BeepState;

// Modbus RTU UART1 定义
#define APP_MODBUS_UART        BSP_UART1
#define APP_MODBUS_UART_BUF    g_uart_buf[APP_MODBUS_UART]

#define CHANNEL_NUM  8
#define DATA_BUF_LEN 50

void loadini(void);
void APP_DTU_Remote_Head_Init(void);
extern void ADC128S052_CS_DISABLE(void);
extern void ADC128S052_CS_ENABLE(void);
extern char uart4_buf[500];

// 报警信息结构
struct AlarmInfo
{
    char   time[64];
    float  position;              // 报警位置（米）
    char   type;                  // 报警类型：0无损伤，1轻微，2严重
    uint16_t positive_magnitude;  // 报警幅度
};

typedef enum 
{
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_PRESSED,
    BUTTON_EVENT_RELEASED
} ButtonEvent;

ButtonEvent Button_DetectEvent(void);

// 统一为 uint16_t
extern uint16_t alarm_button_or_dwin;

#define LED1_toggle() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9)
#define LED_toggle()  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8)

/* 业务主接口 */
void APP_USER_Init(void);
void APP_USER_ADC_Loop(void);
void APP_USER_Rx_Handle(void);       // 预留：UART/外设接收处理（当前占位）
void Modbus_Rec_Handle(void);        // Modbus接收处理

/* 位置数据处理函数 */
void Modbus_Process_Position_Data(uint32_t position);

/* 位置数据获取函数 */
uint32_t Modbus_Get_Current_Position(void);
uint32_t Modbus_Get_Last_Position(void);
uint32_t Modbus_Get_Position_Change_Count(void);
int32_t  Modbus_Get_Position_Diff(void);
void     Modbus_Reset_Position_Change_Count(void);

/* IP与按键/标定 */
void APP_USER_Ip_Set(void);

/* 位置标定相关函数 */
uint8_t APP_USER_Is_Position_Calibrated(void);
float   APP_USER_Get_Relative_Position(void);
void    APP_USER_Set_Zero_Point(uint32_t zero_point);
void    detect_alarm(uint16_t data_filt[8][50]);

/* 总里程相关函数 */
uint32_t APP_USER_Get_Total_Meters(void);
void     APP_USER_Reset_Total_Meters(void);
void     APP_USER_Process_Device_Data(void);

/* 传感器断开检测相关 */
#define SENSOR_FIX_VALUE_DWIN  2048
uint8_t APP_USER_Is_Sensor_Disconnected(void);
void    APP_USER_Fix_Sensor_Data_For_DWIN(uint16_t data[4]);

/* DWIN屏专用显示数据（全局变量） */
extern uint16_t g_dwin_display_data[4];

/* 编码器每圈计数 */
#define ENCODER_COUNTS_PER_REV 4096

void APP_USER_Mileage_Flash_Save_Handle(void);
void FLASH_WriteU32_WithCheck(uint16_t addr, uint32_t value);

#ifdef __cplusplus
}
#endif
#endif  /* __APP_USER_H */
