#ifndef __APP_USER_H
#define __APP_USER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"
#include "app_config.h"

// FLASH存储地址定义

#define     FLASHID_nouse                   (2*30) //dtu ip 
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
#define     FLASH_KALMAN_CONFIG_CHECKSUM       (2*112)  /*卡尔曼配置校验和   2025-01-XX*/

#define     FLASH_MEAN_DEVIATION_THRESHOLD       (2*116)  /*均值偏差阈值   2025-01-XX*/
#define     FLASH_SENSOR_DEVIATION_THRESHOLD       (2*118)  /*单个传感器异常检测阈值   2025-01-XX*/
#define     FLASH_VARIANCE_THRESHOLD       (2*120)  /*方差异常检测阈值   2025-01-XX*/
#define     FLASH_TREND_THRESHOLD       (2*122)  /*趋势异常检测阈值   2025-01-XX*/
#define     FLASH_DEFECT_SCORE_THRESHOLD       (2*124)  /*缺陷得分阈值   2025-01-XX*/

#define     APP_IAP_ADDR_STATUS_OTA                (2*126)  /*OTA升级标志位   2025-01-XX*/

//app中设置的ota
// EEPROM_FLASH_WriteU32(FLASH_OTA_FLAG, OTA_FLAG_MAGIC_NUMBER);
//OTA_START
#define		FLASH_MODEL	  		      	(2*128)	//产品型号位置
#define		FLASH_REMOTE	  		   	(2*130)	//连接服务器:SN,MODEL,VER位置  3X4B
#define		FLASH_OTA_MODEL	  		 	(2*132)	//OTA型号位置

#define     FLASHID           			  (2*134)   //dtu ip 
#define     FLASH_DTU_IP              	  (2*136)   //dtu ip 
#define     WORK_ID                		  (2*138)
#define     APP_IAP_ADDR_RESET            (2*140)
#define     APP_IAP_ADDR_LOG              (2*142)
#define   	APP_IAP_ADDR_OTA_BASE         (2*144)

#define APP_IAP_ADDR_WRITE_BACK			  (APP_IAP_ADDR_OTA_BASE+2)			 //回写次数  		2*150
#define APP_IAP_ADDR_APP_OK				  (APP_IAP_ADDR_OTA_BASE+4)			 //APP成功运行记录	2*152
										  
#define APP_IAP_ADDR_APP_SIZE  			  (APP_IAP_ADDR_OTA_BASE+10)      //升级固件 大小		2*158
#define APP_IAP_ADDR_APP_MD5  			  (APP_IAP_ADDR_OTA_BASE+14)      //升级固件 MD5		2*162
#define APP_IAP_ADDR_APP_VERSION  		  (APP_IAP_ADDR_OTA_BASE+30)  	  //升级 版本号			2*178
										  

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


void APP_DTU_Remote_Head_Init(void);

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
void APP_USER_Rx_Handle(void);


void APP_USER_Ip_Set(void);
void APP_USER_Id_Get(void);



#ifdef __cplusplus
}
#endif
#endif  /* __APP_USER_H */

