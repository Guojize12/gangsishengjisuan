#ifndef __APP_KALMAN_H__
#define __APP_KALMAN_H__

#include "app_config.h"
#include <stdbool.h>
#include <stdint.h>
// ===== 新增：全局通道波动信息结构体 =====
typedef struct
{
    uint16_t fluctuation_magnitude_positive;    // 波动幅度正半波幅度（最大值-基线）
    uint16_t fluctuation_magnitude_negative;    // 波动幅度负半波幅度（基线-最小值）
    uint16_t fluctuation_width;        // 波动宽度（采样点数），从大于或小于正负阈值开始计算，到大于或小于正负阈值结束计算
    uint16_t baseline_value;           // 基线值（正常状态下的ADC值）
    uint16_t peak_value_positive;      // 正峰值（异常状态下的ADC值）
    uint16_t peak_value_negative;      // 负峰值（异常状态下的ADC值）
    uint8_t  anomaly_detected;         // 是否检测到异常

    // 新增字段：用于极值统计
    uint8_t  initialized;              // 是否已初始化
    uint16_t channel_max_value;        // 通道历史最大值
    uint16_t channel_min_value;        // 通道历史最小值
    uint32_t max_update_count;         // 最大值更新次数
    uint32_t min_update_count;         // 最小值更新次数

    // 新增字段：用于区域检测
    uint8_t  in_max_region;            // 是否在最大值区域
    uint8_t  in_min_region;            // 是否在最小值区域
    uint16_t max_start;                // 最大值区域起始位置
    uint16_t max_end;                  // 最大值区域结束位置
    uint16_t min_start;                // 最小值区域起始位置
    uint16_t min_end;                  // 最小值区域结束位置

    // 新增字段：用于宽度管理
    uint16_t current_fluctuation_width; // 当前波动宽度
    uint16_t max_fluctuation_width;    // 历史最大波动宽度
    uint32_t width_update_count;       // 宽度更新次数
} channel_fluctuation_t;

// 对外可见的全局数组

extern gss_device_alarm_stat GSS_device_alarm_stat  ;      // 显式初始化为0
extern gss_device_alarm_stat GSS_device_alarm_stat_temp ;  // 显式初始化为0
extern gss_device_alarm_stat GSS_device_alarm_stat_dwin ; // 显式初始化为0
extern gss_device  GSS_device ;        // 显式初始化为0，确保所有字段初始值正确
extern uint8_t alarm_light_trig; // 声光报警触发标志
extern uint8_t alarm_dtu_trig ;
extern uint8_t alarm_dwin_trig ;

// 配置参数
#define SENSOR_COUNT 4
#define BUFFER_SIZE 50
#define FIXED_POINT_SCALE 1000
#define DT_FIXED 5

// 预热阶段配置
#define WARMUP_SAMPLES 200
#define CONFIG_FILE_PATH "wire_rope_config.bin"

// 数值稳定性配置
#define MIN_VARIANCE_THRESHOLD 1
#define MAX_VARIANCE_THRESHOLD 1000000


// 魔数和版本号
#define CONFIG_MAGIC_NUMBER 0x57524F50  // "WROP"

// 定义int32的范围限制
// #define INT32_MAX 2147483647
// #define INT32_MIN (-2147483648)

// 调整阈值，使其更适合实际数据
//#define MEAN_DEVIATION_THRESHOLD    50.0f    // 从2.5调整到30.0
//#define SENSOR_DEVIATION_THRESHOLD  20.0f    // 从4.0调整到20.0
//#define VARIANCE_THRESHOLD          10.0f    // 从4.2调整到10.0
//#define TREND_THRESHOLD             30.0f    // 从7.5调整到30.0
//#define DEFECT_SCORE_THRESHOLD      5        // 保持不变

// 卡尔曼滤波器结构体（针对STM32F103优化）
typedef struct
{
    float q;        // 过程噪声协方差（系统模型不确定性）
    float r;        // 测量噪声协方差（传感器精度）
    float p;        // 估计误差协方差（状态置信度）
    float x;        // 最优估计值（滤波输出）
    float k;        // 卡尔曼增益（动态权重）
} KalmanFilter;

// 检测结果结构
typedef struct
{
    float filtered_value;     // 滤波值（浮点数）
    float uncertainty;       // 不确定性（浮点数）
    uint8_t has_defect;      // 是否有缺陷
    uint8_t defect_type;     // 缺陷类型
    float severity;          // 严重程度（浮点数）
} Detection_Result;

// 配置参数结构
typedef struct
{
    float baseline_mean[SENSOR_COUNT];      // 改为float
    float baseline_std[SENSOR_COUNT];        // 改为float
    float adaptive_threshold[SENSOR_COUNT];  // 改为float
    float process_noise_scale;              // 改为float
    float measurement_noise_scale;           // 改为float
    uint32_t checksum;
} ConfigParams;

// 单片机优化：紧凑的钢丝绳检测器主结构
typedef struct
{
    KalmanFilter kalman_filters[SENSOR_COUNT];
    uint16_t sensor_buffer[BUFFER_SIZE][SENSOR_COUNT];
    uint32_t buffer_index;
    uint32_t defect_count;
    uint8_t system_mode;              // 改为uint8_t
    ConfigParams config_params;
    uint8_t warmup_complete;
    uint32_t warmup_sample_count;

    // 趋势分析相关（减少缓冲区大小）
    float recent_filtered_values[8];  // 改为float
    uint32_t recent_values_count;
    uint32_t recent_values_index;

    uint32_t total_samples;
} WireRopeDetector;

void median_filter(uint16_t *in, uint16_t *out, int len);
uint8_t process_kalman(uint16_t data[4]);

// 配置参数管理函数
int warmup_save_config(void);
int warmup_load_config(void);
void detection_init(uint16_t measurement[SENSOR_COUNT]);
int warmup_validate_config(void);
void warmup_init_default_config(void);
void kalman_system_init(void);

#endif /*__APP_KALMAN_H__ */

