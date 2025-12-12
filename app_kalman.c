#include "app_config.h"
#include "app_kalman.h"
#include "eeprom_flash.h"

/*
 * 钢丝绳缺陷检测系统 -
 * 简化版卡尔曼滤波器实现
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// 函数声明
extern double fabs(double x);
extern double sqrt(double x);
extern void *memset(void* s, int c, size_t n);
extern int printf(const char *format, ...);
extern uint32_t EEPROM_FLASH_ReadU32(uint16_t Address);
extern int EEPROM_FLASH_WriteU32(uint16_t Address, uint32_t Data);



// 确保包含isfinite函数
#ifndef isfinite
    #define isfinite(x) ((x) == (x) && (x) * 0.0 == 0.0)
#endif

float MEAN_DEVIATION_THRESHOLD = 50.0f;
float SENSOR_DEVIATION_THRESHOLD = 20.0f;
float VARIANCE_THRESHOLD = 10.0f;
float TREND_THRESHOLD = 30.0f;
float DEFECT_SCORE_THRESHOLD = 5;
float current_mean_deviation = 0;
float filtered_value11;
extern uint8_t final_alarm_level ;
uint8_t warmup_init_flag = 0;
uint8_t detection_init_flag = 0;
uint8_t defect_score;
extern uint8_t defect_score_disp;
uint8_t mode_switch ;
uint8_t warmup_save_done = 0;
float variance_fixed = 0.0f;
float baseline_mean_avg = 0;
float trend_deviation = 0.0f;
float max_sensor_deviation = 0.0f;
float variance = 0.0f;
float filtered_sum = 0;
float filtered_measurement[SENSOR_COUNT];
float filtered_avg = 0;
// 计算噪声参数
float avg_std = 0.0f;           // 改为float
uint8_t valid_sensors = 0;      // 保持不变（范围足够）

// 全局检测器实例
WireRopeDetector g_detector;

//卡尔曼滤波器初始化
void kalman_filter_init(KalmanFilter* kf, float q, float r, float init_x)
{
    kf->q = q;
    kf->r = r;
    kf->p = 1.0f;      // 初始协方差（不可为0）
    kf->x = init_x;    // 初始状态估计
    kf->k = 0.0f;
}

//定点数卡尔曼滤波器更新
static  float kalman_filter_update(KalmanFilter* kf, uint16_t measurement)
{
    // 预测步骤（浮点数运算）
    float x_pred = kf->x;
    float p_pred = kf->p + kf->q;
    // 更新步骤（浮点数运算）
    float denominator = p_pred + kf->r;
    float K = (denominator != 0) ? (p_pred / denominator) : 0;


    float innovation = (float)measurement - x_pred;
    float kalman_gain_innovation = K * innovation;

    kf->x = x_pred + kalman_gain_innovation;

    float identity_minus_k = 1.0f - K;
    kf->p = identity_minus_k * p_pred;

    return kf->x;
}

// 方差计算
static  float safe_variance_calculation(int64_t sum_val, int64_t sum_sq, uint32_t count)
{
    if (count == 0) return 0.0f;
    float mean = (sum_val / (float)count);
    float mean_sq = (mean * mean);
    float variance = ((sum_sq / (float)count) - mean_sq);
    return (variance < 0) ? 0.0f : variance;
}

// 校验和计算
static  uint32_t calculate_checksum(const ConfigParams* config)
{
    uint32_t checksum = 0;
    const uint8_t *data = (const uint8_t*)config;
    size_t size = sizeof(ConfigParams) - sizeof(uint32_t);

    for (size_t i = 0; i < size; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

// 使用Flash存储配置参数
static void save_config_to_flash(void)
{
    // 直接存储魔数
    EEPROM_FLASH_WriteU32(FLASH_KALMAN_CONFIG_MAGIC_NUMBER, CONFIG_MAGIC_NUMBER);

    // 使用联合体进行类型转换
    union
    {
        float f;
        uint32_t u;
    } converter;

    // 定义地址数组
    uint16_t baseline_mean_addrs[] =
    {
        FLASH_KALMAN_CONFIG_BASELINE_MEAN_0,
        FLASH_KALMAN_CONFIG_BASELINE_MEAN_1,
        FLASH_KALMAN_CONFIG_BASELINE_MEAN_2,
        FLASH_KALMAN_CONFIG_BASELINE_MEAN_3
    };

    uint16_t baseline_std_addrs[] =
    {
        FLASH_KALMAN_CONFIG_BASELINE_STD_0,
        FLASH_KALMAN_CONFIG_BASELINE_STD_1,
        FLASH_KALMAN_CONFIG_BASELINE_STD_2,
        FLASH_KALMAN_CONFIG_BASELINE_STD_3
    };

    uint16_t adaptive_threshold_addrs[] =
    {
        FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_0,
        FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_1,
        FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_2,
        FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_3
    };

    // 存储 baseline_mean
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        converter.f = g_detector.config_params.baseline_mean[i];
        EEPROM_FLASH_WriteU32(baseline_mean_addrs[i], converter.u);
    }

    // 存储 baseline_std
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        converter.f = g_detector.config_params.baseline_std[i];
        EEPROM_FLASH_WriteU32(baseline_std_addrs[i], converter.u);
    }

    // 存储 adaptive_threshold
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        converter.f = g_detector.config_params.adaptive_threshold[i];
        EEPROM_FLASH_WriteU32(adaptive_threshold_addrs[i], converter.u);
    }

    // 计算校验和
    g_detector.config_params.checksum = calculate_checksum(&g_detector.config_params);
    EEPROM_FLASH_WriteU32(FLASH_KALMAN_CONFIG_CHECKSUM, g_detector.config_params.checksum);
}

static int load_config_from_flash(void)
{
    // 从Flash读取配置参数
    uint32_t magic_number = EEPROM_FLASH_ReadU32(FLASH_KALMAN_CONFIG_MAGIC_NUMBER);

    // 使用联合体进行类型转换
    union
    {
        float f;
        uint32_t u;
    } converter;

    // 定义地址数组
    uint16_t baseline_mean_addrs[] =
    {
        FLASH_KALMAN_CONFIG_BASELINE_MEAN_0,
        FLASH_KALMAN_CONFIG_BASELINE_MEAN_1,
        FLASH_KALMAN_CONFIG_BASELINE_MEAN_2,
        FLASH_KALMAN_CONFIG_BASELINE_MEAN_3
    };

    uint16_t baseline_std_addrs[] =
    {
        FLASH_KALMAN_CONFIG_BASELINE_STD_0,
        FLASH_KALMAN_CONFIG_BASELINE_STD_1,
        FLASH_KALMAN_CONFIG_BASELINE_STD_2,
        FLASH_KALMAN_CONFIG_BASELINE_STD_3
    };

    uint16_t adaptive_threshold_addrs[] =
    {
        FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_0,
        FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_1,
        FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_2,
        FLASH_KALMAN_CONFIG_ADAPTIVE_THRESHOLD_3
    };

    // 读取 baseline_mean
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        converter.u = EEPROM_FLASH_ReadU32(baseline_mean_addrs[i]);
        g_detector.config_params.baseline_mean[i] = converter.f;
    }

    // 读取 baseline_std
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        converter.u = EEPROM_FLASH_ReadU32(baseline_std_addrs[i]);
        g_detector.config_params.baseline_std[i] = converter.f;
    }

    // 读取 adaptive_threshold
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        converter.u = EEPROM_FLASH_ReadU32(adaptive_threshold_addrs[i]);
        g_detector.config_params.adaptive_threshold[i] = converter.f;
    }

    g_detector.config_params.checksum = EEPROM_FLASH_ReadU32(FLASH_KALMAN_CONFIG_CHECKSUM);//校验和

    // 验证魔数和版本
    if (magic_number != CONFIG_MAGIC_NUMBER)
    {
        LOG("magic_number != CONFIG_MAGIC_NUMBER\n");
        return -1;
    }

    // 验证校验和
    uint32_t calculated_checksum = calculate_checksum(&g_detector.config_params);
    if (calculated_checksum != g_detector.config_params.checksum)
    {
        LOG("calculated_checksum != g_detector.config_params.checksum\n");
        return -1;
    }
    return 0;
}
static inline Detection_Result detect_defects_improved(const uint16_t measurement[SENSOR_COUNT], float filtered_value)
{
    Detection_Result result_ddi = {0};
    // 重置全局变量
    variance_fixed = 0.0f;

    // 计算各种特征1（浮点数运算）
    float mean_fixed = 0.0f;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        mean_fixed += (float)measurement[i];
    }
    mean_fixed = mean_fixed / SENSOR_COUNT;

    // 计算各种特征2计算方差
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        float diff = (float)measurement[i] - mean_fixed;
        float diff_sq = diff * diff;
        variance_fixed += diff_sq;
    }
    variance_fixed = variance_fixed / SENSOR_COUNT;


    // 计算与基线的偏差（浮点数运算）
    baseline_mean_avg = 0.0f;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        baseline_mean_avg += g_detector.config_params.baseline_mean[i];
    }
    baseline_mean_avg = baseline_mean_avg / SENSOR_COUNT;
    // 计算当前测量值与基线的偏差（不使用放大）
    current_mean_deviation = fabsf(mean_fixed - baseline_mean_avg);

    // 计算单个传感器的最大偏差
    max_sensor_deviation = 0.0f;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        float sensor_deviation = fabsf((float)measurement[i] - g_detector.config_params.baseline_mean[i]);
        if (sensor_deviation > max_sensor_deviation)
        {
            max_sensor_deviation = sensor_deviation;
        }
    }

//  计算滤波值的趋势（浮点数运算）
//    # 计算滤波值的趋势（如果有历史数据）
    if (g_detector.recent_values_count >= 5)
    {
        float recent_avg = 0.0f;
        int count = 0;
        for (int i = 0; i < 5 && i < g_detector.recent_values_count; i++)
        {
            int index = (g_detector.recent_values_index - 1 - i + 8) % 8;
            recent_avg += g_detector.recent_filtered_values[index];
            count++;
        }
        if (count > 0)
        {
            recent_avg = recent_avg / count;
            trend_deviation = fabsf(filtered_value - recent_avg);
        }
    }

    // 多策略检测（浮点数比较）
    defect_score = 0;

    // 策略1：均值偏差检测
    if (current_mean_deviation > MEAN_DEVIATION_THRESHOLD)
    {
        defect_score += 3;
    }

    // 策略2：单个传感器异常检测
    if (max_sensor_deviation > SENSOR_DEVIATION_THRESHOLD)
    {
        defect_score += 2;
    }

    // 策略3：方差异常检测
    if (variance_fixed > VARIANCE_THRESHOLD)
    {
        defect_score += 1;
    }

    // 策略4：趋势异常检测
    if (trend_deviation > TREND_THRESHOLD)
    {
        defect_score += 1;
    }

    // 综合判断
    if (defect_score >= DEFECT_SCORE_THRESHOLD)
    {
        result_ddi.has_defect = 1;
        result_ddi.defect_type = 1;
        result_ddi.severity = defect_score;
        if (result_ddi.severity > 5.0f)
        {
            result_ddi.severity = 5.0f;
        }

        if (max_sensor_deviation > SENSOR_DEVIATION_THRESHOLD)
        {
            result_ddi.defect_type = 2;
        }
    }

    result_ddi.uncertainty = variance_fixed;

    // 添加调试打印
    static int debug_count = 0;
    if (debug_count++ % 100 == 0)
    {
//        snprintf(uart4_buf, sizeof(uart4_buf),
//                 "\n DEBUG: mean_fixed=%.1f, baseline_mean_avg=%.1f\n current_mean_deviation=   %.1f,  max_sensor_deviation=       %.1f,  variance_fixed=     %.1f,  trend_deviation= %.1f,            defect_score= %d,        filtered_value=%.1f\n",
//                 mean_fixed, baseline_mean_avg, current_mean_deviation, max_sensor_deviation,
//                 variance_fixed, trend_deviation, defect_score, filtered_value);
//        BSP_UART_Transmit(BSP_UART4, (uint8_t*)uart4_buf, strlen(uart4_buf));
//        snprintf(uart4_buf, sizeof(uart4_buf),
//                 " MEAN_DEVIATION_THRESHOLD= %.1f, SENSOR_DEVIATION_THRESHOLD= %.1f, VARIANCE_THRESHOLD= %.1f, TREND_THRESHOLD= %.1f, DEFECT_SCORE_THRESHOLD= %.1f\n",
//                 MEAN_DEVIATION_THRESHOLD, SENSOR_DEVIATION_THRESHOLD, VARIANCE_THRESHOLD, TREND_THRESHOLD, DEFECT_SCORE_THRESHOLD);
//        BSP_UART_Transmit(BSP_UART4, (uint8_t*)uart4_buf, strlen(uart4_buf));
//        snprintf(uart4_buf, sizeof(uart4_buf),
//                 "\nresult.has_defect=%d, result.defect_type=%d, result.severity=%.1f,result.uncertainty=%.1f,result.filtered_value=%.1f   \n", result.has_defect, result.defect_type, result.severity, result.uncertainty, result.filtered_value);
//        BSP_UART_Transmit(BSP_UART4, (uint8_t*)uart4_buf, strlen(uart4_buf));
    }
    filtered_value11  = filtered_value;
    return result_ddi;
}


// ==================== 接口1：预热配置接口 ====================

void warmup_init(void)
{
    mode_switch = 0;
    g_detector.buffer_index = 0;
    // 初始化预热数据
    g_detector.warmup_complete = 0;
    g_detector.warmup_sample_count = 0;

    // 初始化配置参数
    memset(g_detector.config_params.baseline_mean, 0, sizeof(g_detector.config_params.baseline_mean));
    memset(g_detector.config_params.baseline_std, 0, sizeof(g_detector.config_params.baseline_std));
    memset(g_detector.config_params.adaptive_threshold, 0, sizeof(g_detector.config_params.adaptive_threshold));
    g_detector.config_params.checksum = 0;


}

void warmup_process_data(const uint16_t measurement[SENSOR_COUNT])
{
    g_detector.warmup_sample_count++;
    // 存储传感器数据
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        g_detector.sensor_buffer[g_detector.buffer_index][i] = measurement[i];
    }
    g_detector.buffer_index = (g_detector.buffer_index + 1) % BUFFER_SIZE;

    // 检查是否收集足够样本
    if (g_detector.warmup_sample_count >= WARMUP_SAMPLES)
    {
        // 计算每个传感器的均值和标准差
        uint16_t data_points[BUFFER_SIZE][SENSOR_COUNT];

        memcpy(data_points, g_detector.sensor_buffer, sizeof(g_detector.sensor_buffer));
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            //计算均值和平方差
            float mean_val = 0.0f;//均值
            float variance = 0.0f;//方差
            float std_val = 0.0f;//标准差
            for (int j = 0; j < BUFFER_SIZE; j++)
            {
                mean_val += (float)data_points[j][i];
                variance += pow((float)(data_points[j][i] - mean_val), 2);
            }
            mean_val = mean_val / BUFFER_SIZE;
            variance = variance / BUFFER_SIZE;
            // 数值保护：避免负值、NaN/Inf，并使用浮点开方
            {
                float v = variance;
                if (v < 0.0f) v = 0.0f;
                float s = sqrtf(v);
                if (!isfinite(s)) s = 0.0f;
                std_val = s;
            }
            g_detector.config_params.baseline_mean[i] = mean_val;//保存均值
            g_detector.config_params.baseline_std[i] = std_val;//保存标准差
            if (g_detector.config_params.baseline_std[i] < MIN_VARIANCE_THRESHOLD)
            {
                g_detector.config_params.baseline_std[i] = MIN_VARIANCE_THRESHOLD;
            }
            g_detector.config_params.adaptive_threshold[i] = std_val * 3.0f;//保存自适应阈值
        }
        g_detector.warmup_complete = 1;//预热完成
    }
}
// ==================== 接口2：正常检测接口 ====================
void detection_init(uint16_t measurement[SENSOR_COUNT])
{
    int load_result = load_config_from_flash();
    if (load_result != 0)
    {
        LOG("Config load failed. Resetting to warmup mode.\n");
        mode_switch = 0;
        warmup_init_flag = 0;
        warmup_save_done = 0;
        detection_init_flag = 0;
        return;
    }
    // 初始化卡尔曼滤波器
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        kalman_filter_init(&g_detector.kalman_filters[i], 0.01f, 0.1f, (float)2048);//初始化卡尔曼滤波器，2048为默认值
    }

}

uint8_t detection_process_data(uint16_t measurements[SENSOR_COUNT])
{
    uint8_t defect_result = 0;//0 无缺陷，1轻度缺陷，2重度缺陷


    g_detector.total_samples++;

    // 存储传感器数据
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        g_detector.sensor_buffer[g_detector.buffer_index][i] = measurements[i];
    }
    g_detector.buffer_index = (g_detector.buffer_index + 1) % BUFFER_SIZE;

    // 卡尔曼滤波（浮点数运算）
    //得到4个传感器的滤波值
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        // 调试：打印滤波器状态
//        LOG("filter[%d]: x=%.6f, p=%.6f, q=%.6f, r=%.6f, input=%d\n",
//            i, g_detector.kalman_filters[i].x, g_detector.kalman_filters[i].p,g_detector.kalman_filters[i].q, g_detector.kalman_filters[i].r, measurements[i]);
        filtered_measurement[i] = kalman_filter_update(&g_detector.kalman_filters[i], measurements[i]);
    }
//       LOG("0:%.0f,1:%.0f,2:%.0f,3:%.0f\n", filtered_measurement[0], filtered_measurement[1], filtered_measurement[2], filtered_measurement[3]);
//       snprintf(uart4_buf, sizeof(uart4_buf), "0:%.0f,1:%.0f,2:%.0f,3:%.0f\n", filtered_measurement[0], filtered_measurement[1], filtered_measurement[2], filtered_measurement[3]);
//       BSP_UART_Transmit(BSP_UART4, (uint8_t*)uart4_buf, strlen(uart4_buf));

    //计算4个传感器的平均滤波值
    filtered_sum = 0.0f;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        filtered_sum += filtered_measurement[i];
    }
    filtered_avg = filtered_sum / SENSOR_COUNT;

    // 存储最近的滤波值用于趋势分析
    g_detector.recent_filtered_values[g_detector.recent_values_index] = filtered_avg;
    g_detector.recent_values_index = (g_detector.recent_values_index + 1) % 8;
    if (g_detector.recent_values_count < 8)
    {
        g_detector.recent_values_count++;
    }

    // 缺陷检测
    Detection_Result result_defect = detect_defects_improved(measurements, filtered_avg);

    if (result_defect.has_defect)
    {
        g_detector.defect_count++;
        LOG("defect_count=%d \n", g_detector.defect_count);
        //uncertainty 不确定性
        if (result_defect.uncertainty < 500)
        {
            defect_result = 1;
        }
        else
        {
            defect_result = 2;
        }
    }

    return defect_result;//0 无缺陷，1轻度缺陷，2重度缺陷
}


int get_system_mode(void)
{
    return mode_switch;
}
uint8_t process_kalman(uint16_t data1[4])
{
    uint8_t defect_result;
    // ========== 预热模式 ==========
    switch (mode_switch)
    {
    case 0:
        //预热模式初始化
        if (warmup_init_flag == 0)
        {
            warmup_init_flag = 1;
            warmup_init();
            LOG("Warmup mode initialization completed.\n");
        }
        // 处理预热数据
        warmup_process_data(data1);
        // 预热完成-保存数据并切换到检测状态
        if (g_detector.warmup_complete && !warmup_save_done)
        {
            warmup_save_done = 1;
            save_config_to_flash();
            // ===== 自动切换到检测模式 =====
            mode_switch = 1;
            //EEPROM_FLASH_WriteU16(FLASH_MODE_SWITCH, mode_switch);
            LOG("Auto-switched to DETECTION mode. System ready!\n");

        }
        defect_result = 0;
        break;
    case 1:
        if (!detection_init_flag)
        {
            detection_init_flag = 1;
            detection_init(data1);
            LOG("Detection initialization completed.\n");
        }
        // 处理检测数据，返回检测结果 0 无缺陷，1轻度缺陷，2重度缺陷
        defect_result = detection_process_data(data1);

        break;
    default:
        break;
    }
    return defect_result;
}
