#include "app_config.h"
#include "app_kalman.h"
#include "eeprom_flash.h"

/*
 * 钢丝绳缺陷检测系统 -
 * 简化版卡尔曼滤波器实现
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// 函数声明（来自其他模块）
extern double fabs(double x);
extern double sqrt(double x);
extern void *memset(void* s, int c, size_t n);
extern int printf(const char *format, ...);
extern uint32_t EEPROM_FLASH_ReadU32(uint16_t Address);
extern int EEPROM_FLASH_WriteU32(uint16_t Address, uint32_t Data);

extern uint32_t HAL_GetTick(void); // 获取毫秒时钟

// 确保包含isfinite函数
#ifndef isfinite
    #define isfinite(x) ((x) == (x) && (x) * 0.0f == 0.0f)
#endif

float MEAN_DEVIATION_THRESHOLD = 50.0f;
float SENSOR_DEVIATION_THRESHOLD = 20.0f;
float VARIANCE_THRESHOLD = 10.0f;
float TREND_THRESHOLD = 30.0f;
float DEFECT_SCORE_THRESHOLD = 5.0f;
float current_mean_deviation = 0.0f;
float filtered_value11;
extern uint8_t final_alarm_level;
uint8_t warmup_init_flag = 0;
uint8_t detection_init_flag = 0;
uint8_t defect_score;
extern uint8_t defect_score_disp;
uint8_t mode_switch;
uint8_t warmup_save_done = 0;
float variance_fixed = 0.0f;
float baseline_mean_avg = 0.0f;
float trend_deviation = 0.0f;
float max_sensor_deviation = 0.0f;
float variance = 0.0f;
float filtered_sum = 0.0f;
float filtered_measurement[SENSOR_COUNT];
float filtered_avg = 0.0f;
// 计算噪声参数
float avg_std = 0.0f;
uint8_t valid_sensors = 0;

// 全局检测器实例
WireRopeDetector g_detector;

// ====== 卡尔曼滤波器 ======
void kalman_filter_init(KalmanFilter* kf, float q, float r, float init_x)
{
    kf->q = q;
    kf->r = r;
    kf->p = 1.0f;      // 初始协方差（不可为0）
    kf->x = init_x;    // 初始状态估计
    kf->k = 0.0f;
}

static float kalman_filter_update(KalmanFilter* kf, uint16_t measurement)
{
    // 预测
    float x_pred = kf->x;
    float p_pred = kf->p + kf->q;
    // 更新
    float denominator = p_pred + kf->r;
    float K = (denominator != 0.0f) ? (p_pred / denominator) : 0.0f;

    float innovation = (float)measurement - x_pred;
    float kalman_gain_innovation = K * innovation;

    kf->x = x_pred + kalman_gain_innovation;

    float identity_minus_k = 1.0f - K;
    kf->p = identity_minus_k * p_pred;

    return kf->x;
}

// 方差计算（未使用：保留）
static float safe_variance_calculation(int64_t sum_val, int64_t sum_sq, uint32_t count)
{
    if (count == 0U) return 0.0f;
    float mean = (sum_val / (float)count);
    float mean_sq = (mean * mean);
    float variance = ((sum_sq / (float)count) - mean_sq);
    return (variance < 0.0f) ? 0.0f : variance;
}

// 校验和计算
static uint32_t calculate_checksum(const ConfigParams* config)
{
    uint32_t checksum = 0U;
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
    union { float f; uint32_t u; } converter;

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

    // 存储 baseline_mean/std/threshold
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        converter.f = g_detector.config_params.baseline_mean[i];
        EEPROM_FLASH_WriteU32(baseline_mean_addrs[i], converter.u);
    }
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        converter.f = g_detector.config_params.baseline_std[i];
        EEPROM_FLASH_WriteU32(baseline_std_addrs[i], converter.u);
    }
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
    union { float f; uint32_t u; } converter;

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

    // 验证魔数和校验和
    if (magic_number != CONFIG_MAGIC_NUMBER)
    {
        LOG("magic_number != CONFIG_MAGIC_NUMBER\n");
        return -1;
    }
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
    Detection_Result result_ddi = (Detection_Result){0};
    // 重置全局变量
    variance_fixed = 0.0f;

    // 计算各种特征1：均值
    float mean_fixed = 0.0f;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        mean_fixed += (float)measurement[i];
    }
    mean_fixed = mean_fixed / (float)SENSOR_COUNT;

    // 计算各种特征2：方差
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        float diff = (float)measurement[i] - mean_fixed;
        variance_fixed += diff * diff;
    }
    variance_fixed = variance_fixed / (float)SENSOR_COUNT;

    // 与基线的偏差
    baseline_mean_avg = 0.0f;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        baseline_mean_avg += g_detector.config_params.baseline_mean[i];
    }
    baseline_mean_avg = baseline_mean_avg / (float)SENSOR_COUNT;
    current_mean_deviation = fabsf(mean_fixed - baseline_mean_avg);

    // 单通道最大偏差
    max_sensor_deviation = 0.0f;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        float sensor_deviation = fabsf((float)measurement[i] - g_detector.config_params.baseline_mean[i]);
        if (sensor_deviation > max_sensor_deviation)
        {
            max_sensor_deviation = sensor_deviation;
        }
    }

    // 趋势（使用最近历史）
    if (g_detector.recent_values_count >= 5U)
    {
        float recent_avg = 0.0f;
        int count = 0;
        for (int i = 0; i < 5 && i < (int)g_detector.recent_values_count; i++)
        {
            int index = (int)((g_detector.recent_values_index - 1 + 8U - (uint32_t)i) % 8U);
            recent_avg += g_detector.recent_filtered_values[index];
            count++;
        }
        if (count > 0)
        {
            recent_avg = recent_avg / (float)count;
            trend_deviation = fabsf(filtered_value - recent_avg);
        }
    }

    // 多策略检测
    defect_score = 0;
    if (current_mean_deviation > MEAN_DEVIATION_THRESHOLD) { defect_score += 3; }
    if (max_sensor_deviation > SENSOR_DEVIATION_THRESHOLD) { defect_score += 2; }
    if (variance_fixed > VARIANCE_THRESHOLD)                { defect_score += 1; }
    if (trend_deviation > TREND_THRESHOLD)                  { defect_score += 1; }

    if (defect_score >= (int)DEFECT_SCORE_THRESHOLD)
    {
        result_ddi.has_defect  = 1;
        result_ddi.defect_type = (max_sensor_deviation > SENSOR_DEVIATION_THRESHOLD) ? 2 : 1;
        result_ddi.severity    = (defect_score > 5) ? 5.0f : (float)defect_score;
    }

    result_ddi.uncertainty = variance_fixed;
    filtered_value11  = filtered_value;
    return result_ddi;
}

// ============= 预热辅助：递推统计更新 & 最终落盘准备 =============
static inline void warmup_running_update(const uint16_t measurement[SENSOR_COUNT])
{
    // 每通道使用Welford递推
    for (int i = 0; i < SENSOR_COUNT; ++i)
    {
        float x = (float)measurement[i];
        uint32_t n_old = g_detector.warmup_running_n[i];
        uint32_t n_new = n_old + 1U;
        float mean_old = g_detector.warmup_running_mean[i];
        float delta  = x - mean_old;
        float mean_new = mean_old + delta / (float)n_new;
        float delta2 = x - mean_new;
        float M2_new = g_detector.warmup_running_M2[i] + delta * delta2;

        g_detector.warmup_running_n[i]   = n_new;
        g_detector.warmup_running_mean[i]= mean_new;
        g_detector.warmup_running_M2[i]  = M2_new;

        // 同步更新当前基线（预热期间可用于显示/调试）
        float variance = (n_new > 0U) ? (M2_new / (float)n_new) : 0.0f;
        if (variance < 0.0f) variance = 0.0f;
        float std_val = sqrtf(variance);
        if (!isfinite(std_val)) std_val = 0.0f;
        if (std_val < (float)MIN_VARIANCE_THRESHOLD) std_val = (float)MIN_VARIANCE_THRESHOLD;

        g_detector.config_params.baseline_mean[i]      = mean_new;
        g_detector.config_params.baseline_std[i]       = std_val;
        g_detector.config_params.adaptive_threshold[i] = std_val * 3.0f;
    }
}

static inline void warmup_finalize_and_prepare_config(void)
{
    // 用当前整轮预热的递推统计，最终定稿一次
    for (int i = 0; i < SENSOR_COUNT; ++i)
    {
        uint32_t n = g_detector.warmup_running_n[i];
        float mean = (n > 0U) ? g_detector.warmup_running_mean[i] : 0.0f;
        float variance = (n > 0U) ? (g_detector.warmup_running_M2[i] / (float)n) : 0.0f;
        if (variance < 0.0f) variance = 0.0f;
        float std_val = sqrtf(variance);
        if (!isfinite(std_val)) std_val = 0.0f;
        if (std_val < (float)MIN_VARIANCE_THRESHOLD) std_val = (float)MIN_VARIANCE_THRESHOLD;

        g_detector.config_params.baseline_mean[i]      = mean;
        g_detector.config_params.baseline_std[i]       = std_val;
        g_detector.config_params.adaptive_threshold[i] = std_val * 3.0f;
    }
}

// ==================== 接口1：预热配置接口 ====================
void warmup_init(void)
{
    mode_switch = 0;
    g_detector.buffer_index = 0;

    // 初始化预热状态
    g_detector.warmup_complete = 0;
    g_detector.warmup_sample_count = 0;

    // 递推统计清零
    for (int i = 0; i < SENSOR_COUNT; ++i)
    {
        g_detector.warmup_running_mean[i] = 0.0f;
        g_detector.warmup_running_M2[i]   = 0.0f;
        g_detector.warmup_running_n[i]    = 0U;
    }

    // 清空配置参数缓存
    memset(g_detector.config_params.baseline_mean, 0, sizeof(g_detector.config_params.baseline_mean));
    memset(g_detector.config_params.baseline_std, 0, sizeof(g_detector.config_params.baseline_std));
    memset(g_detector.config_params.adaptive_threshold, 0, sizeof(g_detector.config_params.adaptive_threshold));
    g_detector.config_params.checksum = 0U;

    // 最近趋势缓存清零
    g_detector.recent_values_count = 0U;
    g_detector.recent_values_index = 0U;
    for (int i = 0; i < 8; ++i) g_detector.recent_filtered_values[i] = 0.0f;

    // 标志位
    warmup_save_done = 0;
    detection_init_flag = 0;

    // 记录预热起始时间/超时阈值（总预热超时）
    g_detector.warmup_start_time_ms = HAL_GetTick();
    g_detector.warmup_timeout_ms    = WARMUP_TIMEOUT_MS;

    LOG("Warmup initialized. timeout=%u ms\n", (unsigned)g_detector.warmup_timeout_ms);
}

// 新增：统一的“总预热超时计时器重置”实现
void warmup_reset_timeout_counter(void)
{
    g_detector.warmup_start_time_ms = HAL_GetTick();
    LOG("Warmup timeout counter reset.\n");
}

void warmup_process_data(const uint16_t measurement[SENSOR_COUNT])
{
    g_detector.warmup_sample_count++;

    // 存储传感器数据（环形缓冲）
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        g_detector.sensor_buffer[g_detector.buffer_index][i] = measurement[i];
    }
    g_detector.buffer_index = (g_detector.buffer_index + 1U) % BUFFER_SIZE;

    // 累计递推统计（每点更新）
    warmup_running_update(measurement);

    // 不在这里设置 warmup_complete，是否切换由主流程判定
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
        kalman_filter_init(&g_detector.kalman_filters[i], 0.01f, 0.1f, (float)2048); // 2048为默认值
    }
}

uint8_t detection_process_data(uint16_t measurements[SENSOR_COUNT])
{
    uint8_t defect_result = 0; // 0 无缺陷，1轻度缺陷，2重度缺陷

    g_detector.total_samples++;

    // 存储传感器数据
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        g_detector.sensor_buffer[g_detector.buffer_index][i] = measurements[i];
    }
    g_detector.buffer_index = (g_detector.buffer_index + 1U) % BUFFER_SIZE;

    // 卡尔曼滤波
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        filtered_measurement[i] = kalman_filter_update(&g_detector.kalman_filters[i], measurements[i]);
    }

    // 平均滤波值
    filtered_sum = 0.0f;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        filtered_sum += filtered_measurement[i];
    }
    filtered_avg = filtered_sum / (float)SENSOR_COUNT;

    // 最近趋势
    g_detector.recent_filtered_values[g_detector.recent_values_index] = filtered_avg;
    g_detector.recent_values_index = (g_detector.recent_values_index + 1U) % 8U;
    if (g_detector.recent_values_count < 8U) { g_detector.recent_values_count++; }

    // 缺陷检测
    Detection_Result result_defect = detect_defects_improved(measurements, filtered_avg);

    if (result_defect.has_defect)
    {
        g_detector.defect_count++;
        LOG("defect_count=%lu \n", (unsigned long)g_detector.defect_count);
        // uncertainty 不确定性
        defect_result = (result_defect.uncertainty < 500.0f) ? 1 : 2;
    }

    return defect_result; // 0 无缺陷，1轻度缺陷，2重度缺陷
}

int get_system_mode(void)
{
    return mode_switch;
}


// 检查是否可以切换到检测模式（按键触发等）
void handle_mode_switch(uint16_t data1[4]) {
    // 改动点：
    // 1) 不再因样本不足200而回退到预热模式
    // 2) 样本>=200：保存并切换；样本<200：不保存，直接进入检测，沿用上次参数
    if (mode_switch == 1 && !detection_init_flag) {
        if (g_detector.warmup_sample_count >= WARMUP_SAMPLES) {
            if (!warmup_save_done) {
                warmup_finalize_and_prepare_config();
                save_config_to_flash();
                warmup_save_done = 1;
            }
            g_detector.warmup_complete = 1;
            LOG("Switched to DETECTION by BUTTON. Samples=%lu\n",
                (unsigned long)g_detector.warmup_sample_count);
        } else {
            // 不足200点：不保存，不覆盖；直接进入检测模式，使用上次的参数
            g_detector.warmup_complete = 1;
            LOG("Force switch to DETECTION (samples=%lu < %d). Keep previous parameters.\n",
                (unsigned long)g_detector.warmup_sample_count, WARMUP_SAMPLES);
        }
        // 注意：真正的检测初始化在 process_detection_mode 中进行（detection_init_flag 将置1）
    }
}

// 预热模式的详细流程
void process_warmup_mode(uint16_t data1[4])
{
    if (warmup_init_flag == 0) 
		{
        warmup_init_flag = 1;
        warmup_init();
        LOG("Warmup mode initialization completed.\n");
    }

    warmup_process_data(data1);

    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - g_detector.warmup_start_time_ms;
    if (elapsed_ms >= g_detector.warmup_timeout_ms) 
    {
        if (g_detector.warmup_sample_count >= WARMUP_SAMPLES)
        {
            if (!warmup_save_done)
            {
                warmup_finalize_and_prepare_config();
                save_config_to_flash();
                warmup_save_done = 1;
            }
            mode_switch = 1; // 先切换
            EEPROM_FLASH_WriteU16(FLASH_MODE_SWITCH, mode_switch); // 再写入
            g_detector.warmup_complete = 1;
            LOG("Auto-switched to DETECTION (timeout=%lu ms). Samples=%lu\n",
                (unsigned long)elapsed_ms, (unsigned long)g_detector.warmup_sample_count);
        }
				else 
        {
            // 关键改动：即使<200点，也切换（但不保存），只保留原参数
            mode_switch = 1;
            EEPROM_FLASH_WriteU16(FLASH_MODE_SWITCH, mode_switch);
            g_detector.warmup_complete = 1;
            LOG("Force auto-switch to DETECTION due to timeout (samples=%lu < %d). Keep previous parameters.\n",
                (unsigned long)g_detector.warmup_sample_count, WARMUP_SAMPLES);
        }
    }
}

// 检测模式的详细流程
uint8_t process_detection_mode(uint16_t data1[4]) {
    if (!detection_init_flag) {
        detection_init_flag = 1;
        detection_init(data1);
        LOG("Detection initialization completed.\n");
    }
    return detection_process_data(data1);
}

// 主流程
uint8_t process_kalman(uint16_t data1[4]) {
    uint8_t defect_result = 0;

    handle_mode_switch(data1);

    switch (mode_switch) {
        case 0:
            process_warmup_mode(data1);
            defect_result = 0;
            break;
        case 1:
            defect_result = process_detection_mode(data1);
            break;
        default:
            defect_result = 0;
            break;
    }

    return defect_result;
}
