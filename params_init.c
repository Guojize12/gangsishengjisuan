#include "app_user.h"
#include "params_init.h"
#include "position.h"
#include "app_memory.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*
 Centralized params storage:
 - single struct params_flash_t is stored at PARAMS_FLASH_ADDR (halfword index),
   with fields matching previous separate FLASH variables.
 - fields: magic + version + payload + checksum
 - Save: only when in-memory payload differs from flash (reduce write frequency)
*/

/* External variables referenced elsewhere in project (retain as before) */
extern uint16_t damage_degree;
extern uint16_t alarm_real_position[5];
extern uint16_t small_alarm_count;
extern uint16_t big_alarm_count;
extern uint16_t last_small_alarm_count;
extern uint16_t last_big_alarm_count;

extern uint8_t mode_switch ;
extern uint8_t flash_save_enable;
extern float MEAN_DEVIATION_THRESHOLD;
extern float SENSOR_DEVIATION_THRESHOLD;
extern float VARIANCE_THRESHOLD;
extern float TREND_THRESHOLD;
extern float DEFECT_SCORE_THRESHOLD;

extern gss_device  GSS_device;
extern gss_device_alarm_stat GSS_device_alarm_stat;

/* NOTE: nucSysIDNo 在旧代码为 uint32_t，但读写使用 EEPROM_FLASH_ReadU16，
   为兼容性，这里使用 uint32_t 存内存，但读/写时低16位有效 */
static uint32_t nucSysIDNo = 0;

/* 集中存储结构：请勿随意改变字段顺序（影响兼容性） */
typedef struct {
    uint32_t magic;                /* magic */
    uint16_t version;              /* structure version */
    uint16_t reserved;             /* 对齐 / 预留 */

    uint32_t nucSysIDNo;           /* 设备ID（低16位为兼容旧存储） */

    uint32_t position_range_upper;
    uint32_t position_range_lower;
    uint32_t position_signal_upper;
    uint32_t position_signal_lower;

    uint16_t Threshold_set1;
    uint16_t Threshold_set2;
    uint16_t Threshold_set3;

    uint32_t position_zero_point;

    uint16_t alarm_real_position[5]; /* 5 * 2B = 10B */

    uint16_t damage_degree;
    uint16_t small_alarm_count;
    uint16_t big_alarm_count;

    uint32_t g_total_meters;

    uint16_t mode_switch;
    uint16_t total_length;

    /* 浮点门限（保留原先量纲），按 float 存 */
    float MEAN_DEVIATION_THRESHOLD;
    float SENSOR_DEVIATION_THRESHOLD;
    float VARIANCE_THRESHOLD;
    float TREND_THRESHOLD;
    float DEFECT_SCORE_THRESHOLD;

    uint16_t flash_save_enable;

    uint32_t checksum;             /* 末尾校验和（对除 checksum 本身外所有字节求和） */
} params_flash_t;

/* 本地缓存（用于写前比对） */
static params_flash_t s_cached_flash;

/* 计算校验和（对整个结构体，checksum 字段置0再求字节和） */
static uint32_t params_calc_checksum(const params_flash_t *p)
{
    params_flash_t tmp;
    memcpy(&tmp, p, sizeof(tmp));
    tmp.checksum = 0;
    const uint8_t *bytes = (const uint8_t*)&tmp;
    uint32_t sum = 0;
    for (size_t i = 0; i < sizeof(tmp); ++i) sum += bytes[i];
    return sum;
}

/* 将内存运行时参数填充到 params_flash_t */
static void fill_params_flash_from_runtime(params_flash_t *p)
{
    memset(p, 0, sizeof(*p));
    p->magic = PARAMS_FLASH_MAGIC;
    p->version = PARAMS_FLASH_VERSION;
    p->reserved = 0;
    p->nucSysIDNo = nucSysIDNo;
    p->position_range_upper = GSS_device.position_range_upper;
    p->position_range_lower = GSS_device.position_range_lower;
    p->position_signal_upper = GSS_device.position_signal_upper;
    p->position_signal_lower = GSS_device.position_signal_lower;

    p->Threshold_set1 = GSS_device.Threshold_set1;
    p->Threshold_set2 = GSS_device.Threshold_set2;
    p->Threshold_set3 = GSS_device.Threshold_set3;

    p->position_zero_point = GSS_device.position_zero_point;

    for (int i = 0; i < 5; ++i) p->alarm_real_position[i] = alarm_real_position[i];

    p->damage_degree = damage_degree;
    p->small_alarm_count = small_alarm_count;
    p->big_alarm_count = big_alarm_count;

    p->g_total_meters = g_total_meters;

    p->mode_switch = (uint16_t)mode_switch;
    p->total_length = GSS_device.total_length;

    p->MEAN_DEVIATION_THRESHOLD = MEAN_DEVIATION_THRESHOLD;
    p->SENSOR_DEVIATION_THRESHOLD = SENSOR_DEVIATION_THRESHOLD;
    p->VARIANCE_THRESHOLD = VARIANCE_THRESHOLD;
    p->TREND_THRESHOLD = TREND_THRESHOLD;
    p->DEFECT_SCORE_THRESHOLD = DEFECT_SCORE_THRESHOLD;

    p->flash_save_enable = flash_save_enable;
    p->checksum = 0;
    p->checksum = params_calc_checksum(p);
}

/* 将 params_flash_t 解开回运行时变量 */
static void apply_params_flash_to_runtime(const params_flash_t *p)
{
    nucSysIDNo = p->nucSysIDNo;
    GSS_device.position_range_upper = p->position_range_upper;
    GSS_device.position_range_lower = p->position_range_lower;
    GSS_device.position_signal_upper = p->position_signal_upper;
    GSS_device.position_signal_lower = p->position_signal_lower;

    GSS_device.Threshold_set1 = p->Threshold_set1;
    GSS_device.Threshold_set2 = p->Threshold_set2;
    GSS_device.Threshold_set3 = p->Threshold_set3;

    GSS_device.position_zero_point = p->position_zero_point;

    for (int i = 0; i < 5; ++i) alarm_real_position[i] = p->alarm_real_position[i];

    damage_degree = p->damage_degree;
    small_alarm_count = p->small_alarm_count;
    big_alarm_count = p->big_alarm_count;
    last_small_alarm_count = small_alarm_count;
    last_big_alarm_count = big_alarm_count;

    g_total_meters = p->g_total_meters;

    mode_switch = (uint8_t)p->mode_switch;
    GSS_device.total_length = p->total_length;

    MEAN_DEVIATION_THRESHOLD = p->MEAN_DEVIATION_THRESHOLD;
    SENSOR_DEVIATION_THRESHOLD = p->SENSOR_DEVIATION_THRESHOLD;
    VARIANCE_THRESHOLD = p->VARIANCE_THRESHOLD;
    TREND_THRESHOLD = p->TREND_THRESHOLD;
    DEFECT_SCORE_THRESHOLD = p->DEFECT_SCORE_THRESHOLD;

    flash_save_enable = (uint8_t)(p->flash_save_enable & 0xff);
}

/* 将 params_flash_t 写入 flash（使用 APP_MEMORY_Write_BUF16） */
int PARAMS_SaveAll(void)
{
    params_flash_t local;
    fill_params_flash_from_runtime(&local);

    /* 读出当前 flash 内容到临时结构以便比较，避免不必要写入 */
    params_flash_t flash_now;
    memset(&flash_now, 0, sizeof(flash_now));
    APP_MEMORY_Read_BUF16(PARAMS_FLASH_ADDR, (uint16_t*)&flash_now, sizeof(flash_now) / 2);

    /* 如果完全一致（包括 checksum），则跳过写入 */
    if (memcmp(&local, &flash_now, sizeof(local)) == 0) {
        return 0; /* 无需写入 */
    }

    /* 写入（半字为单位） */
    APP_MEMORY_Write_BUF16(PARAMS_FLASH_ADDR, (uint16_t*)&local, sizeof(local) / 2);

    /* 更新缓存拷贝 */
    memcpy(&s_cached_flash, &local, sizeof(local));
    return 0;
}

/* 从Flash读取并校验，成功则填充运行时变量 */
int PARAMS_LoadAll(void)
{
    params_flash_t stored;
    memset(&stored, 0, sizeof(stored));
    int ret = APP_MEMORY_Read_BUF16(PARAMS_FLASH_ADDR, (uint16_t*)&stored, sizeof(stored) / 2);
    if (ret != 0 && ret != USR_EOK) {
        LOG("PARAMS_LoadAll: read failed\n");
        return -1;
    }

    if (stored.magic != PARAMS_FLASH_MAGIC) {
        LOG("PARAMS_LoadAll: magic mismatch: 0x%08X\n", (unsigned)stored.magic);
        return -2;
    }
    if (stored.version != PARAMS_FLASH_VERSION) {
        LOG("PARAMS_LoadAll: version mismatch: %u\n", (unsigned)stored.version);
        /* 若版本不一致，可在此实现兼容迁移逻辑；当前选择回退默认 */
        return -3;
    }
    uint32_t calc = params_calc_checksum(&stored);
    if (calc != stored.checksum) {
        LOG("PARAMS_LoadAll: checksum mismatch (read=%u calc=%u)\n", (unsigned)stored.checksum, (unsigned)calc);
        return -4;
    }

    /* 校验通过：应用到运行时 */
    apply_params_flash_to_runtime(&stored);
    memcpy(&s_cached_flash, &stored, sizeof(stored));
    return 0;
}

/* 仅校验Flash是否有效（不加载） */
int PARAMS_IsValid(void)
{
    params_flash_t stored;
    memset(&stored, 0, sizeof(stored));
    int ret = APP_MEMORY_Read_BUF16(PARAMS_FLASH_ADDR, (uint16_t*)&stored, sizeof(stored) / 2);
    if (ret != 0 && ret != USR_EOK) return 0;
    if (stored.magic != PARAMS_FLASH_MAGIC) return 0;
    if (stored.version != PARAMS_FLASH_VERSION) return 0;
    if (stored.checksum != params_calc_checksum(&stored)) return 0;
    return 1;
}

/* 恢复为默认参数并写入（被旧的 WriteDefaultParamsToFlash 调用） */
void PARAMS_ResetDefaults(void)
{
    /* 设置默认值（与原 WriteDefaultParamsToFlash 保持一致或更安全的默认） */
    nucSysIDNo = 9587;
    GSS_device.position_range_upper = 2;
    GSS_device.position_range_lower = 0;
    GSS_device.position_signal_upper = 3434944;
    GSS_device.position_signal_lower = 3134944;
    GSS_device.Threshold_set1 = 120;
    GSS_device.Threshold_set2 = 200;
    GSS_device.Threshold_set3 = 9;

    for (int i = 0; i < 5; ++i) alarm_real_position[i] = 0;

    damage_degree = 100;
    small_alarm_count = 0;
    big_alarm_count = 0;
    last_small_alarm_count = 0;
    last_big_alarm_count = 0;

    g_total_meters = 0;

    mode_switch = 0;

    GSS_device.total_length = 200;

    MEAN_DEVIATION_THRESHOLD = 200.0f;
    SENSOR_DEVIATION_THRESHOLD = 200.0f;
    VARIANCE_THRESHOLD = 100.0f;
    TREND_THRESHOLD = 50.0f;
    DEFECT_SCORE_THRESHOLD = 9.0f;

    flash_save_enable = 0;

    /* 将默认写入Flash */
    PARAMS_SaveAll();
}

/* 兼容保留的旧接口：用新的集中函数实现 */
void LoadParamsFromFlash(void)
{
    int r = PARAMS_LoadAll();
    if (r != 0) {
        /* 读取失败：回退到逐项读（保留兼容逻辑），但更推荐直接调用默认恢复 */
        /* 尝试从旧单项位置读取（尽量兼容） */
        nucSysIDNo  = (uint32_t)APP_MEMORY_Read_U16(FLASHID);
        GSS_device.position_range_upper = APP_MEMORY_Read_U32(FLASH_POS_TOP_DATA);
        GSS_device.position_range_lower = APP_MEMORY_Read_U32(FLASH_POS_BUT_DATA);
        GSS_device.position_signal_upper = APP_MEMORY_Read_U32(FLASH_SIG_TOP_DATA);
        GSS_device.position_signal_lower = APP_MEMORY_Read_U32(FLASH_SIG_BUT_DATA);
        GSS_device.Threshold_set1 = APP_MEMORY_Read_U16(FLASH_THRESHOLD_1);
        GSS_device.Threshold_set2 = APP_MEMORY_Read_U16(FLASH_THRESHOLD_2);
        GSS_device.Threshold_set3 = APP_MEMORY_Read_U16(FLASH_THRESHOLD_3);
        GSS_device.position_zero_point = APP_MEMORY_Read_U32(FLASH_POSITION_ZERO_POINT);
        for (int i = 0; i < 5; ++i)
            alarm_real_position[i] = APP_MEMORY_Read_U16(FLASH_ALARM_POSITION_COUNT1 + i*2);
        damage_degree = APP_MEMORY_Read_U16(FLASH_DAMAGE_DEGREE);
        small_alarm_count = APP_MEMORY_Read_U16(FLASH_SMALL_ALARM_COUNT);
        last_small_alarm_count = small_alarm_count;
        big_alarm_count = APP_MEMORY_Read_U16(FLASH_BIG_ALARM_COUNT);
        last_big_alarm_count = big_alarm_count;
        g_total_meters = APP_MEMORY_Read_U32(FLASH_TOTAL_METERS);

        MEAN_DEVIATION_THRESHOLD = (float)APP_MEMORY_Read_U16(FLASH_MEAN_DEVIATION_THRESHOLD) / 10.0f;
        SENSOR_DEVIATION_THRESHOLD = (float)APP_MEMORY_Read_U16(FLASH_SENSOR_DEVIATION_THRESHOLD) / 10.0f;
        VARIANCE_THRESHOLD = (float)APP_MEMORY_Read_U16(FLASH_VARIANCE_THRESHOLD) / 10.0f;
        TREND_THRESHOLD = (float)APP_MEMORY_Read_U16(FLASH_TREND_THRESHOLD) / 10.0f;
        DEFECT_SCORE_THRESHOLD = (float)APP_MEMORY_Read_U16(FLASH_DEFECT_SCORE_THRESHOLD) / 10.0f;
        flash_save_enable = (uint8_t)APP_MEMORY_Read_U16(FLASH_SAVE_ENABLE);

        /* 若旧区也无有效 ID，则恢复默认并写集中区 */
        if (nucSysIDNo == 0) {
            PARAMS_ResetDefaults();
        }
    }
}

void WriteDefaultParamsToFlash(void)
{
    PARAMS_ResetDefaults();
}

/* 现有 loadini() 调用链不变（保持兼容） */
void loadini(void)
{
    InitCoreData();
    LoadParamsFromFlash();
    /* 如果读取后检测到 nucSysIDNo == 0（代表旧区空），LoadParamsFromFlash 内会执行 ResetDefaults */
    InitSmartCalibration();
    srand(HAL_GetTick());
}

void InitCoreData(void)
{
    position_diff = 0;
    g_current_position = 0;

    GSS_device.real_speed = 0.0f;
    GSS_device.run_direction = 0;
    GSS_device.position_data_real = 0.0f;
    GSS_device.position_data_ad = 0;
    GSS_device.alarm = 0;
    GSS_device.degree_of_damage = 0;

    GSS_device.position_slope = 0.0f;
    GSS_device.position_offset = 0.0f;
}
