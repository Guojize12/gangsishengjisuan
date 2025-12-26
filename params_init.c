#include "app_user.h"
#include "params_init.h"
#include "position.h" // 使用 g_current_position / g_total_meters / position_diff

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

static uint32_t    nucSysIDNo = 0;          /*设备ID-8034     2025-05-19*/

// 1. 核心数据初始化
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

// 2. 从FLASH加载参数
void LoadParamsFromFlash(void)
{
    nucSysIDNo  = EEPROM_FLASH_ReadU16(FLASHID);
    GSS_device.position_range_upper = EEPROM_FLASH_ReadU32(FLASH_POS_TOP_DATA);
    GSS_device.position_range_lower = EEPROM_FLASH_ReadU32(FLASH_POS_BUT_DATA);
    GSS_device.position_signal_upper = EEPROM_FLASH_ReadU32(FLASH_SIG_TOP_DATA);
    GSS_device.position_signal_lower = EEPROM_FLASH_ReadU32(FLASH_SIG_BUT_DATA);
    GSS_device.Threshold_set1 = EEPROM_FLASH_ReadU16(FLASH_THRESHOLD_1);
    GSS_device.Threshold_set2 = EEPROM_FLASH_ReadU16(FLASH_THRESHOLD_2);
    GSS_device.Threshold_set3 = EEPROM_FLASH_ReadU16(FLASH_THRESHOLD_3);
//    alarm_button_or_dwin = EEPROM_FLASH_ReadU16(FLASH_BUTTON_OR_DWIN);
    GSS_device.position_zero_point = EEPROM_FLASH_ReadU32(FLASH_POSITION_ZERO_POINT);

    for (int i = 0; i < 5; i++) {
        alarm_real_position[i] = EEPROM_FLASH_ReadU16(FLASH_ALARM_POSITION_COUNT1 + i * 2);
    }
    damage_degree = EEPROM_FLASH_ReadU16(FLASH_DAMAGE_DEGREE);
    small_alarm_count = EEPROM_FLASH_ReadU16(FLASH_SMALL_ALARM_COUNT);
    last_small_alarm_count = small_alarm_count;
    big_alarm_count = EEPROM_FLASH_ReadU16(FLASH_BIG_ALARM_COUNT);
    last_big_alarm_count = big_alarm_count;
    g_total_meters = EEPROM_FLASH_ReadU32(FLASH_TOTAL_METERS);
		s_total_distance_m_f = g_total_meters / 1000.0f;
		
    // mode_switch = EEPROM_FLASH_ReadU16(FLASH_MODE_SWITCH);
    MEAN_DEVIATION_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_MEAN_DEVIATION_THRESHOLD) / 10;
    SENSOR_DEVIATION_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_SENSOR_DEVIATION_THRESHOLD) / 10;
    VARIANCE_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_VARIANCE_THRESHOLD) / 10;
    TREND_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_TREND_THRESHOLD) / 10;
    DEFECT_SCORE_THRESHOLD = (float)EEPROM_FLASH_ReadU16(FLASH_DEFECT_SCORE_THRESHOLD) / 10;
    flash_save_enable = EEPROM_FLASH_ReadU16(FLASH_SAVE_ENABLE);
		
//		LOGT("掉电恢复: g_total_meters(flash)=%lu, s_total_distance_m_f=%.3f\n", g_total_meters, s_total_distance_m_f);
//    LOGT("掉电恢复: AD上限=%lu, AD下限=%lu, 斜率=%.7f, 偏置=%.3f\n",
//		GSS_device.position_signal_upper,
//		GSS_device.position_signal_lower,
//		GSS_device.position_slope,
//		GSS_device.position_offset);
//    LOGT("启动: g_total_meters(LOAD)%lu, s_total_distance_m_f=%.3f\n", g_total_meters, s_total_distance_m_f);


}

// 3. 初次上电或参数丢失时默认参数写入FLASH
void WriteDefaultParamsToFlash(void)
{
    nucSysIDNo = 9587;
    EEPROM_FLASH_WriteU16(FLASHID, nucSysIDNo);
    GSS_device.position_range_upper = 2;
    EEPROM_FLASH_WriteU32(FLASH_POS_TOP_DATA, GSS_device.position_range_upper);
    GSS_device.position_range_lower = 0;
    EEPROM_FLASH_WriteU32(FLASH_POS_BUT_DATA, GSS_device.position_range_lower);
    GSS_device.position_signal_upper = 3434944;
    EEPROM_FLASH_WriteU32(FLASH_SIG_TOP_DATA, GSS_device.position_signal_upper);
    GSS_device.position_signal_lower = 3134944;
    EEPROM_FLASH_WriteU32(FLASH_SIG_BUT_DATA, GSS_device.position_signal_lower);
    GSS_device.Threshold_set1 = 120;
    EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_1, GSS_device.Threshold_set1);
    GSS_device.Threshold_set2 = 200;
    EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_2, GSS_device.Threshold_set2);
    GSS_device.Threshold_set3 = 9;
    EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_3, GSS_device.Threshold_set3);
//    alarm_button_or_dwin = 0;
//	  alarm_button_or_dwin = 1;
//    EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);

    for (int i = 0; i < 5; i++) {
        alarm_real_position[i] = 0;
        EEPROM_FLASH_WriteU16(FLASH_ALARM_POSITION_COUNT1 + i * 2,
                              (uint16_t)alarm_real_position[i]);
    }
    damage_degree = 100;
    EEPROM_FLASH_WriteU16(FLASH_DAMAGE_DEGREE, damage_degree);

    g_total_meters = 0;
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


// 5. 主函数
void loadini(void)
{
    InitCoreData();
    LoadParamsFromFlash();

    if (nucSysIDNo == 0) {
        WriteDefaultParamsToFlash();
    }

    InitSmartCalibration();

    srand(HAL_GetTick());
}