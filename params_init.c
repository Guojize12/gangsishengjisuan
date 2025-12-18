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

    if (nucSysIDNo == 0)
    {
        nucSysIDNo = 9587;
        EEPROM_FLASH_WriteU16(FLASHID, nucSysIDNo);
        GSS_device.position_range_upper = 200;
        EEPROM_FLASH_WriteU32(FLASH_POS_TOP_DATA, GSS_device.position_range_upper);
        GSS_device.position_range_lower = 0;
        EEPROM_FLASH_WriteU32(FLASH_POS_BUT_DATA, GSS_device.position_range_lower);
        GSS_device.position_signal_upper = 3434944;
        EEPROM_FLASH_WriteU32(FLASH_SIG_TOP_DATA, GSS_device.position_signal_upper);
        GSS_device.position_signal_lower = 3134944;
        EEPROM_FLASH_WriteU32(FLASH_SIG_BUT_DATA, GSS_device.position_signal_lower);
        GSS_device.Threshold_set1 = 120;//轻微损伤电压标定
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_1, GSS_device.Threshold_set1);
        GSS_device.Threshold_set2 = 200;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_2, GSS_device.Threshold_set2);
        GSS_device.Threshold_set3 = 9;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_3, GSS_device.Threshold_set3);
        alarm_button_or_dwin = 0;//按钮标定为0，DWIN标定为1
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
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
    // ===== 智能标定初始化逻辑（保持原有流程，不改DWIN）=====
    if (alarm_button_or_dwin == 0)
    {
        // 按钮标定模式下，零点已从FLASH读取，无需额外处理
    }
    else if (alarm_button_or_dwin == 1)
    {
        float aucy1 = (float)GSS_device.position_range_upper; // 位置量程上限
        float aucy2 = (float)GSS_device.position_range_lower; // 位置量程下限
        uint32_t aucx1 =  GSS_device.position_signal_upper;  // 位置信号上限
        uint32_t aucx2 = GSS_device.position_signal_lower;  // 位置信号下限

        if (aucx1 != aucx2)
        {
            GSS_device.position_slope = (aucy1 - aucy2) / (aucx1 - aucx2);
            GSS_device.position_offset = aucy1 - GSS_device.position_slope * aucx1;
        }
        else
        {
            GSS_device.position_slope = 0.0f;
            GSS_device.position_offset = 0.0f;
            alarm_button_or_dwin = 0;
        }
        uint32_t range_zero_point = GSS_device.position_range_lower;

        if (GSS_device.position_zero_point == 0)
        {
            GSS_device.position_zero_point = range_zero_point;
            EEPROM_FLASH_WriteU32(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
        }
        else
        {
            if (GSS_device.position_zero_point < GSS_device.position_range_lower ||
                    GSS_device.position_zero_point > GSS_device.position_range_upper)
            {
                // 日志位置预留：零点越界警告
            }
        }
    }
    else
    {
        alarm_button_or_dwin = 0;
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
    }

    srand(HAL_GetTick());
}