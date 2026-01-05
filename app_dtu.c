#include "app_dtu.h"
#include "app_config.h"

#define APP_DTU_UART          BSP_UART3
#define APP_DTU_UART_BUF      g_uart_buf[APP_DTU_UART]

#define APP_DTU_SIGNAL_TIMEOUT  (6200)  //dtu连接超时判断时间:15秒200毫秒

extern gss_device  GSS_device;
extern gss_device_alarm_stat GSS_device_alarm_stat;
static app_rtu_rx_def g_dtu_rx; //dtu 多通道接收数据

static Timer g_timer_dtu = {0};
extern uint32_t g_current_position ;     // 当前位置值
extern  int32_t position_diff;

extern uint8_t alarm_dtu_trig ;
extern uint16_t damage_degree;
extern uint16_t small_alarm_count ;
extern uint16_t big_alarm_count ;
extern AlarmInfo alarm_info_max;

// 卡尔曼滤波器实时数据
extern float MEAN_DEVIATION_THRESHOLD;      // 均值偏差实时阈值
extern float SENSOR_DEVIATION_THRESHOLD;    // 单传感器实时阈值
extern float VARIANCE_THRESHOLD;            // 方差异常实时阈值
extern float trend_deviation;               // 趋势异常实时数据
extern float DEFECT_SCORE_THRESHOLD;        // 综合判断实时阈值
extern float filtered_value11;              // 滤波实时数据

dtu_remote_def  g_dtu_remote =
{
    .uint_sn = REMOTE_SN,
    .uint_model = REMOTE_MODEL,
    .uint_ver = REMOTE_VER,
};

dtu_cmd_def g_dtu_cmd = {0};
bsp_gps_def g_gps_date = {0};
dtu_remote_cmd_def g_dtu_remote_cmd =
{
    .normal_interval = 20,
    .work_interval = 20,
};

void APP_DTU_Status_Reset(void)
{
    g_dtu_cmd.cnt_status = USR_ERROR;
    g_dtu_cmd.power_on_status = USR_ERROR;
    g_dtu_cmd.response_cmd = DTU_CNT_STEP0;
    g_dtu_cmd.send_num = 0;
    g_dtu_cmd.time_num = 0; //dtu上传时间判断计时清0

    g_dtu_cmd.response_num = 0; //dtu重置时清零，避免立即进入超时状态

    g_dtu_cmd.gps_send_interval = 31; //定位上传间隔31s
    g_dtu_cmd.gps_enable = 0; //不支持定位功能
}

//dtu串口直发
void APP_DTU_Send_Buf(uint8_t *buffer, uint16_t size)
{
    BSP_UART_Transmit(APP_DTU_UART, buffer, size);
//  LOG("%s\n",buffer);
}

//dtu多通道发
void APP_DTU_Send(uint8_t *buffer, uint16_t size)
{

    APP_RTU_AT_Tx_Chl(0, buffer, size);
}
//指令头打包
void APP_DTU_Head_Packing(uint8_t type, uint8_t *txBuf, uint16_t not_head_len, uint16_t cmd, uint8_t pid)
{
    memcpy(txBuf, g_dtu_cmd.head, 20);
    txBuf[1] = type;
    txBuf[2] = (uint8_t)(not_head_len >> 8);
    txBuf[3] = (uint8_t)not_head_len & 0xFF;
    txBuf[17] = (uint8_t)(cmd >> 8);
    txBuf[18] = (uint8_t)cmd & 0xFF;
    txBuf[20] = pid;
    uint16_t crc16 = bsp_crc16((uint8_t*)txBuf, 21);

    txBuf[21] = (uint8_t)crc16 & 0xFF;
    txBuf[22] = (uint8_t)(crc16 >> 8);
}

//设备通用应答
void APP_DTU_Response_Result(uint16_t cmd, uint8_t   state, uint8_t *rxBuf, uint16_t rxLen)
{
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, 1, cmd, rxBuf[20]);
    txBuf[23] = state;
    uint16_t crc16 = bsp_crc16(txBuf + 23, 1);
    txBuf[24] = (uint8_t)crc16 & 0xFF;
    txBuf[25] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, 26);
}

//设备心跳
void APP_DTU_Send_Hearbeat(void)
{
    LOGT("smt:ht\n");
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, 0, DTU_CMD_DEVICE_HEARTBEAT, 0);
    APP_DTU_Send(txBuf, 23);

//  LOGT("send heart_beat to remote [%d]\n", 23);
//    LOG_HEX(txBuf, 23);
    if (g_dtu_cmd.net_status == USR_STATE_OFF)
    {
        LOGT("smt:ht,no net to cnt..\n");
    }
}

//设备主动获取服务器时间
void APP_DTU_GetServerTime(void)
{
    LOGT("smt:get time\n");
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, 0, DTU_CMD_DEVICE_TIMESYNC, 0);
    APP_DTU_Send(txBuf, 23);
}

void APP_DTU_SendDTUPowerOnData(void)
{
    LOGT("smt:send poweron\n");
    uint8_t txBuf[100];
    uint16_t num = 23;

    BSP_RTC_Get(&g_bsp_rtc);
    txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8;
    txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
    txBuf[num++] = g_bsp_rtc.month;
    txBuf[num++] = g_bsp_rtc.day;
    txBuf[num++] = g_bsp_rtc.hour;
    txBuf[num++] = g_bsp_rtc.minute;
    txBuf[num++] = g_bsp_rtc.second;

    txBuf[num++] = 0;
    txBuf[num++] = 0;
    txBuf[num++] = 0;
    txBuf[num++] = 0;

    uint8_t version[3];
    APP_VERSION_Get_Soft(version);//软件版本
    txBuf[num++] = version[0];
    txBuf[num++] = version[1];
    txBuf[num++] = version[2];

    APP_VERSION_Get_Hard(version);//硬件版本
    txBuf[num++] = version[0];
    txBuf[num++] = version[1];
    txBuf[num++] = version[2];

//    APP_CONFIG_Get_Model();
    memcpy(txBuf + num, g_app_cfg.model, 20);
    num += 20;

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_DEVICE_POWE_ON_STATUS, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
}


//sim卡信息
void APP_DTU_SendDTUSim(void)
{
    if (strlen(g_app_rtu_sim.iccid) < 10)
    {
        LOGT("err:read iccid none.\n");
        return;
    }

    LOGT("smt:sim iccid.\n");

    uint8_t txBuf[64] = {0};
    uint16_t num = 23;

    BSP_RTC_Get(&g_bsp_rtc);
    txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8;
    txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
    txBuf[num++] = g_bsp_rtc.month;
    txBuf[num++] = g_bsp_rtc.day;
    txBuf[num++] = g_bsp_rtc.hour;
    txBuf[num++] = g_bsp_rtc.minute;
    txBuf[num++] = g_bsp_rtc.second;

    txBuf[num++] = 20; //Sim卡号ICCID长度
    memcpy(txBuf + num, g_app_rtu_sim.iccid, 20);
    num += 20;
    txBuf[num++] = 0; //Sim入网号长度
    txBuf[num++] = g_app_rtu_sim.signal_per; //信号强度

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_DEVICE_SIM, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);

}

//时间同步
void APP_DTU_TimeSync_Set(uint8_t *rxBuf, uint16_t rxLen)
{

    LOGT("rtc %d-%d\n", rxBuf[23], rxBuf[24]);
    uint16_t ttt = (rxBuf[23] * 256) + rxBuf[24];

    // 修复：判断是否为2位年份（服务器下发校时）
    if (ttt < 100)  // 服务器下发的2位年份
    {
        ttt += 2000;  // 补上2000
    }

    g_bsp_rtc.year = ttt % 100;
    g_bsp_rtc.month = rxBuf[25];
    g_bsp_rtc.day = rxBuf[26];
    g_bsp_rtc.hour = rxBuf[27];
    g_bsp_rtc.minute = rxBuf[28];
    g_bsp_rtc.second = rxBuf[29];
    if (ttt > 2019)
    {
        BSP_RTC_Set(g_bsp_rtc);
    }
    LOGT("timing:%04d-%02d-%02d %02d:%02d:%02d\n",
         (ttt > 100 ? ttt : 2000 + g_bsp_rtc.year),
         g_bsp_rtc.month, g_bsp_rtc.day,
         g_bsp_rtc.hour, g_bsp_rtc.minute, g_bsp_rtc.second);

}

//设备心跳
void APP_DTU_Response_Hearbeat(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    uint8_t txBuf[64] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, 0, cmd, rxBuf[20]);
    APP_DTU_Send(txBuf, 23);
}

void APP_DTU_Cmd_Upload_Interval_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data;
    ie_data.u8_buf[3] = rxBuf[23];
    ie_data.u8_buf[2] = rxBuf[24];
    ie_data.u8_buf[1] = rxBuf[25];
    ie_data.u8_buf[0] = rxBuf[26];
    g_dtu_remote_cmd.normal_interval = ie_data.u32 * 10;

    ie_data.u8_buf[3] = rxBuf[27];
    ie_data.u8_buf[2] = rxBuf[28];
    ie_data.u8_buf[1] = rxBuf[29];
    ie_data.u8_buf[0] = rxBuf[30];
    g_dtu_remote_cmd.work_interval = ie_data.u32 * 10;
    //TODO 存储 实时数据 上传间隔
//    APP_CONFIG_User_Upload_Time_Set(g_dtu_remote_cmd.normal_interval,g_dtu_remote_cmd.work_interval);
    APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
    LOGT("updata interval normal[%d],work[%d]\n", g_dtu_remote_cmd.normal_interval, g_dtu_remote_cmd.work_interval);

}
/* 参数设置 */
void nucRecDTUSetParaData(uint16_t cmd, uint32_t  addr, uint32_t dat, uint8_t *rxBuf, uint16_t rxLen)
{
    extern float MEAN_DEVIATION_THRESHOLD;
    extern float SENSOR_DEVIATION_THRESHOLD;
    extern float VARIANCE_THRESHOLD;
    extern float TREND_THRESHOLD;
    extern float DEFECT_SCORE_THRESHOLD;
    extern uint16_t flash_save_enable;
    extern uint16_t alarm_button_or_dwin;

    uint8_t ret = DTU_CMD_RESPONSE_SUCCESS;

    // 声明所有可能用到的局部变量（避免switch-case警告）
    float pos_upper1, pos_lower1, pos_upper2, pos_lower2, pos_zero;
    uint32_t sig_upper1, sig_lower1, sig_upper2, sig_lower2;

    LOGT("set addr:%d --value:%d\n", addr, dat);
    switch (addr)
    {
    // 公共参数
    case 0x0001://修改设备号
        APP_CONFIG_Did_Set(dat);
        break;
    case 0x0002://正常上传间隔  秒
        g_dtu_remote_cmd.normal_interval = dat * 10; // 秒转100ms单位：秒×10
        //TODO 存储到EEPROM: APP_CONFIG_User_Upload_Time_Set(dat*10,0);
        break;
    case 0x0003://运行时上传间隔 秒
        g_dtu_remote_cmd.work_interval = dat * 10; // 秒转100ms单位：秒×10
        //TODO 存储到EEPROM: APP_CONFIG_User_Upload_Time_Set(0,dat*10);
        break;
    case 0x0005://锁机 参数 0正常， 1锁机
        //TODO 实现锁机功能
        break;
    case 0x0006://迪文电表支持 不为零时上传参数消息
        if (dat != 0)
        {
            APP_DTU_Send_System_config(); // 上传配置信息
        }
        return; // 不发送通用回复
    case 0x0007://设备登录密码 6位数字
        //TODO APP_CONFIG_Set_DwinPassword(dat);
        break;
    case 0x0008://心跳间隔 秒
        //TODO 实现心跳间隔设置
        break;
    case 0x0009://自动校准配置 0关闭 1打开
        //TODO 实现自动校准配置
        break;
    case 0x000A://reboot重启
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("system:rebooting...\n");
        BSP_DELAY_MS(100);
        BSP_CONFIG_System_Reset();
        return;
    case 0x000B://DTU重启
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("dtu:rebooting...\n");
        BSP_DELAY_MS(200);
        //TODO BSP_POWER_Reboot(0); // 如果硬件支持
        return;
    case 0x000C://恢复出厂
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("factory:resetting...\n");
        BSP_DELAY_MS(200);
        //TODO BSP_EEPROM_Reset_Factory();
        return;
    case 0x000D://静音 0正常 1静音断电保存 2静音断电不保存
        //TODO 实现静音功能
        break;
    case 0x000E://运行模式 0正常 1调试模式 2调试10分钟 3非工作模式
        //TODO 实现运行模式切换
        break;
    case 0x000F://设备控制台透传 0关闭 1开启5分钟 2永久开启
        //TODO 实现透传功能
        break;

    // 钢丝绳探伤参数 0x1e00-0x1e12
    case 0x1e00://位置量程上限 米
        GSS_device.position_range_upper = dat;
        EEPROM_FLASH_WriteU32(FLASH_POS_TOP_DATA, GSS_device.position_range_upper); //保存位置量程上限
        GSS_device.position_signal_upper = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_TOP_DATA, GSS_device.position_signal_upper);//保存位置信号上限
        alarm_button_or_dwin = 1;//按钮标定为0，DWIN标定为1
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        /*************位置参数计算*************************/
        pos_upper1 = (float)GSS_device.position_range_upper; // 位置量程上限
        pos_lower1 = (float)GSS_device.position_range_lower; // 位置量程下限
        sig_upper1 = GSS_device.position_signal_upper;  // 位置信号上限
        sig_lower1 = GSS_device.position_signal_lower;  // 位置信号下限
        if (sig_upper1 != sig_lower1)
        {
            // 计算线性标定参数
            GSS_device.position_slope = (pos_upper1 - pos_lower1) / (sig_upper1 - sig_lower1);
            GSS_device.position_offset = pos_upper1 - GSS_device.position_slope * sig_upper1;
        }
        /*************位置参数计算结束*************************/
        LOGT("gss:position_range_upper=%d\n", dat);
        break;
    case 0x1e01://位置量程下限 米
        GSS_device.position_range_lower = dat;
        EEPROM_FLASH_WriteU32(FLASH_POS_BUT_DATA, GSS_device.position_range_lower);//保存位置量程下限
        GSS_device.position_signal_lower = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_BUT_DATA, GSS_device.position_signal_lower);//保存位置信号下限
        alarm_button_or_dwin = 1;//按钮标定为0，DWIN标定为1
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        /*************位置参数计算*************************/
        pos_lower2 = (float)GSS_device.position_range_lower; // 位置量程下限
        pos_upper2 = (float)GSS_device.position_range_upper; // 位置量程上限
        sig_lower2 = GSS_device.position_signal_lower;  // 位置信号下限
        sig_upper2 = GSS_device.position_signal_upper;  // 位置信号上限
        if (sig_upper2 != sig_lower2)
        {
            // 计算线性标定参数
            GSS_device.position_slope = (pos_upper2 - pos_lower2) / (sig_upper2 - sig_lower2);
            GSS_device.position_offset = pos_upper2 - GSS_device.position_slope * sig_upper2;
        }
        /*************位置参数计算结束*************************/
        LOGT("gss:position_range_lower=%d\n", dat);
        break;
    case 0x1e02://位置信号上限
        GSS_device.position_signal_upper = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_TOP_DATA, GSS_device.position_signal_upper);//保存位置信号上限
        alarm_button_or_dwin = 1;//按钮标定为0，DWIN标定为1
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        LOGT("gss:position_signal_upper=%d\n", dat);
        break;
    case 0x1e03://位置信号下限
        GSS_device.position_signal_lower = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_BUT_DATA, GSS_device.position_signal_lower);//保存位置信号下限
        alarm_button_or_dwin = 1;//按钮标定为0，DWIN标定为1
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        LOGT("gss:position_signal_lower=%d\n", dat);
        break;
    case 0x1e04://总绳长设置 米
        GSS_device.total_length = dat;
        EEPROM_FLASH_WriteU16(TOTAL_LEN_1, GSS_device.total_length);
        LOGT("gss:total_length=%d\n", dat);
        break;
    case 0x1e05://均值偏差阈值 单位0.1
        MEAN_DEVIATION_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_MEAN_DEVIATION_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:MEAN_DEVIATION_THRESHOLD=%f\n", MEAN_DEVIATION_THRESHOLD);
        break;
    case 0x1e06://单传感器阈值 单位0.1
        SENSOR_DEVIATION_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_SENSOR_DEVIATION_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:SENSOR_DEVIATION_THRESHOLD=%f\n", SENSOR_DEVIATION_THRESHOLD);
        break;
    case 0x1e07://方差异常阈值 单位0.1
        VARIANCE_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_VARIANCE_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:VARIANCE_THRESHOLD=%f\n", VARIANCE_THRESHOLD);
        break;
    case 0x1e08://趋势异常阈值 单位0.1
        TREND_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_TREND_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:TREND_THRESHOLD=%f\n", TREND_THRESHOLD);
        break;
    case 0x1e09://缺陷得分阈值 单位0.1
        DEFECT_SCORE_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_DEFECT_SCORE_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:DEFECT_SCORE_THRESHOLD=%f\n", DEFECT_SCORE_THRESHOLD);
        break;
    case 0x1e0a://存储模式 0或1
        //TODO 实现存储模式设置
        if (dat == 0)
        {
            flash_save_enable = 0;//不保存
            EEPROM_FLASH_WriteU16(FLASH_SAVE_ENABLE, 0);
        }
        else
        {
            flash_save_enable = 1;//保存
            EEPROM_FLASH_WriteU16(FLASH_SAVE_ENABLE, 1);
        }
        break;
    case 0x1e0b://工作算法模式 0或1
        //TODO 实现工作算法模式设置
        break;
    case 0x1e0c://损1轻微损伤标定 单位1
        GSS_device.Threshold_set1 = dat;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_1, GSS_device.Threshold_set1);
        break;
    case 0x1e0d://损1严重损伤标定 单位1
        GSS_device.Threshold_set2 = dat;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_2, GSS_device.Threshold_set2);
        break;
    case 0x1e0f://损1波长1宽度 单位1
        GSS_device.Threshold_set3 = dat;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_3, GSS_device.Threshold_set3);
        break;
    case 0x1e10://损1波长2宽度 单位1
        GSS_device.Threshold_set4 = dat;
        break;
    case 0x1e11://一键标定
        //TODO 实现一键标定功能
        GSS_device.position_zero_point = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
        alarm_button_or_dwin = 0;//按钮标定为0，DWIN标定为1
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        /*************位置参数计算*************************/
        pos_zero = (float)GSS_device.position_zero_point; // 位置标定零点
        if (pos_zero != 0)
        {
            // 计算线性标定参数
            GSS_device.position_slope = (pos_zero - pos_zero) / (pos_zero - pos_zero);
            GSS_device.position_offset = pos_zero - GSS_device.position_slope * pos_zero;
        }
        /*************位置参数计算结束*************************/
        break;

    default:
        ret = DTU_CMD_RESPONSE_FAIL;
        LOGT("error:unknown addr:0x%04X\n", addr);
        break;
    }

    APP_DTU_Response_Result(cmd, ret, rxBuf, rxLen);
}

void APP_DTU_Cmd_Config_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data = {0};
    uint32_t addr, data;
    ie_data.u8_buf[3] = rxBuf[23];
    ie_data.u8_buf[2] = rxBuf[24];
    ie_data.u8_buf[1] = rxBuf[25];
    ie_data.u8_buf[0] = rxBuf[26];
    addr = ie_data.i32;
    ie_data.u8_buf[3] = rxBuf[27];
    ie_data.u8_buf[2] = rxBuf[28];
    ie_data.u8_buf[1] = rxBuf[29];
    ie_data.u8_buf[0] = rxBuf[30];
    data = ie_data.i32;

    nucRecDTUSetParaData(cmd, addr, data, rxBuf, rxLen);
}
/* 参数设置 */

/* 参数获取 0x8014*/
void APP_DTU_Cmd_Config_Get(uint16_t cmd, uint32_t addr, uint8_t pid)
{
    IEEE754 ie_data = {0};

    uint8_t txBuf[128] = {0};
    uint16_t num = 23;

    ie_data.u32 = addr;//参数ID
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    txBuf[num++] = 1; //参数个数

    extern float MEAN_DEVIATION_THRESHOLD;
    extern float SENSOR_DEVIATION_THRESHOLD;
    extern float VARIANCE_THRESHOLD;
    extern float TREND_THRESHOLD;
    extern float DEFECT_SCORE_THRESHOLD;

    switch (addr)
    {
    // 公共参数
    case 0x0001://设备号
        ie_data.u32 = g_app_cfg.did;
        break;
    case 0x0002://正常上传间隔  秒
        ie_data.u32 = g_dtu_remote_cmd.normal_interval / 10;
        break;
    case 0x0003://运行时上传间隔 秒
        ie_data.u32 = g_dtu_remote_cmd.work_interval / 10;
        break;
    case 0x0005://锁机 参数 0正常 1锁机
        ie_data.u32 = 0; //TODO 读取锁机状态
        break;
    case 0x0007://设备登录密码  6位数字
        ie_data.u32 = 0; //TODO 读取密码
        break;
    case 0x0008://心跳间隔 秒
        ie_data.u32 = 0; //TODO 读取心跳间隔
        break;
    case 0x0009://自动校准配置 0关闭 1打开
        ie_data.u32 = 0; //TODO 读取自动校准状态
        break;
    case 0x000D://静音 0正常 1静音断电保存 2静音断电不保存
        ie_data.u32 = 0; //TODO 读取静音状态
        break;
    case 0x000E://运行模式
        ie_data.u32 = 0; //TODO 读取运行模式
        break;
    case 0x000F://设备控制台透传
        ie_data.u32 = 0; //TODO 读取透传状态
        break;

    // 钢丝绳探伤参数 0x1e00-0x1e12
    case 0x1e00://位置量程上限 米 单位0.1
        ie_data.u32 = GSS_device.position_range_upper;
        break;
    case 0x1e01://位置量程下限 米 单位0.1
        ie_data.u32 = GSS_device.position_range_lower;
        break;
    case 0x1e02://位置信号上限
        ie_data.u32 = GSS_device.position_signal_upper;
        break;
    case 0x1e03://位置信号下限
        ie_data.u32 = GSS_device.position_signal_lower;
        break;
    case 0x1e04://总绳长设置 米 单位0.1
        ie_data.u32 = GSS_device.total_length;
        break;
    case 0x1e05://均值偏差阈值 单位0.1
        ie_data.u32 = (uint32_t)(MEAN_DEVIATION_THRESHOLD * 10);
        break;
    case 0x1e06://单传感器阈值 单位0.1
        ie_data.u32 = (uint32_t)(SENSOR_DEVIATION_THRESHOLD * 10);
        break;
    case 0x1e07://方差异常阈值 单位0.1
        ie_data.u32 = (uint32_t)(VARIANCE_THRESHOLD * 10);
        break;
    case 0x1e08://趋势异常阈值 单位0.1
        ie_data.u32 = (uint32_t)(TREND_THRESHOLD * 10);
        break;
    case 0x1e09://综合判断阈值 单位0.1
        ie_data.u32 = (uint32_t)(DEFECT_SCORE_THRESHOLD * 10);
        break;
    case 0x1e0a://存储模式 0或1
        ie_data.u32 = 0; //TODO 读取存储模式
        break;
    case 0x1e0b://工作算法模式 0或1
        ie_data.u32 = 0; //TODO 读取算法模式
        break;
    case 0x1e0c://损1轻微损伤标定 单位1
        ie_data.u32 = GSS_device.Threshold_set1;
        break;
    case 0x1e0d://损1严重损伤标定 单位1
        ie_data.u32 = GSS_device.Threshold_set2;
        break;
    case 0x1e0f://损1波长1宽度 单位1
        ie_data.u32 = GSS_device.Threshold_set3;
        break;
    case 0x1e10://损1波长2宽度 单位1
        ie_data.u32 = GSS_device.Threshold_set4;
        break;
    case 0x1e11://一键标定
        ie_data.u32 = 0; //只写不读
        break;
    case 0x1e12://预留
        ie_data.u32 = 0;
        break;

    default:
        ie_data.u32 = 0;
        LOGT("warn:unknown addr:0x%04X for get\n", addr);
        break;
    }

    LOGT("get addr:0x%04X --value:%d\n", addr, ie_data.u32);

    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, (num - 23), cmd, pid); // 使用传入的正确PID

    uint16_t crc16 = bsp_crc16(txBuf + 23, (num - 23));

    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
    LOGT("send config get to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}

void APP_DTU_Cmd_Config_Get_Response(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data = {0};
    ie_data.u8_buf[3] = rxBuf[23];
    ie_data.u8_buf[2] = rxBuf[24];
    ie_data.u8_buf[1] = rxBuf[25];
    ie_data.u8_buf[0] = rxBuf[26];

    APP_DTU_Cmd_Config_Get(cmd, ie_data.u32, rxBuf[20]); // 传递正确的PID
}
/* 参数获取 */

/* IP设置 */
void APP_DTU_Cmd_Ip_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data = {0};
    app_dtu_ip_def dtu_ip = {0};

    ie_data.u8_buf[3] = rxBuf[28];
    ie_data.u8_buf[2] = rxBuf[29];
    ie_data.u8_buf[1] = rxBuf[30];
    ie_data.u8_buf[0] = rxBuf[31];
    dtu_ip.port = ie_data.i32;
    ie_data.u8_buf[3] = rxBuf[24];
    ie_data.u8_buf[2] = rxBuf[25];
    ie_data.u8_buf[1] = rxBuf[26];
    ie_data.u8_buf[0] = rxBuf[27];

    memcpy(dtu_ip.ip, rxBuf + 32, ie_data.i32);

    APP_RTU_AT_Ip_Set(rxBuf[23] - 1, dtu_ip.ip, dtu_ip.port, 1);
}

/* IP读取 */
void APP_DTU_Cmd_Ip_Get(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    if (rxBuf[23] == 0 || rxBuf[23] > 4)
    {
        LOG("err:mt get ip!\n");
        return;
    }
    uint8_t chl = rxBuf[23] - 1; //读取位置

    IEEE754 ie_data = {0};
    uint8_t txBuf[128] = {0};
    uint16_t num = 23;

    txBuf[num++] = 0;
    txBuf[num++] = rxBuf[23];

    ie_data.u32 = strlen(g_app_dtu_ip[chl].ip); //ip长度
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    ie_data.u32 = g_app_dtu_ip[chl].port; //端口号
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    memcpy(txBuf + num, g_app_dtu_ip[chl].ip, strlen(g_app_dtu_ip[chl].ip)); //ip
    num += strlen(g_app_dtu_ip[chl].ip);

    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, (num - 23), cmd, rxBuf[20]);

    uint16_t crc16 = bsp_crc16(txBuf + 23, (num - 23));

    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
}

void APP_DTU_Parse_Read(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    switch (cmd)
    {
    case DTU_CMD_DEVICE_HEARTBEAT:
        if (rxBuf[20] == 99)
        {
            g_dtu_cmd.cnt_status = USR_EOK;
            g_dtu_cmd.response_num = 0; //平台有应答 计数清0
            g_dtu_cmd.net_status = USR_STATE_ON;//dtu连接成功
            if (BSP_CONFIG_Show_Get() == 52)
            {
                LOGT("rmt:ht\n");
            }
        }
        else
        {
            LOGT("err:rmt ht[%d]\n", rxBuf[20]);
            LOG_HEX(APP_DTU_UART_BUF.rxBuf, APP_DTU_UART_BUF.rxLen)
        }
        break;
    case DTU_CMD_DEVICE_TIMESYNC:
        g_dtu_cmd.response_num = 0; //平台有应答 计数清0
        APP_DTU_TimeSync_Set(rxBuf, rxLen);
        g_dtu_cmd.cnt_status = USR_EOK;
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:time sync\n");
        }
        break;
    case DTU_CMD_DEVICE_POWE_ON_STATUS:
        g_dtu_cmd.cnt_status = USR_EOK;
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:po\n");
        }
        break;
    case DTU_CMD_SERVER_GATEWAY:
        g_dtu_cmd.response_num = 0; //平台有应答 计数清0
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:gateway[%d]\n", rxBuf[23]);
        }
        break;
    case DTU_CMD_DEVICE_SIM:
        g_app_rtu_sim.sta = 1;
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:sim info\n");
        }
        break;
    case DTU_CMD_DEVICE_GPS:
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:gps info\n");
        }
        break;

    case DTU_CMD_SERVER_GSS_DATA_UPLOAD:
        g_dtu_cmd.response_num = 0; //平台有应答 计数清0
        LOGT("rmt:rtd gss\n");
        break;
    case DTU_CMD_SERVER_GSS_ALARM_UPLOAD:
        g_dtu_cmd.response_num = 0; //平台有应答 计数清0
        LOGT("rmt:ent gss\n");
        break;
    case DTU_CMD_SERVER_GSS_CONFIG_INFO:
        g_dtu_cmd.response_num = 0; //平台有应答 计数清0
        LOGT("rmt:config info ack\n");
        break;

    default:
        break;
    }
}

void APP_DTU_Parse_Write(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    switch (cmd)
    {
    case DTU_CMD_SERVER_HEARTBEAT:
        APP_DTU_Response_Hearbeat(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:ht\n");
        }

        break;
    case DTU_CMD_SERVER_TIMESYNC:
        APP_DTU_TimeSync_Set(rxBuf, rxLen);
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:time sync\n");
        }
        break;
    case DTU_CMD_SERVER_DATA_UPLOAD_FREQUENCY://0x8002上报间隔设置
        APP_DTU_Cmd_Upload_Interval_Set(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:up interval\n");
        }
        break;

//    case DTU_CMD_SERVER_REMOTE_TRAN:
//        if(BSP_CONFIG_Show_Get()==52)
//        {
//            LOGT("wmt:tran\n");
//        }
//        APP_DTU_Cmd_Remote_Tran(rxBuf,rxLen);
//        break;

    case DTU_CMD_SERVER_DATA_UPLOAD_ADDRESS_SET://0x8003IP设置
        APP_DTU_Cmd_Ip_Set(cmd, rxBuf, rxLen);
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:ip set\n");
        }
        break;
    case DTU_CMD_SERVER_DATA_UPLOAD_ADDRESS_GET://0x8013IP读取
        APP_DTU_Cmd_Ip_Get(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:ip get\n");
        }
        break;
    case DTU_CMD_SERVER_SYSTEM_CONFIG_SET://0x8004系统配置设置
        APP_DTU_Cmd_Config_Set(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:cfg set\n");
        }
        break;
    case DTU_CMD_SERVER_SYSTEM_CONFIG_GET://0x8014系统配置读取
        APP_DTU_Cmd_Config_Get_Response(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOG("wmt:cfg get\n");
        }
        break;
    case DTU_CMD_SERVER_DEVICE_OTA://0x8005 OTA远程升级
        LOGT("ota:remote upgrade triggered\n");
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("ota:enter boot mode in 1s...\n");
        // 设置标志位，重启进入bootloader模式
        LOGT("ota:set boot flag to 0x%08X\n", OTA_FLAG_MAGIC_NUMBER);
        EEPROM_FLASH_WriteU32(APP_IAP_ADDR_STATUS_OTA, OTA_FLAG_MAGIC_NUMBER);
        LOGT("ota:rebooting to bootloader...\n");
        BSP_CONFIG_System_Reset(); // 重启设备
        break;
    default:
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_PARSE_FAIL, rxBuf, rxLen);
        break;
    }
}

//数据头打包
void APP_DTU_Remote_Head_Init(void)
{
    g_dtu_cmd.head[0] = '$';
    g_dtu_cmd.head[4] = ((g_dtu_remote.uint_sn / 1000000) % 10) +0x30;
    g_dtu_cmd.head[5] = ((g_dtu_remote.uint_sn / 100000) % 10) +0x30;
    g_dtu_cmd.head[6] = ((g_dtu_remote.uint_sn / 10000) % 10) +0x30;
    g_dtu_cmd.head[7] = ((g_dtu_remote.uint_sn / 1000) % 10) +0x30;
    g_dtu_cmd.head[8] = ((g_dtu_remote.uint_sn / 100) % 10) +0x30;
    g_dtu_cmd.head[9] = ((g_dtu_remote.uint_sn / 10) % 10) +0x30;
    g_dtu_cmd.head[10] = (g_dtu_remote.uint_sn % 10) +0x30;
    g_dtu_cmd.head[11] = ((g_app_cfg.did / 10000) % 10) +0x30;
    g_dtu_cmd.head[12] = ((g_app_cfg.did / 1000) % 10) +0x30;
    g_dtu_cmd.head[13] = ((g_app_cfg.did / 100) % 10) +0x30;
    g_dtu_cmd.head[14] = ((g_app_cfg.did / 10) % 10) +0x30;
    g_dtu_cmd.head[15] = (g_app_cfg.did % 10) +0x30;
    g_dtu_cmd.head[16] = g_dtu_remote.uint_ver;
    g_dtu_cmd.head[19] = g_dtu_remote.uint_model;
}

void APP_DTU_Gps_Check(uint8_t *rxBuf, uint16_t rxLen)
{
    if (strstr((char *)APP_DTU_UART_BUF.rxBuf, "$GNRMC") != NULL)
    {
        g_dtu_cmd.gps_status = bsp_gps_parse((char*)rxBuf, &g_gps_date);
        g_dtu_cmd.gps_enable = 1;
    }
    else if (strstr((char *)rxBuf, "$GNGGA") != NULL)
    {
    }
}

//return 1 数据头正确, 0 未识别
int APP_DTU_Remote_Check_Head(uint8_t *rxBuf, uint16_t rxLen)
{
    int ret = 0;
    uint16_t crc16 = bsp_crc16((uint8_t*)rxBuf, 21); //crc 校验 数据头
    if ((((uint8_t)crc16 & 0xFF) == rxBuf[21]) &&
            ((uint8_t)(crc16 >> 8) == rxBuf[22]))
    {
        ret = 1;
    }

    return ret;
}

//return 1 有数据未处理完, 0 没数据
int APP_DTU_Remote_Check_Body(void)
{
    int ret = 0;
    uint16_t len = g_dtu_rx.rxBuf[2];
    len = (len << 8) + g_dtu_rx.rxBuf[3];

    int deal = 0; //数据是否可用
    uint16_t dtu_remote_index = 23 + len;
    if (len > 0) //有数据 判断crc
    {
        uint16_t crc16 = bsp_crc16(g_dtu_rx.rxBuf + 23, len); //crc 校验 数据

        if ((((uint8_t)crc16 & 0xFF) == g_dtu_rx.rxBuf[len + 23]) &&
                ((uint8_t)(crc16 >> 8) == g_dtu_rx.rxBuf[len + 23 + 1]))
        {
            deal = 1;
            dtu_remote_index += 2; //增加2字节 校验长度
        }
        else
        {
            LOGT("err:body crc [0x%X]\n", crc16);
            if (g_dtu_rx.rxLen > 50)
            {
                LOG_HEX(g_dtu_rx.rxBuf, 50);
            }
            else
            {
                LOG_HEX(g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
            }
        }
    }
    else //没数据 判断是不是心跳
    {
        deal = 1;
    }

    if (deal > 0) //数据有效
    {
        g_dtu_cmd.cmd.u8_buf[1] = g_dtu_rx.rxBuf[17]; //缓存命令编号
        g_dtu_cmd.cmd.u8_buf[0] = g_dtu_rx.rxBuf[18];
        g_dtu_cmd.pid = g_dtu_rx.rxBuf[20];      //缓存包序列
        if (g_dtu_rx.rxBuf[1] == 'R')                       //判断为设备主动发送后平台的回复
        {
            APP_DTU_Parse_Read(g_dtu_cmd.cmd.u16, g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        }
        else if (g_dtu_rx.rxBuf[1] == 'W')          //判断为平台主动发送给设备的命令
        {
            APP_DTU_Parse_Write(g_dtu_cmd.cmd.u16, g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        }

        if (g_dtu_rx.rxLen > dtu_remote_index) //判断是否有剩余数据
        {
            g_dtu_rx.rxLen -= dtu_remote_index;
            memcpy(g_dtu_rx.rxBuf, g_dtu_rx.rxBuf + dtu_remote_index, g_dtu_rx.rxLen);
            ret = 1;
        }
    }

    return ret;
}

//return 0 未识别数据, 1 数据已解析
int APP_DTU_Remote_Check(void)
{
    int ret = 0;
    while (g_dtu_rx.rxLen > 22)
    {
        ret = APP_DTU_Remote_Check_Head(g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        if (ret == 1) //头正确
        {
            if (APP_DTU_Remote_Check_Body() == 0) //已处理完,退出
            {
                break;
            }
        }
        else //不可用包 退出
        {
            break;
        }
    }
    return ret;
}

void APP_DTU_Rec_Handle(void)
{
    if (BSP_UART_Rec_Read(APP_DTU_UART) == USR_EOK)
    {
        if (BSP_CONFIG_Show_Get() == 50)
        {
            LOGT("rtu rx[%d]: \n", APP_DTU_UART_BUF.rxLen);
            LOG_HEX(APP_DTU_UART_BUF.rxBuf, APP_DTU_UART_BUF.rxLen);
        }
        //多通道数据解析
        g_dtu_rx = APP_RTU_AT_Rx_Chl(APP_DTU_UART_BUF.rxBuf, APP_DTU_UART_BUF.rxLen);
        if (g_dtu_rx.rxLen > 0)
        {
            APP_DTU_Remote_Check();
        }
    }
}

extern bsp_dat_def Value_real[30];
extern uint16_t g_dwin_display_data[4];  // DWIN屏曲线显示专用数据（已处理传感器断开情况）

//发送实时数据/事件数据
void APP_DTU_Send_DTURealTimeRecord(void)
{
    uint8_t txBuf[1024] = {0};
    uint16_t num = 23;
    uint16_t pack_len = 1;
    IEEE754 aucData1 ;
    txBuf[num++] = 0; //默认值 第一版    //23
    txBuf[num++] = pack_len >> 8; //数据数量
    txBuf[num++ ] = pack_len; //数据数量

    for (u8 i = 0; i < pack_len; i++)
    {
        BSP_RTC_Get(&g_bsp_rtc);
        txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8; //年
        txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
        txBuf[num++] = g_bsp_rtc.month;
        txBuf[num++] = g_bsp_rtc.day;
        txBuf[num++] = g_bsp_rtc.hour;
        txBuf[num++] = g_bsp_rtc.minute;
        txBuf[num++] = g_bsp_rtc.second;

        aucData1.flt = (GSS_device.position_data_real < 0) ? -GSS_device.position_data_real : GSS_device.position_data_real;//位置实际值 厘米为单位
        txBuf[num++] = aucData1.u8_buf[3]; //num33
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.u32 = GSS_device.position_data_ad;//位置ad值
        txBuf[num++] = aucData1.u8_buf[3]; //num37
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 确保上报的速度为正值（方向信息由run_direction表示）
        aucData1.flt = (GSS_device.real_speed < 0) ? -GSS_device.real_speed : GSS_device.real_speed;//实时速度  200ms的变化量，还需要加K值
        txBuf[num++] = aucData1.u8_buf[3]; //num41
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 使用DWIN屏显示数据（已处理传感器断开情况）
        // 传感器正常时：g_dwin_display_data与GSS_device.hall_ad相同
        // 传感器断开时：g_dwin_display_data显示稳定波动（1900/2000/2100/2200±25）

        txBuf[num++] = g_dwin_display_data[0] >> 8; //电磁1的AD信号
        txBuf[num++] = g_dwin_display_data[0] & 0xff;

        aucData1.flt = ((float)g_dwin_display_data[0] * 3300.0f) / 4095.0f;//电磁1的电压信号

        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = g_dwin_display_data[1] >> 8; //电磁2的AD信号
        txBuf[num++] = g_dwin_display_data[1] & 0xff;

        aucData1.flt = ((float)g_dwin_display_data[1] * 3300.0f) / 4095.0f;//电磁2的电压信号

        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = g_dwin_display_data[2] >> 8; //电磁3的AD信号
        txBuf[num++] = g_dwin_display_data[2] & 0xff;

        aucData1.flt = ((float)g_dwin_display_data[2] * 3300.0f) / 4095.0f;//电磁3的电压信号

        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = g_dwin_display_data[3] >> 8; //电磁4的AD信号
        txBuf[num++] = g_dwin_display_data[3] & 0xff;

        aucData1.flt = ((float)g_dwin_display_data[3] * 3300.0f) / 4095.0f;//电磁4的电压信号

        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

//      aucData1.u32 = GSS_device.degree_of_damage;//钢丝绳损伤程度
        aucData1.flt = alarm_info_max.type;//最大损伤程度
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = GSS_device.alarm >> 8; //损伤程度报警状态
        txBuf[num++] = GSS_device.alarm & 0xff;

        txBuf[num++] = small_alarm_count % 2500 >> 8; //轻微损伤数量
        txBuf[num++] = small_alarm_count % 2500 & 0xff;

        txBuf[num++] = big_alarm_count % 2500 >> 8; //严重损伤数量
        txBuf[num++] = big_alarm_count % 2500 & 0xff;

        txBuf[num++] = GSS_device.total_length >> 8; //绳总长
        txBuf[num++] = GSS_device.total_length & 0xff;

        txBuf[num++] = damage_degree >> 8; //损伤度-健康度
        txBuf[num++] = damage_degree & 0xff;
        //最大损伤位置
        txBuf[num++] = GSS_device.max_damage_position >> 8; //最大损伤位置
        txBuf[num++] = GSS_device.max_damage_position & 0xff;

        // ===== 新增字段：实时阈值和滤波数据 =====
        // 字节62-65: 均值偏差实时阈值
        aucData1.flt = MEAN_DEVIATION_THRESHOLD;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节66-69: 单传感器实时阈值
        aucData1.flt = SENSOR_DEVIATION_THRESHOLD;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节70-73: 方差异常实时阈值
        aucData1.flt = VARIANCE_THRESHOLD;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节74-77: 趋势异常实时数据
        aucData1.flt = trend_deviation;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节78-81: 综合判断实时阈值
        aucData1.flt = DEFECT_SCORE_THRESHOLD;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节82-85: 滤波实时数据
        aucData1.flt = filtered_value11;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];
    }
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_SERVER_GSS_DATA_UPLOAD, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);

    LOGT("send real-time data to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}
//发送报警事件/事件数据
void APP_DTU_Send_DTUAlarm_sig(void)
{
    uint8_t txBuf[1024] = {0};
    uint16_t num = 23;
    IEEE754 aucData1 ;
//  txBuf[num++]=0; //默认值 第一版    //23
//  txBuf[num++]=pack_len>>8;//数据数量
//  txBuf[num++]=pack_len;//数据数量
    BSP_RTC_Get(&g_bsp_rtc);
    txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8; //年
    txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
    txBuf[num++] = g_bsp_rtc.month;
    txBuf[num++] = g_bsp_rtc.day;
    txBuf[num++] = g_bsp_rtc.hour;
    txBuf[num++] = g_bsp_rtc.minute;
    txBuf[num++] = g_bsp_rtc.second;
    LOGT("----------send alarm time: %04d-%02d-%02d %02d:%02d:%02d\n",
         2000 + g_bsp_rtc.year,
         g_bsp_rtc.month,
         g_bsp_rtc.day,
         g_bsp_rtc.hour,
         g_bsp_rtc.minute,
         g_bsp_rtc.second);

    txBuf[num++] = GSS_device_alarm_stat.alarm; //触发条件Byte :  1发现损坏报警2发现损坏预警
    aucData1.flt = GSS_device_alarm_stat.position_data_real;    //位置数据

    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    // 将编码器的值赋给position_data_ad
    aucData1.u32 = GSS_device_alarm_stat.position_data_ad = g_current_position; //位置AD值

    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

// 确保报警时上报的速度为正值（方向信息由run_direction表示）
    float alarm_speed = (GSS_device_alarm_stat.real_speed < 0) ? -GSS_device_alarm_stat.real_speed : GSS_device_alarm_stat.real_speed;
    aucData1.flt = alarm_speed / 50; //实时速度
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    txBuf[num++] = GSS_device_alarm_stat.hall_ad[0] >> 8; //AD1信号
    txBuf[num++] = GSS_device_alarm_stat.hall_ad[0] & 0xff;

    GSS_device_alarm_stat.hall_v[0] = (uint32_t)((GSS_device_alarm_stat.hall_ad[0] * 3300) / 4095);
    aucData1.flt = GSS_device_alarm_stat.hall_v[0];//ad1的电压值

    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    txBuf[num++] = GSS_device_alarm_stat.hall_ad[1] >> 8; //AD2信号
    txBuf[num++] = GSS_device_alarm_stat.hall_ad[1] & 0xff;

    GSS_device_alarm_stat.hall_v[1] = (uint32_t)((GSS_device_alarm_stat.hall_ad[1] * 3300) / 4095);
    aucData1.flt = GSS_device_alarm_stat.hall_v[1];//ad2的电压值

    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    txBuf[num++] = GSS_device_alarm_stat.hall_ad[2] >> 8; //AD3信号
    txBuf[num++] = GSS_device_alarm_stat.hall_ad[2] & 0xff;

    GSS_device_alarm_stat.hall_v[2] = (uint32_t)((GSS_device_alarm_stat.hall_ad[2] * 3300) / 4095);
    aucData1.flt = GSS_device_alarm_stat.hall_v[2];//ad3的电压值

    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    txBuf[num++] = GSS_device_alarm_stat.hall_ad[3] >> 8; //ad4的AD信号
    txBuf[num++] = GSS_device_alarm_stat.hall_ad[3] & 0xff;

    GSS_device_alarm_stat.hall_v[3] = (uint32_t)((GSS_device_alarm_stat.hall_ad[3] * 3300) / 4095);
    aucData1.flt = GSS_device_alarm_stat.hall_v[3];//ad4的电压值

    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    aucData1.flt = GSS_device_alarm_stat.degree_of_damage = GSS_device_alarm_stat.alarm; //钢丝绳损伤程度

    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    // 添加位置信息到DTU上报数据
//      aucData1.u32 = GSS_device_alarm_stat.position_data;//位置数据
//
//      txBuf[num++]=aucData1.u8_buf[3];
//      txBuf[num++]=aucData1.u8_buf[2];
//      txBuf[num++]=aucData1.u8_buf[1];
//      txBuf[num++]=aucData1.u8_buf[0];
//
//      txBuf[num++]=GSS_device_alarm_stat.position_valid;//位置数据有效性
//
//

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_SERVER_GSS_ALARM_UPLOAD, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);

    LOGT("send alarm data to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}
//发送配置信息0x1e04 - 钢丝绳探伤参数上传
void APP_DTU_Send_System_config(void)
{
    extern float MEAN_DEVIATION_THRESHOLD;
    extern float SENSOR_DEVIATION_THRESHOLD;
    extern float VARIANCE_THRESHOLD;
    extern float TREND_THRESHOLD;
    extern float DEFECT_SCORE_THRESHOLD;

    uint8_t txBuf[256] = {0};
    uint16_t num = 23;
    IEEE754 aucData;

    // 字节0-6: 时间
    BSP_RTC_Get(&g_bsp_rtc);
    txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8; //年 高字节
    txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF; //年 低字节
    txBuf[num++] = g_bsp_rtc.month;
    txBuf[num++] = g_bsp_rtc.day;
    txBuf[num++] = g_bsp_rtc.hour;
    txBuf[num++] = g_bsp_rtc.minute;
    txBuf[num++] = g_bsp_rtc.second;

    // 字节7: 版本
    txBuf[num++] = 0; //当前版本为0

    // 字节8-11: 位置量程上限 (UInt32)
    aucData.u32 = GSS_device.position_range_upper;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节12-15: 位置量程下限 (UInt32)
    aucData.u32 = GSS_device.position_range_lower;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节16-19: 位置信号上限 (UInt32)
    aucData.u32 = GSS_device.position_signal_upper;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节20-23: 位置信号下限 (UInt32)
    aucData.u32 = GSS_device.position_signal_lower;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节24-27: 位置实时信号 (Float32)
    aucData.flt = GSS_device_alarm_stat.position_data_real;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节28-31: 位置实时信号 (UInt32)
    aucData.u32 = GSS_device_alarm_stat.position_data_ad;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节32-33: 总绳长设置 (UInt16)
    txBuf[num++] = (GSS_device.total_length >> 8) & 0xFF;
    txBuf[num++] = GSS_device.total_length & 0xFF;

    // 字节34-37: 均值偏差阈值 (Float32)
    aucData.flt = MEAN_DEVIATION_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节38-41: 单传感器阈值 (Float32)
    aucData.flt = SENSOR_DEVIATION_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节42-45: 方差异常阈值 (Float32)
    aucData.flt = VARIANCE_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节46-49: 趋势异常阈值 (Float32)
    aucData.flt = TREND_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节50-53: 综合判断阈值 (Float32)
    aucData.flt = DEFECT_SCORE_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    // 字节54: 存储模式 (UInt8) - 暂时填0，后续可扩展
    txBuf[num++] = 0;

    // 字节55: 工作算法模式 (UInt8) - 暂时填0，后续可扩展
    txBuf[num++] = 0;

    // 字节56-57: 损1轻微损伤标定 (UInt16)
    txBuf[num++] = (GSS_device.Threshold_set1 >> 8) & 0xFF;
    txBuf[num++] = GSS_device.Threshold_set1 & 0xFF;

    // 字节58-59: 损1严重损伤标定 (UInt16)
    txBuf[num++] = (GSS_device.Threshold_set2 >> 8) & 0xFF;
    txBuf[num++] = GSS_device.Threshold_set2 & 0xFF;

    // 字节60-61: 损1波长1宽度 (UInt16)
    txBuf[num++] = (GSS_device.Threshold_set3 >> 8) & 0xFF;
    txBuf[num++] = GSS_device.Threshold_set3 & 0xFF;

    // 字节62-63: 损1波长2宽度 (UInt16)
    txBuf[num++] = (GSS_device.Threshold_set4 >> 8) & 0xFF;
    txBuf[num++] = GSS_device.Threshold_set4 & 0xFF;

    // 打包数据包头
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_SERVER_GSS_CONFIG_INFO, 0);

    // 计算CRC16
    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    // 发送数据
    APP_DTU_Send(txBuf, num);

    LOGT("send config info to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}

#define APP_DTU_WAITTIME 120   //单位 ms
static uint16_t  g_net_cnt_retry[DTU_CNT_STEP_MAX] =
{
    0, 30, 30, 30
};

//开机数据上传
uint8_t  APP_DTU_Connect_Remote_Handle(void)
{
    uint8_t ret = USR_ERROR;

    g_dtu_cmd.send_num++;
    g_dtu_cmd.send_cmd = DTU_CNT_STEP_MAX;

    if (g_dtu_cmd.cnt_status == USR_EOK) //有应答
    {
        g_dtu_cmd.cnt_status = USR_ERROR;
        g_dtu_cmd.send_num = 0;
        if (++g_dtu_cmd.response_cmd >= DTU_CNT_STEP_MAX) //指令发送完成
        {
            ret = USR_EOK;
        }
        else
        {
            g_dtu_cmd.send_cmd = g_dtu_cmd.response_cmd;
        }
    }
    else
    {
        if (g_net_cnt_retry[DTU_CNT_STEP0] == 0) //首次注册
        {
            g_net_cnt_retry[DTU_CNT_STEP0] = APP_DTU_WAITTIME; //上电等待
        }
        else if (APP_RTU_AT_Chk_Ready() == 2 && g_net_cnt_retry[DTU_CNT_STEP0] == APP_DTU_WAITTIME)
        {
            g_dtu_cmd.send_cmd = DTU_CNT_STEP0;
            g_net_cnt_retry[DTU_CNT_STEP0] = 30;
            g_dtu_cmd.send_num = 0;
        }
        else if (APP_RTU_AT_Chk_Ready() == 1 && g_dtu_cmd.send_num < 50)
        {
            return ret;
        }
        //指令间隔
        else  if (g_dtu_cmd.send_num % g_net_cnt_retry[g_dtu_cmd.response_cmd] == 0) //重试
        {
            g_dtu_cmd.send_cmd = g_dtu_cmd.response_cmd;

            if (g_dtu_cmd.response_cmd == (DTU_CNT_STEP_MAX - 1)) //最后一步
            {
                if (g_dtu_cmd.send_num / g_net_cnt_retry[g_dtu_cmd.response_cmd] > 2) //3次失败,继续下一步
                {
                    LOGT("err:startup networking!\n");
                    ret = USR_EOK;//跳出开机联网

                    g_dtu_cmd.response_cmd = 0;
                    g_dtu_cmd.cnt_status = USR_ERROR;
                    g_dtu_cmd.send_cmd = DTU_CNT_STEP_MAX;
                    g_dtu_cmd.send_num = 0;
                }
            }
            else if (g_dtu_cmd.response_cmd == DTU_CNT_STEP0) //第一步
            {
                if (g_net_cnt_retry[DTU_CNT_STEP0] == APP_DTU_WAITTIME) //上电后 改成3s
                {
                    g_net_cnt_retry[DTU_CNT_STEP0] = 30;
                    g_dtu_cmd.send_num = 0;
                }
                else if (g_dtu_cmd.send_num / g_net_cnt_retry[g_dtu_cmd.response_cmd] > 10) //>10次未成功,改成 30s/次
                {
                    g_net_cnt_retry[DTU_CNT_STEP0] = 300;
                    g_dtu_cmd.send_num = 0;
                }
            }
            else  //其它步骤
            {
                if (g_dtu_cmd.send_num / g_net_cnt_retry[g_dtu_cmd.response_cmd] > 2) //3次失败,继续下一步
                {
                    g_dtu_cmd.send_cmd = ++g_dtu_cmd.response_cmd;
                    g_dtu_cmd.send_num = 0;
                }
            }
        }
    }

    switch (g_dtu_cmd.send_cmd)
    {
    case DTU_CNT_STEP0: //心跳
        LOGT("cnt:step0 - send heartbeat [retry:%d]\n", g_dtu_cmd.send_num);
        APP_DTU_Send_Hearbeat();//发送心跳
        break;
    case DTU_CNT_STEP1: //时间同步
        LOGT("cnt:step1 - get server time [retry:%d]\n", g_dtu_cmd.send_num);
        APP_DTU_GetServerTime();//获取服务器时间
        break;
    case DTU_CNT_STEP2: //上报开机信息
        LOGT("cnt:step2 - send power on data [retry:%d]\n", g_dtu_cmd.send_num);
        APP_DTU_SendDTUPowerOnData();//开机状态
        break;
    case DTU_CNT_STEP3: //完成
        APP_DTU_Send_System_config();//上报配置信息
        g_dtu_cmd.cnt_status = USR_EOK;
        LOGT("cnt:step3 - connect complete!\n");
        break;
    default:
        break;
    }

    return ret;
}
void APP_DTU_Callback(void)
{
    if (g_dtu_cmd.power_on_status == USR_ERROR) //DTU上电检测
    {
        if (APP_DTU_Connect_Remote_Handle() == USR_EOK)
        {
            g_dtu_cmd.power_on_status = USR_EOK;
            LOGT("Connect to remote server complete!\n");
        }
    }
    else if (++g_dtu_cmd.response_num > APP_DTU_SIGNAL_TIMEOUT)   //dtu信号超时检测#define APP_DTU_SIGNAL_TIMEOUT  (1200)  //dtu连接超时判断时间:2分钟
    {
        g_dtu_cmd.net_status = USR_STATE_OFF;//dtu断网

        // 超时后立即重连策略：首次超时立即重连，之后每30秒重连一次
        if (g_dtu_cmd.response_num == APP_DTU_SIGNAL_TIMEOUT + 10) //首次超时，立即重连 10秒后重连
        {
            LOGT("warn:dtu timeout - no response for 2min, reconnecting now...\n");
         
            
            // 复位RTU相关标志，让程序重新走上电连接流程（类似第一次运行）
            g_app_rtu_at.poweron = 0;           // 复位上电标志
            g_app_rtu_at.poweron_chk = 0;       // 复位上电检测计数
            
            // 重置重试间隔参数（从快速重试开始）
            g_net_cnt_retry[DTU_CNT_STEP0] = 30;
            // 复位DTU状态
            APP_DTU_Status_Reset();             // 内部已将response_num清零
            g_dtu_cmd.power_on_status = USR_ERROR; // 重新进入开机联网流程
            
            LOGT("reconnect:tcp reset complete, start reconnection\n");
        }
        // 如果30秒后仍未连接成功，每30秒重试一次
        else if ((g_dtu_cmd.response_num - APP_DTU_SIGNAL_TIMEOUT) % 300 == 0)
        {
            uint16_t retry_times = (g_dtu_cmd.response_num - APP_DTU_SIGNAL_TIMEOUT) / 300;
            LOGT("warn:reconnect retry #%d - still offline\n", retry_times);
            // 重置重试间隔参数（确保每次重试都从快速重试开始）
            g_net_cnt_retry[DTU_CNT_STEP0] = 0;
            
            // 再次复位标志，重新尝试连接
            g_app_rtu_at.poweron = 0;
            g_app_rtu_at.poweron_chk = 0;
            APP_DTU_Status_Reset();
            g_dtu_cmd.power_on_status = USR_ERROR;
        }
    }
    else
    {
        g_dtu_cmd.net_status = USR_STATE_ON;//dtu连接成功

        // 检查是否有报警需要上报
        if (alarm_dtu_trig == 1 && (GSS_device_alarm_stat.alarm == 1 || GSS_device_alarm_stat.alarm == 2)) // 有报警时优先上报报警数据
        {
            alarm_dtu_trig = 0;
            APP_DTU_Send_DTUAlarm_sig();
            // 上报后清除报警标志，避免重复上报
        }
        //空闲实时数据
        else if (g_dtu_cmd.time_num % g_dtu_remote_cmd.normal_interval == 0) //每2s检测一次
        {
            static uint16_t normal_upload_counter = 0;  // 正常上传计数器

            if ((position_diff < -80) || (position_diff > 80))
            {
                // 情况1：position_diff超限，立即上传
                APP_DTU_Send_DTURealTimeRecord();
                normal_upload_counter = 0;  // 重置计数器
                LOGT("send:position exceed [diff:%d]\n", position_diff);
            }
            else
            {
                // 情况2：position_diff正常，每30秒上传一次
                normal_upload_counter++;
                if (normal_upload_counter >= 15)  // 15次 × 2秒 = 30秒
                {
                    APP_DTU_Send_DTURealTimeRecord();
                    normal_upload_counter = 0;
                    LOGT("send:normal interval [30s]\n");
                }
            }
        }


        if (g_dtu_cmd.time_num % 99 == 0) //心跳
        {
            APP_DTU_Send_Hearbeat();
        }
        else if (g_dtu_cmd.time_num % 20 == 0 && g_app_rtu_sim.sta == 0) //上电发送sim卡信息
        {
            APP_DTU_SendDTUSim();
        }
        else if (g_dtu_cmd.gps_enable > 0) //支持gps
        {
            if (g_dtu_cmd.time_num % g_dtu_cmd.gps_send_interval == 0) //发送gps信息
            {
                if (g_gps_date.position_valid == 0)
                {
                    g_dtu_cmd.gps_send_interval = 301; //无定位30s检测
                    LOGT("no gps!\n");
                }
                else
                {
                    g_dtu_cmd.gps_send_interval = 18990;//有定位30分检测
//                    APP_DTU_Send_Gps();
                }
            }
        }

        if (g_dtu_cmd.time_num % 36000 == 0) //每小时发设备参数
        {
            APP_DTU_Send_System_config();
        }
        else if (g_dtu_cmd.time_num % 3010 == 0) //每10分校时   3010
        {
            APP_DTU_GetServerTime();
        }

        g_dtu_cmd.time_num++;
    }
}

int  APP_DTU_Remote_Cnt_Sta_Get(void)
{
    return g_dtu_cmd.net_status;
}

void APP_DTU_Init(void)
{
    APP_RTU_AT_Init();//建立了 APP_RTU_AT_Config_Handle APP_RTU_AT_Config_Handle_Err   2个回调函数
    APP_DTU_Status_Reset();
    APP_DTU_Remote_Head_Init();//数据头打包
    BSP_DELAY_MS(2000);
    BSP_TIMER_Init(&g_timer_dtu, APP_DTU_Callback, TIMEOUT_100MS, TIMEOUT_100MS); //建立了APP_DTU_Callback回调函数
    BSP_TIMER_Start(&g_timer_dtu);
}

void APP_DTU_Handle(void)
{
    APP_DTU_Rec_Handle();
}

void BSP_DTU_Power_Reboot(void)
{
    // 1. 先拉低，断电
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(1000); // 建议断电1s以上，确保电容放电和模块掉电彻底

    // 2. 再拉高，重新上电
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(1000); // 上电后延时，确保模块完成自检
}

/*****END OF FILE****/

