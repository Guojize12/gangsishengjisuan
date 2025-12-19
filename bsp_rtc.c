#include "bsp_config.h"
#include "bsp_rtc.h"
#include "time.h"

bsp_rtc_def g_bsp_rtc =
 {
	.year = 22,
	.month = 1,
	.day = 1,
	 .hour = 13,
	 .minute =13,
	 .second =22,
 
 };

void BSP_RTC_Get(bsp_rtc_def *time)
{
    RTC_TimeTypeDef TIME= {0};
    RTC_DateTypeDef DATE= {0};

    HAL_RTC_GetTime(&hrtc,&TIME,RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc,&DATE,RTC_FORMAT_BIN);

    time->year = DATE.Year;
    time->month = DATE.Month;
    time->day = DATE.Date;
    time->hour = TIME.Hours;
    time->minute = TIME.Minutes;
    time->second = TIME.Seconds;
#if 0
    struct tm info= {0};
    info.tm_year = time->year - 1900;
    info.tm_mon =time->month - 1;
    info.tm_mday = time->day;
    info.tm_hour =time->hour;
    info.tm_min = time->minute;
    info.tm_sec = time->second;

    time->utc = mktime(&info) - 28800;
	#endif
//    LOG("Time Get:%d-%d-%d %d:%d:%d\n",time->year,time->month,time->day,time->hour,
//                                                                time->minute,  
//                                                                 time->second  );
	}
void BSP_RTC_Set(bsp_rtc_def time)
{
    RTC_TimeTypeDef TIME= {0};
    RTC_DateTypeDef DATE= {0};
    DATE.WeekDay=1;
    DATE.Year=time.year;
    DATE.Month=time.month;
    DATE.Date=time.day;

    TIME.Hours=time.hour;
    TIME.Minutes=time.minute;
    TIME.Seconds=time.second;

    if (HAL_RTC_SetTime(&hrtc,&TIME,RTC_FORMAT_BIN) != HAL_OK)
    {
        LOG("SET TIME ERR!\n");
    }

    if (HAL_RTC_SetDate(&hrtc,&DATE,RTC_FORMAT_BIN) != HAL_OK)
    {
        LOG("SET DATE ERR!\n");
    }

    LOG("Time Set:%d-%d-%d  %d:%d:%d\n",DATE.Year+2000,DATE.Month,DATE.Date,TIME.Hours,TIME.Minutes,TIME.Seconds);
    BSP_RTC_Get(&g_bsp_rtc);
    LOG("Time Get:%d-%d-%d %d:%d:%d\n",time.year+2000,time.month,time.day,time.hour,
                                                                time.minute,  
                                                                 time.second  );
}

void BSP_RTC_Init(void)
{
    MX_RTC_Init();
    
    // 上电时初始化RTC时间为2025年8月19日
    bsp_rtc_def init_time = {
        .year = 25,      // 2025年
        .month = 8,      // 8月
        .day = 19,       // 19日
        .hour = 0,       // 0时
        .minute = 0,     // 0分
        .second = 0      // 0秒
    };
    BSP_RTC_Set(init_time);
}

/*****END OF FILE****/

