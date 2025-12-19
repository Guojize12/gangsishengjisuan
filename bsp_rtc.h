#ifndef __BSP_RTC_H
#define __BSP_RTC_H

#ifdef __cplusplus
extern "C" {
#endif
 
#include "stdint.h"
#include "rtc.h"

typedef struct
{
    /** 年 */
    uint8_t year;
    /** 月 */
    uint8_t month;
    /** 日 */
    uint8_t day;
    /** 周 */
    uint8_t week;
    /** 时 */
    uint8_t hour;
    /** 分 */
    uint8_t minute;
    /** 秒 */
    uint8_t second;
    uint8_t utc;
} bsp_rtc_def;



extern bsp_rtc_def g_bsp_rtc;

void BSP_RTC_Set(bsp_rtc_def time);
void BSP_RTC_Get(bsp_rtc_def *time); 
void BSP_RTC_Init(void); 

#ifdef __cplusplus
}
#endif
#endif  /* __BSP_RTC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
