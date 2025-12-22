/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_ERROR_HANDLER_H
#define __APP_ERROR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

// 错误类型定义
typedef enum
{
    ERROR_TYPE_NONE = 0,
    ERROR_TYPE_UART_TIMEOUT,
    ERROR_TYPE_ALARM_OVERFLOW,
    ERROR_TYPE_SYSTEM_HANG,
    ERROR_TYPE_MEMORY_OVERFLOW,
    ERROR_TYPE_MAX
} error_type_t;

// 错误处理函数
void APP_ERROR_Handler(error_type_t error_type, const char* error_msg);
void APP_ERROR_Reset_System(void);
void APP_ERROR_Init(void);

// 安全重启函数
void APP_ERROR_Safe_Restart(void);

// 错误统计
uint32_t APP_ERROR_Get_Count(error_type_t error_type);
void APP_ERROR_Clear_Count(error_type_t error_type);

#ifdef __cplusplus
}
#endif
#endif /*__APP_ERROR_HANDLER_H */

/*****END OF FILE****/