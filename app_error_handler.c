#include "app_error_handler.h"
#include "app_config.h"

// 错误计数器
static uint32_t error_counts[ERROR_TYPE_MAX] = {0};
static uint32_t total_error_count = 0;
static uint32_t last_error_time = 0;

/**
 * @brief 错误处理函数
 * @param error_type 错误类型
 * @param error_msg 错误消息
 */
void APP_ERROR_Handler(error_type_t error_type, const char* error_msg)
{
    uint32_t current_time = HAL_GetTick();

    // 更新错误计数
    if (error_type < ERROR_TYPE_MAX)
    {
        error_counts[error_type]++;
        total_error_count++;
    }

    // 记录错误时间
    last_error_time = current_time;

    // 根据错误类型采取不同的处理措施
    switch (error_type)
    {
    case ERROR_TYPE_UART_TIMEOUT:
        LOG("ERROR: UART timeout - %s\r\n", error_msg ? error_msg : "Unknown");
        // UART超时不需要重启，只记录日志
        break;

    case ERROR_TYPE_ALARM_OVERFLOW:
        LOG("ERROR: Alarm overflow - %s\r\n", error_msg ? error_msg : "Unknown");
        // 报警溢出，清空报警缓冲区
        // 这里可以添加清理报警缓冲区的代码
        break;

    case ERROR_TYPE_SYSTEM_HANG:
        LOG("CRITICAL: System hang detected - %s\r\n", error_msg ? error_msg : "Unknown");
        // 系统挂起，执行安全重启
        APP_ERROR_Safe_Restart();
        break;

    case ERROR_TYPE_MEMORY_OVERFLOW:
        LOG("CRITICAL: Memory overflow - %s\r\n", error_msg ? error_msg : "Unknown");
        // 内存溢出，执行安全重启
        APP_ERROR_Safe_Restart();
        break;

    default:
        LOG("ERROR: Unknown error type %d - %s\r\n", error_type, error_msg ? error_msg : "Unknown");
        break;
    }

    // 如果错误频率过高，执行保护措施
    if (total_error_count > 10 && (current_time - last_error_time) < 10000)
    {
        LOG("CRITICAL: Too many errors in short time, executing safe restart\r\n");
        APP_ERROR_Safe_Restart();
    }
}

/**
 * @brief 安全重启系统
 */
void APP_ERROR_Safe_Restart(void)
{
    LOG("Executing safe system restart...\r\n");

    // 保存重要数据到Flash（如果需要）
    // 这里可以添加保存关键数据的代码

    // 关闭所有外设
    __disable_irq();

    // 等待一小段时间确保日志输出完成
    for (volatile int i = 0; i < 1000000; i++);

    // 执行软件复位
    NVIC_SystemReset();
}

/**
 * @brief 系统重启（简单版本）
 */
void APP_ERROR_Reset_System(void)
{
    LOG("System reset requested\r\n");
    NVIC_SystemReset();
}

/**
 * @brief 初始化错误处理模块
 */
void APP_ERROR_Init(void)
{
    // 清零所有错误计数
    for (int i = 0; i < ERROR_TYPE_MAX; i++)
    {
        error_counts[i] = 0;
    }
    total_error_count = 0;
    last_error_time = 0;

    LOG("Error handler initialized\r\n");
}

/**
 * @brief 获取特定错误类型的计数
 * @param error_type 错误类型
 * @return 错误计数
 */
uint32_t APP_ERROR_Get_Count(error_type_t error_type)
{
    if (error_type < ERROR_TYPE_MAX)
    {
        return error_counts[error_type];
    }
    return 0;
}

/**
 * @brief 清除特定错误类型的计数
 * @param error_type 错误类型
 */
void APP_ERROR_Clear_Count(error_type_t error_type)
{
    if (error_type < ERROR_TYPE_MAX)
    {
        error_counts[error_type] = 0;
    }
}

/*****END OF FILE****/