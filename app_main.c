#include "app_config.h"
#include "app_main.h"
#include "bsp_uart.h"
#include "app_user.h"

static Timer g_timer_iwdg = {0};
/**
  * @brief  喂看门狗
  * @param  None.
  * @retval None.
  */
static void APP_MAIN_Callback_Iwdg(void)
{
    BSP_IWDG_Refresh();

//      uint8_t txbuf[10]={1,2,3,4,5,6,7,8,9,0};
//      BSP_UART_Transmit(BSP_UART3,txbuf,10);
//      for(int i=40;i<80;i++)
//      {
//      APP_MEMORY_Write_U32(i*4,i);
//      }
//      for(int i=40;i<80;i++)
//      {
//      APP_MEMORY_Write_U16(i*2,i);
//      }
}

static void APP_MAIN_Tmr_Init(void)
{
    BSP_TIMER_Init(&g_timer_iwdg, APP_MAIN_Callback_Iwdg, TIMEOUT_2S, TIMEOUT_2S);
    BSP_TIMER_Start(&g_timer_iwdg);
}

/**
  * @brief  功能初始化
  * @param  None.
  * @retval None.
  */
void APP_MAIN_Init(void)
{
    APP_MAIN_Tmr_Init();

    /* add app init*/
    APP_CONFIG_Init();
    APP_DTU_Init();//建立1  APP_RTU_AT_Config_Handle 建立2 APP_RTU_AT_Config_Handle_Err  //建立3   APP_DTU_Callback回调函数

    /* add app init end */
    APP_VERSION_Print();
    APP_USER_Init();//建立1 APP_USER_button_Loop  建立2 Modbus_Send_ReadCmd
    APP_DWIN_Init();//建立APP_DWIN_SUB_Display_Handle
//  APP_RTU_AT_Init();
}
/**
  * @brief  主循环
  * @param  None.
  * @retval None.
  */
// 全局变量用于监控主循环健康状态
uint32_t g_main_loop_heartbeat = 0;

void APP_MAIN_Handle(void)
{
    APP_MAIN_Init();

    while (1)
    {
        /* add bsp handle*/
        APP_USER_ADC_Loop();
        /** UART*/
        BSP_UART_Handle();
        /** TIMER*/
        BSP_TIMER_Handle();
        /* add bsp handle end */

        /* add app handle*/
        APP_DTU_Handle();
        Modbus_Rec_Handle();  // Modbus接收处理
        APP_DWIN_Rec_Handle();//迪文屏接收消息处理
//        APP_USER_Rx_Handle();
        /* add app handle end */

        /* add log handle*/
        /** RTT*/
        USER_LOG_Input_Handle();
        /* add log end */

        // 给系统一些休息时间，避免过度占用CPU
        HAL_Delay(1);
    }
}

/*****END OF FILE****/

