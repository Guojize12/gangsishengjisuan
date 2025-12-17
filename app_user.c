#include "app_config.h"
#include "app_user.h"
#include <string.h>

#include "app_dtu.h" // 用于GSS_device_alarm_stat和上报

// FLASH存储地址定义已移到 app_user.h 中

void APP_USER_Init(void)
{
//    BSP_CONFIG_Init();
  
}

#define USE_DEFAULT


void APP_USER_Id_Get(void)
{
#ifdef USE_DEFAULT
    g_user_config.sys_config.eeprom_enable = EEPROM_FLASH_ReadU16(CFG_ADDR_EN);
    LOG("Read CFG_ADDR_EN: eeprom_enable= 0x%X (%d), expect=%d\n",
        g_user_config.sys_config.eeprom_enable,
        g_user_config.sys_config.eeprom_enable,
        EEPROM_ENABLE_VALUE);
    if(EEPROM_ENABLE_VALUE == g_user_config.sys_config.eeprom_enable)
	{
        g_user_config.sys_config.device_id = EEPROM_FLASH_ReadU32(CFG_ADDR_DID);
        LOG("device_id= %d\n",g_user_config.sys_config.device_id);
    }
    else//如果为其他说明程序没运行
    {
        LOG("device new! Setting eeprom_enable to 0, OTA check will be skipped!\n");
        g_user_config.sys_config.eeprom_enable = 0;
        return;//new device!
    }
	#if 0	
    if(APP_LOG_Read()<OTA_ERR_APP_MD5)
    {
        g_user_config.sys_config.reset_time=BSP_EEPROM_ReadU8(APP_IAP_ADDR_RESET);
        if(g_user_config.sys_config.reset_time > 10)
        {
            APP_LOG_Write(OTA_ERR_JUMP_APP);
        }
        else
        {
            EEPROM_FLASH_WriteU16(APP_IAP_ADDR_RESET,++g_user_config.sys_config.reset_time);
        }
    }
		#endif
#endif
}

