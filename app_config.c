#include "app_config.h"

app_cfg_def  g_app_cfg  =
{
    .did = 123456,                          //默认设备ID
    .dtp = 1,
    .dnm = 1,
    .com = 0,

};

void APP_CONFIG_Did_Set(uint32_t did)
{
    g_app_cfg.did = did;
    APP_MEMORY_Write_U32(CFG_ADDR_DID, g_app_cfg.did);
}

uint32_t APP_CONFIG_Did_Get(void)
{
    g_app_cfg.did = APP_MEMORY_Read_U32(CFG_ADDR_DID);
    return g_app_cfg.did;
}

void APP_CONFIG_Dtp_Set(uint32_t dtp)
{
    g_app_cfg.dtp = dtp;
    APP_MEMORY_Write_U16(CFG_ADDR_DTP, g_app_cfg.dtp);
}

uint32_t APP_CONFIG_Dtp_Get(void)
{
    g_app_cfg.dtp = APP_MEMORY_Read_U16(CFG_ADDR_DTP);
    return g_app_cfg.dtp;
}

void APP_CONFIG_DNM_Set(uint32_t dnm)
{
    g_app_cfg.dnm = dnm;
    APP_MEMORY_Write_U16(CFG_ADDR_DNM, g_app_cfg.dnm);
}

uint32_t APP_CONFIG_DNM_Get(void)
{
    g_app_cfg.dnm = APP_MEMORY_Read_U16(CFG_ADDR_DNM);
    return g_app_cfg.dnm;
}

void APP_CONFIG_COM_Set(uint32_t com)
{
    g_app_cfg.com = com;
    APP_MEMORY_Write_U16(CFG_ADDR_COM, g_app_cfg.com);
}

uint32_t APP_CONFIG_COM_Get(void)
{
    g_app_cfg.com = APP_MEMORY_Read_U16(CFG_ADDR_COM);
    return g_app_cfg.com;
}

//默认配置
void APP_CONFIG_Reset(uint16_t id)
{
    APP_CONFIG_COM_Set(0);
    APP_CONFIG_Did_Set(id);
    APP_CONFIG_Dtp_Set(1);
    APP_CONFIG_DNM_Set(0);
    LOG("factory reset!\n");
}

//读取所有配置
void APP_CONFIG_Read(void)
{
    APP_CONFIG_Did_Get();
    APP_CONFIG_Dtp_Get();
    APP_CONFIG_DNM_Get();
    APP_CONFIG_COM_Get();
}

void APP_CONFIG_Init(void)
{
    if (APP_MEMORY_Read_U16(CFG_ADDR_EN) != 9527)
    {
        APP_MEMORY_Write_U16(CFG_ADDR_EN, 9527);
        APP_CONFIG_Reset(12345);
    }
    else
    {
        APP_CONFIG_Read();
    }
}

/*****END OF FILE****/

