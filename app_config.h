#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"
#include "app_version.h"
#include "app_main.h"
#include "app_user.h"
#include "app_memory.h"
#include "app_rtu_at.h"
#include "app_dtu.h"
#include "app_dwin.h"
#include "app_dwin_sub.h"
#include "app_kalman.h"

/** 系统参数*/
typedef struct
{
    uint32_t  did; //设备ID
    uint16_t  dtp; //设备类型
    uint16_t  dnm; //设备号
    uint16_t  com; //通信类型
    char  model[20];
} app_cfg_def;
extern app_cfg_def  g_app_cfg;

void APP_CONFIG_Init(void);
void APP_CONFIG_Reset(uint16_t id);

void APP_CONFIG_Did_Set(uint32_t did);
uint32_t APP_CONFIG_Did_Get(void);
void APP_CONFIG_Dtp_Set(uint32_t dtp);
uint32_t APP_CONFIG_Dtp_Get(void);
void APP_CONFIG_DNM_Set(uint32_t dnm);
uint32_t APP_CONFIG_DNM_Get(void);

void APP_CONFIG_COM_Set(uint32_t com);
uint32_t APP_CONFIG_COM_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */

/*****END OF FILE****/

