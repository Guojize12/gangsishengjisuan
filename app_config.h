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
#include <string.h>
#include <stdint.h>

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

/* 新：集中存储在 EEPROM/FLASH 的配置结构相关定义 */
#define APP_CFG_MAGIC          0xA5C3F00DU
#define APP_CFG_STORE_VERSION  1U   // 配置结构版本（用于以后兼容迁移）
#define APP_CFG_STORE_ADDR     CFG_ADDR_KEY  // 从 app_memory.h 中定义的地址起始位置

/* API: 初始化/重置/读写（向后兼容旧接口） */
void APP_CONFIG_Init(void);
void APP_CONFIG_Reset(uint16_t id);

/* 整体保存/加载接口（新） */
int  APP_CONFIG_SaveAll(void);   // 将 g_app_cfg 序列化并写入 EEPROM（含魔数+校验）
int  APP_CONFIG_LoadAll(void);   // 从 EEPROM 读取并校验，失败返回非0
int  APP_CONFIG_IsValid(void);   // 检查 EEPROM 中集中存储的配置是否合法

/* 保留并兼容原有按字段的接口（底层现在会同步集中存储） */
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
