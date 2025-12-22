#ifndef PARAMS_INIT_H
#define PARAMS_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 原有导出接口（保留） */
void loadini(void);
void InitCoreData(void);
void LoadParamsFromFlash(void);
void WriteDefaultParamsToFlash(void);

/* 新增：集中存储接口 */
int  PARAMS_SaveAll(void);    /* 将内存参数整体写入Flash（有变更才写） */
int  PARAMS_LoadAll(void);    /* 从Flash整体读取并校验，失败返回非0 */
int  PARAMS_IsValid(void);    /* 仅校验Flash中参数是否有效（魔数+checksum） */
void PARAMS_ResetDefaults(void); /* 恢复默认并写入Flash */

/* FLASH 存储起始地址（半字地址，与 app_user.h 中其他 FLASH_xxx 一致的风格） */
#define PARAMS_FLASH_ADDR    (2*128) /* halfword index 256 — 可根据实际Flash布局调整 */

/* 版本/魔数定义 */
#define PARAMS_FLASH_MAGIC    0x50415253UL /* 'PARS' */
#define PARAMS_FLASH_VERSION  1U

#ifdef __cplusplus
}
#endif
#endif
