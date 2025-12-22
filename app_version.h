#ifndef __APP_VERSION_H
#define __APP_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

void APP_VERSION_Print(void);

void APP_VERSION_Get_Hard(uint8_t *version_hard);
void APP_VERSION_Get_Soft(uint8_t *version_soft);
char *APP_VERSION_Get_Model(void);

uint32_t APP_VERSION_Get_Soft_U32(void);
uint32_t APP_VERSION_Get_Hard_U32(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_VERSION_H */

/*****END OF FILE****/
