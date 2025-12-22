#ifndef __APP_MEMORY_H
#define __APP_MEMORY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

//每个地址映射 2字节
enum
{
    CFG_ADDR_EN = 0, //设备使用标志 2B
    CFG_ADDR_DID = 1, //设备ID  4B
    CFG_ADDR_DTP = 3, //设备类型 2B
    CFG_ADDR_DNM = 4, //设备号 2B
    CFG_ADDR_COM = 5, //通信类型  2B
    CFG_ADDR_KEY = 6, //密钥地址 //16B

};

//addr   每个地址存 2字节
//u16Buf 16位数据
//u16Len 16位数据的个数

void APP_MEMORY_Write_BUF16(uint16_t addr, uint16_t *u16Buf, uint16_t u16Len);
int APP_MEMORY_Read_BUF16(uint16_t addr, uint16_t *u16Buf, uint16_t u16Len);
void APP_MEMORY_Write_U16(uint16_t addr, uint16_t u16Date);
uint16_t APP_MEMORY_Read_U16(uint16_t addr);

void APP_MEMORY_Write_U32(uint16_t addr, uint32_t u32Date);
uint32_t APP_MEMORY_Read_U32(uint16_t addr);

#ifdef __cplusplus
}
#endif
#endif  /* __APP_MEMORY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
