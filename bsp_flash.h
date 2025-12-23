#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stdbool.h>

#define BSP_FLASH_ADDR_END   0x08040000           //flash 结束地址
#define BSP_FLASH_PAGE_SIZE  FLASH_PAGE_SIZE       //页大小 

void BSP_FLASH_Erase_Pages(uint32_t addr_start, uint32_t addr_end);
int BSP_FLASH_Write(uint32_t address, uint32_t *pData, uint32_t len_bytes);
uint32_t BSP_FLASH_Read(uint32_t address, uint32_t *pData, uint32_t len);

#endif
