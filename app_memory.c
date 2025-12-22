#include "app_config.h"
#include "app_memory.h"

void APP_MEMORY_Write_BUF16(uint16_t addr, uint16_t *u16Buf, uint16_t u16Len)
{
    EEPROM_FLASH_Write_Buf(addr, u16Buf, u16Len);
}

int APP_MEMORY_Read_BUF16(uint16_t addr, uint16_t *u16Buf, uint16_t u16Len)
{
    return EEPROM_FLASH_Read_Buf(addr, u16Buf, u16Len);
}

void APP_MEMORY_Write_U16(uint16_t addr, uint16_t u16Date)
{
    EEPROM_FLASH_WriteU16(addr, u16Date);
}

uint16_t APP_MEMORY_Read_U16(uint16_t addr)
{
    return EEPROM_FLASH_ReadU16(addr);
}

void APP_MEMORY_Write_U32(uint16_t addr, uint32_t u32Date)
{
    EEPROM_FLASH_WriteU32(addr, u32Date);
}

uint32_t APP_MEMORY_Read_U32(uint16_t addr)
{
    return EEPROM_FLASH_ReadU32(addr);
}


/*****END OF FILE****/
