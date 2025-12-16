#include "eeprom_flash.h"
#include "bsp_config.h"

//#define USE_EE_LOG
/* Erase from PART_BASE_ADDRESS, size EEPROM_PART_SIZE */
int EE_ErasePart(int part)
{
    uint32_t erase_address = part == 0 ? PART0_BASE_ADDRESS : PART1_BASE_ADDRESS;
    uint32_t erase_size = part == 0 ? EEPROM_PART0_SIZE : EEPROM_PART1_SIZE;

    if (erase_address < PART0_BASE_ADDRESS || erase_address >= PART1_END_ADDRESS)
        return USR_ERROR;
#ifdef USE_EE_LOG
    LOG( "ee_e 0x%08X size %#X\r\n", erase_address, erase_size);
#endif

    BSP_FLASH_Erase_Pages(erase_address,erase_address+erase_size);
    return USR_EOK;
}

/* If the write operation is not a 32-bit atomic operation,
   Write the low 16-bit first, then write the high 16-bit */
int EE_ProgramWord(uint32_t Address, uint32_t Data)
{
    if (Address < PART0_BASE_ADDRESS || Address >= PART1_END_ADDRESS)
        return USR_ERROR;
#ifdef USE_EE_LOG
    LOG( "ee_w 0x%08X 0x%08X\r\n", Address, Data);
#endif
    return BSP_FLASH_Write(Address,&Data,4);
}
 
/*****END OF FILE****/
