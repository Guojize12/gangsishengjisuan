#ifndef _EEPROM_FLASH_H_
#define _EEPROM_FLASH_H_

#include "bsp_flash.h"

/* eeprom storage data num max, 1-65535, unit halfword.
Must be smaller than (EEPROM_PART_SIZE/4)-2 */
#define EEPROM_NUM_MAX          322		//*2byte

/* Define the size of the sectors to be used */
#define FLASH_SECTOR_SIZE       BSP_FLASH_PAGE_SIZE

/* EEPROM Use two partitions, each partition size is
an integer multiple of the erased page */
#define EEPROM_PART0_SIZE       (FLASH_SECTOR_SIZE*2)
#define EEPROM_PART1_SIZE       (EEPROM_PART0_SIZE)

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS    (BSP_FLASH_ADDR_END-FLASH_SECTOR_SIZE*4)
#define EEPROM_END_ADDRESS      (EEPROM_START_ADDRESS + EEPROM_PART0_SIZE + EEPROM_PART1_SIZE)

/* Pages 0 and 1 base and end addresses */
#define PART0_BASE_ADDRESS      (EEPROM_START_ADDRESS)
#define PART0_END_ADDRESS       ((PART0_BASE_ADDRESS + EEPROM_PART0_SIZE))

#define PART1_BASE_ADDRESS      (PART0_END_ADDRESS)
#define PART1_END_ADDRESS       ((PART1_BASE_ADDRESS + EEPROM_PART1_SIZE))

/* PAGE is marked to record data */
#define PART_USED_MARK          (0xEAE5D135 + EEPROM_NUM_MAX)	//Different number, use new data 

/* flash read program erase callback function */
int EE_ErasePart(int part);
int EE_ProgramWord(uint32_t Address, uint32_t Data);
#define EE_ReadWord(Addr) (*(volatile uint32_t*)Addr)

int EEPROM_FLASH_Init(void);
int EEPROM_FLASH_Format(void);
uint16_t EEPROM_FLASH_ReadU16(uint16_t Address);
uint32_t EEPROM_FLASH_ReadU32(uint16_t Address);
int EEPROM_FLASH_WriteU16(uint16_t Address, uint16_t Data);
int EEPROM_FLASH_WriteU32(uint16_t Address, uint32_t Data);
int EEPROM_FLASH_Read_Buf(uint16_t Address, uint16_t *buf, uint16_t length);
int EEPROM_FLASH_Write_Buf(uint16_t Address, uint16_t *buf, uint16_t length);

#endif /* _EEPROM_FLASH_H_ */

/*****END OF FILE****/
