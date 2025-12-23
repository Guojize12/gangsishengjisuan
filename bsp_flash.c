#include "bsp_flash.h"
#include "bsp_config.h"
 
static void BSP_STMFLASH_Unlock(void)
{
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
    {
        HAL_FLASH_Unlock();
    }
}
 
static void BSP_STMFLASH_Lock(void)
{
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) == RESET)
    {
        HAL_FLASH_Lock();
    }
} 

void BSP_FLASH_Erase_Pages(uint32_t addr_start, uint32_t addr_end)
{
    int page_num = (addr_end-addr_start)/BSP_FLASH_PAGE_SIZE;
    if((addr_end-addr_start)%BSP_FLASH_PAGE_SIZE>0)
    {
        page_num++;
    }
		
    uint32_t PAGEError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = addr_start;
    EraseInitStruct.NbPages     = page_num;
		
		BSP_STMFLASH_Unlock();
    __disable_irq();
    HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
    __enable_irq();
		BSP_STMFLASH_Lock();
}

int BSP_FLASH_Write(uint32_t address, uint32_t *pData, uint32_t len_bytes)
{
    uint32_t i;
		BSP_STMFLASH_Unlock();
    __disable_irq();
    for(i=0; i<len_bytes; i+=4)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+i, *(pData + (i/4))) != HAL_OK)
				{
						return -1;
				}
    }
    __enable_irq();
		BSP_STMFLASH_Lock();
 
		return 0;
}

uint32_t BSP_FLASH_Read(uint32_t address, uint32_t *pData, uint32_t len)
{
    uint32_t i;
    for (i=0; i<len; i+=4)
    {
        *pData++ = *(__IO uint32_t*)(address + i);
    }
    return len;
}

/*****END OF FILE****/
