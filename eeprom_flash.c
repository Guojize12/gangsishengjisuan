#include "eeprom_flash.h"
#include "bsp_config.h"

static uint16_t EEPROM_data[EEPROM_NUM_MAX] = {0};
static int EEPROM_PART_USE = -1;

/* -------------- EE Private function -------------- */

static int EE_Format(int part)
{
    uint32_t i;
    uint32_t Part_Addr = (part == 0 ? PART0_BASE_ADDRESS : PART1_BASE_ADDRESS);
    uint32_t Address;

    if (EE_ErasePart(part) != USR_EOK)
        return USR_ERROR;

    LOG("EEPROM move data to partiton %d\r\n", part);
    Address = Part_Addr + 4;
    for (i = 0; i < EEPROM_NUM_MAX; i++)
    {
        if (EE_ProgramWord(Address, (i << 16) | EEPROM_data[i]) != USR_EOK)
            return USR_ERROR;

        Address += 4;
    }

    if (EE_ProgramWord(Part_Addr, PART_USED_MARK) != USR_EOK)
        return USR_ERROR;

    return USR_EOK;
}

static int EE_Init(void)
{
    if (EE_ReadWord(PART0_BASE_ADDRESS) == PART_USED_MARK)
    {
        EEPROM_PART_USE = 0;
    }
    else if (EE_ReadWord(PART1_BASE_ADDRESS) == PART_USED_MARK)
    {
        EEPROM_PART_USE = 1;
    }
    else
    {
        LOG("EE_Format -> EE_Init\n");
        if (EE_Format(0) != USR_EOK)
            return USR_ERROR;
        EEPROM_PART_USE = 0;
    }

    return USR_EOK;
}

static int EE_ReadVariable(uint32_t VirtAddress, uint16_t *Data)
{
    uint32_t PartStartAddress = (EEPROM_PART_USE == 0 ? PART0_BASE_ADDRESS : PART1_BASE_ADDRESS);
    uint32_t Address = (EEPROM_PART_USE == 0 ? PART0_END_ADDRESS : PART1_END_ADDRESS) - 4;
    uint32_t temp;

    VirtAddress <<= 16;
    for (; Address > PartStartAddress; Address -= 4)
    {
        temp = EE_ReadWord(Address);
        if ((temp & 0xffff0000) == VirtAddress)
        {
            *Data = temp & 0xffff;
            return USR_EOK;
        }
    }
    return USR_ERROR;
}

static int EE_WriteVariable(uint32_t VirtAddress, uint16_t Data)
{
    uint32_t PartStartAddress = (EEPROM_PART_USE == 0 ? PART0_BASE_ADDRESS : PART1_BASE_ADDRESS);
    uint32_t Address = (EEPROM_PART_USE == 0 ? PART0_END_ADDRESS : PART1_END_ADDRESS) - 4;

    if (EE_ReadWord(Address) != 0xffffffff)
    {
        LOG("EE_Format -> EE_WriteVariable\n");
        EEPROM_data[VirtAddress] = Data;
        if (EE_Format(EEPROM_PART_USE == 0 ? 1 : 0) != USR_EOK)
            return USR_ERROR;

        EEPROM_PART_USE = EEPROM_PART_USE == 0 ? 1 : 0;

#if PART0_BASE_ADDRESS != PART1_BASE_ADDRESS
        if (EE_ErasePart(EEPROM_PART_USE == 0 ? 1 : 0) != USR_EOK)
            return USR_ERROR;
#endif

        return USR_EOK;
    }
    else
    {
        for (Address -= 4; Address > PartStartAddress; Address -= 4)
        {
            if (EE_ReadWord(Address) != 0xffffffff)
                break;
        }

        if (EE_ProgramWord(Address + 4, (VirtAddress << 16) | Data) != USR_EOK)
            return USR_ERROR;

        EEPROM_data[VirtAddress] = Data;

        return USR_EOK;
    }
}


int EEPROM_FLASH_Init(void)
{
    if (EE_Init() != USR_EOK)
    {
        return USR_ERROR;
    }

    for (int i = 0; i < EEPROM_NUM_MAX; i++)
        EE_ReadVariable(i, &EEPROM_data[i]);

    return USR_EOK;
}

uint16_t EEPROM_FLASH_ReadU16(uint16_t Address)
{
    if (Address >= EEPROM_NUM_MAX)
        return 0;

    return EEPROM_data[Address];
}

int EEPROM_FLASH_WriteU16(uint16_t Address, uint16_t Data)
{
    if (EEPROM_PART_USE == -1 || Address >= EEPROM_NUM_MAX)
        return USR_ERROR;
   LOG("EEPROM_data[Address] =%d [%d\n",EEPROM_data[Address],Data);
    if (EEPROM_data[Address] == Data)
        return USR_EOK;

    if (EE_WriteVariable(Address, Data) != USR_EOK)
    {
        LOG("EEPROM FLASH write error\r\n");
        return USR_ERROR;
    }

    return USR_EOK;
}

int EEPROM_FLASH_Read_Buf(uint16_t Address, uint16_t *buf, uint16_t length)
{
    if (Address + length > EEPROM_NUM_MAX)
        return USR_ERROR;

    memcpy(buf, EEPROM_data + Address, length << 1);

    return USR_EOK;
}

int EEPROM_FLASH_Write_Buf(uint16_t Address, uint16_t *buf, uint16_t length)
{
    if (EEPROM_PART_USE == -1 || Address + length > EEPROM_NUM_MAX)
        return USR_ERROR;
    while (length--)
    {
        if (EEPROM_data[Address] != *buf)
        {
            if (EE_WriteVariable(Address, *buf) != USR_EOK)
            {
                LOG( "EEPROM FLASH write buf error\r\n");
                return USR_ERROR;
            }
        }

        buf++;
        Address++;
    }
    return USR_EOK;
}

uint32_t EEPROM_FLASH_ReadU32(uint16_t Address)
{
    if (Address >= EEPROM_NUM_MAX)
        return 0;

    uint32_t read = 0;
    EEPROM_FLASH_Read_Buf(Address,(uint16_t *)&read,2);

    return read;
}

int EEPROM_FLASH_WriteU32(uint16_t Address, uint32_t Data)
{
    if (Address >= EEPROM_NUM_MAX)
        return 0;

    return EEPROM_FLASH_Write_Buf(Address,(uint16_t *)&Data,2);
}

int EEPROM_FLASH_Format(void)
{
    LOG("EE_Format -> EEPROM_Format\n");
    memset(EEPROM_data, 0, sizeof(EEPROM_data));
    if (EE_Format(0) != USR_EOK)
    {
        LOG( "EEPROM FLASH Format error\r\n");
        return USR_ERROR;
    }

    EEPROM_PART_USE = 0;

    if (EE_ErasePart(1) != USR_EOK)
        return USR_ERROR;

    return USR_EOK;
}

/*****END OF FILE****/

