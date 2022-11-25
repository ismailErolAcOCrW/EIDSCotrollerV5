
/** Put this in the src folder **/
#include "main.h"
#include "flash.h"

void Flash_Unlock()
{
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
	HAL_Delay(500);
}

void Flash_Erase()
{
	FLASH_Erase_Sector(FLASH_SECTOR_4, VOLTAGE_RANGE_3);
	HAL_Delay(500);
}

void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data)
{
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Flash_Address, Flash_Data);
}

void Flash_Lock()
{
	HAL_FLASH_Lock();
	HAL_Delay(500);
}

uint32_t Flash_Read(uint32_t Flash_Address)
{
	uint32_t Flash_Data;
	Flash_Data = *(uint32_t*) Flash_Address;
	HAL_Delay(1);
	return Flash_Data;
}
