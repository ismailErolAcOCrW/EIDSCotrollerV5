#include "stm32f4xx_hal.h"

void Flash_Unlock();
void Flash_Erase();
void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data);

void Flash_Lock();
uint32_t 	Flash_Read(uint32_t Flash_Address);
