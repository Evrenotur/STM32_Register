/*
 * Eprom.c
 *
 *  Created on: Mar 12, 2023
 *      Author: Software
 */


#include "Eprom.h"

uint16_t Read_Flash(uint32_t  adr)
{
  uint16_t * Pntr = (uint16_t *)adr;
  return(*Pntr);
}



void Write_Flash( uint32_t address ,  uint16_t* data)
{
    FLASH_EraseInitTypeDef EraseStruct;
    uint32_t PageError;

    EraseStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseStruct.PageAddress = address;
    EraseStruct.NbPages	    = 1;

    HAL_FLASH_Unlock();                            /* Yazma işlemi için flash ın kilidi kaldırıldı */
    HAL_FLASHEx_Erase(&EraseStruct,&PageError);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,address,*data);
    HAL_FLASH_Lock();
}
