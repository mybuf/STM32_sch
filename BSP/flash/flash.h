#ifndef __FLASH_H__
#define __FLASH_H__


#include "stm32f10x.h"
#include "stm32f10x_flash.h"




#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
#define FLASH_PAGE_SIZE ((uint16_t)0x0800)

#else
#define FLASH_PAGE_SIZE ((uint16_t)0x0400)
#endif

#define FLASH_BASE_ADDR 0x08010000
#define MCU_BASE_ADDR   0x08019000
#define FLASH_FPGA_DATA_LEN	22

int Flash_Read(uint32_t iAddress, uint16_t *buf, int32_t iNbrToRead) ;
int Flash_Write(uint32_t iAddress, uint16_t *buf, uint32_t iNbrToWrite);
void flash_program(void) ;
void flash_program_mcu(void)  ;

#endif
