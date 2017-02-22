

#include "stm32f10x_flash.h"
#include "flash.h"

extern u16 flash_tab[512] ;
extern u16 GD_BUF[1024] ;

void flash_program(void)
{
    u16  i ; 
	FLASH_Unlock();
	FLASH_ErasePage(FLASH_BASE_ADDR);
    for(i=0;i<32;i++) {
        FLASH_ProgramHalfWord(FLASH_BASE_ADDR+2*i,flash_tab[i]);
    }
    FLASH_Lock();
}

void flash_program_mcu(void) 
{
    u16  i ; 
    u16  checkSum = 0 ;
	FLASH_Unlock();
	FLASH_ErasePage(MCU_BASE_ADDR);
    checkSum = 0 ;
    for(i=0x40;i<0x7F;i++) {
        FLASH_ProgramHalfWord(MCU_BASE_ADDR+2*i,GD_BUF[i]);
        checkSum += GD_BUF[i] ;
    }
    
    FLASH_ProgramHalfWord(MCU_BASE_ADDR+2*i,checkSum&0xFFFF) ;
        
    FLASH_Lock();
}
