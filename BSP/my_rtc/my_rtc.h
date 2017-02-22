
#include "stm32f10x.h"
#define BKP_DR1_FLAG 0x5AA5

#define BIRTH_YEAR 2016

//#define RTCClockSource_LSI   /* 用内置的32K 时钟晶振源 */
#define RTCClockSource_LSE   /* 用外置的32.768K 时钟晶振源 */



u8 MRTC_Init(void) ;
u8 MRTC_Set(u16 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec) ;
void MRTC_Get(void) ;
void Clock_ini(void);
