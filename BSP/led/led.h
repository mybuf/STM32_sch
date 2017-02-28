
#include "stm32f10x.h"

void LED_config(void) ;
#define LED_OFF GPIO_SetBits(GPIOC,GPIO_Pin_13)
#define LED_ON GPIO_ResetBits(GPIOC,GPIO_Pin_13)

#define LED_0_0_0_1 0x000000FF
#define LED_1_1 0x0000FFFF

#define LED_3_0 0xF0F0F000
#define LED_3_1 0xF0F0F0C0
#define LED_3_2 0xF0F0F0CC
#define LED_2_0 0xF0F00000
#define LED_2_1 0xF0F0C000
#define LED_2_2 0xF0F0C0C0

