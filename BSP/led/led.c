
#include "led.h"


void LED_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				     //LED1    配置为通用推挽输出  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
}
