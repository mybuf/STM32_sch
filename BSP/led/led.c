
#include "led.h"


void LED_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				     //LED1    ����Ϊͨ���������  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
}
