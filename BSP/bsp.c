
#include "bsp.h"
//#include "stm32f10x_iwdg.h"


/****************************************************************************
* 名    称：void sys_config(void)
* 功    能：系统时钟配置为72MHZ， 外设时钟配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/ 
void sys_config(void){
   SystemInit(); 
   //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP,ENABLE) ;
   //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO  , ENABLE);  
   SysTick_Config(72000) ;	//时钟节拍中断时1ms一次  用于定时  
}

// 看门狗配置
u8 WDG_flag = 0 ;
static void WDG_Configuration(void)
{
    WDG_flag = 0;
    /* Check if the system has resumed from IWDG reset */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)	/* IWDGRST flag set */
    {
      //  LED_Open();
        /* Clear reset flags */
        RCC_ClearFlag();
        WDG_flag = 1;
    }
    /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
       dispersion) */
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    /* IWDG counter clock: LSI/32 */
    IWDG_SetPrescaler(IWDG_Prescaler_32);
    /* Set counter reload value to obtain 250ms IWDG TimeOut.
       Counter Reload Value = 250ms/IWDG counter clock period  (::;:default 40K)
     */
    IWDG_SetReload(500);
    /* Reload IWDG counter */
    IWDG_ReloadCounter();
    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();	
}

///////////////////////////////////////////////////////////////////////////////
void NVIC_Configuration(void)
{
  /*  结构声明*/
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  /* Configure one bit for preemption priority */
  /* 优先级组 说明了抢占优先级所用的位数，和子优先级所用的位数   在这里是1， 7 */    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	  
  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//设置串口1中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//抢占优先级 0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;				//子优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能
  NVIC_Init(&NVIC_InitStructure);
    
     
//  /* Enable the RTC Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;					//配置外部中断源（秒中断） 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);  
    
}

void bsp_config(void)
{
    sys_config() ;
    NVIC_Configuration();
    USART_config() ;
    cpu2fpga_config() ;
    LED_config() ;
    //MRTC_Init() ;
    //Clock_ini() ;
    
    SPI1_Init() ;
    
    
    WDG_Configuration() ;
}
