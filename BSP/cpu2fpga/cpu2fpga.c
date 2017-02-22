
#include "stm32f10x.h"
#include "cpu2fpga.h"

#include "stm32f10x_gpio.h"

#define IRQ_PIN		GPIO_Pin_0
#define CS_n			GPIO_Pin_1 
#define SCLK			GPIO_Pin_2 
#define SDO				GPIO_Pin_3 
#define SDI				GPIO_Pin_4

#define CPU2FPGA_W 0x0001 
#define CPU2FPGA_R 0x0002 


uint8_t sendDataBuffer[256] ;
extern uint16_t fpga2cpu_data ;
extern uint16_t cpu2fpga_rbuf[256] ;
extern uint16_t cpu2fpga_wbuf[256] ;

int cpu2fpga_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure ;
	// cs_n 、 wr 、 clock 、dout 、din 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_InitStructure.GPIO_Pin = CS_n | SCLK | SDO ;
	GPIO_Init(GPIOA,&GPIO_InitStructure) ;

	GPIO_InitStructure.GPIO_Pin = SDI ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
	GPIO_Init(GPIOA,&GPIO_InitStructure) ;
	
	
	// 中断
	
	GPIO_InitStructure.GPIO_Pin = IRQ_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
	GPIO_Init(GPIOA,&GPIO_InitStructure) ;
	
	
	
	
	GPIO_SetBits(GPIOA,CS_n | SCLK | SDO) ; // 初始化
	
	return 0 ;
}

int sclk_gen(void)
{
    int i ;
    
	for(i=5;i>0;i--) ;
	GPIO_ResetBits(GPIOA,SCLK) ;
	GPIO_SetBits(GPIOA,SCLK) ;
	GPIO_ResetBits(GPIOA,SCLK) ;
	GPIO_ResetBits(GPIOA,SCLK) ;
	GPIO_ResetBits(GPIOA,SCLK) ;
	for(i=5;i>0;i--) ;
	return 0 ;
}

int send_16bit_data(uint16_t data_in)
{		
	uint16_t i = 0 ;	
	for(i=0;i<16;i++)	// 1byte 串行输出 高位在前
			{
				if(0x8000 == (data_in & 0x8000))
				{
					GPIO_SetBits(GPIOA,SDO) ;
				}
				else
				{
					GPIO_ResetBits(GPIOA,SDO) ;
				}
				sclk_gen() ; // 输出时钟
				data_in = data_in<<1 ;
			}
			return 0 ;
}
uint16_t rec_16bit_data(void)
{
	int i ;
	uint16_t data = 0x0000 ;
	for(i=5;i>0;i--) ;
	for(i=0;i<16;i++) {
		data = data<<1 ;
		
		for(i=5;i>0;i--) ;
		if(0x00 != (0x01&GPIO_ReadInputDataBit(GPIOA,SDI))) {
			data |= 0x0001 ;
		}
		else {
			data &= 0xFFFE ;
		}
		sclk_gen() ;
	}
	for(i=5;i>0;i--) ;
	return data ;
}

int write_fpga(uint16_t addr, uint16_t *p,uint16_t len) 
{
	
	int j ;
	uint16_t checksum = 0x0000 ;
	GPIO_SetBits(GPIOA,CS_n) ;
	GPIO_ResetBits(GPIOA,CS_n) ;
	
	send_16bit_data(0x5AA5) ;	
	send_16bit_data(0x0001) ;	
	send_16bit_data(len) ;	
	send_16bit_data(addr) ;

	for(j=0 ; j<len; j++)
	{
		send_16bit_data(*p) ;
		checksum = checksum + *p ;
		p++ ;
	}
//	GPIO_SetBits(GPIOA,RW) ;		
	GPIO_SetBits(GPIOA,CS_n) ;
	
	return 0 ;
}

void read_fpga(u16 addr,uint16_t *pbuf ,uint16_t len) 
{
	
	int j ;
	uint16_t checksum = 0x0000 ;
	GPIO_SetBits(GPIOA,CS_n) ;
	GPIO_ResetBits(GPIOA,CS_n) ;
	
	send_16bit_data(0x5AA5) ;	
	send_16bit_data(0x0002) ;	
	send_16bit_data(len) ;	
	send_16bit_data(addr) ;

	checksum = 0x5AA5 + 0x0002 + len + addr ;
	for(j=0 ; j<(len+1); j++)
	{
		*pbuf = rec_16bit_data() ;
		checksum = checksum + *pbuf ;
		pbuf++ ;
		//cpu2fpga_rbuf[j] = rec_16bit_data() ;
	}
			
	GPIO_SetBits(GPIOA,CS_n) ;
}

uint8_t read_irq(void)
{
		return GPIO_ReadInputDataBit(GPIOA,IRQ_PIN) ;
	
}



