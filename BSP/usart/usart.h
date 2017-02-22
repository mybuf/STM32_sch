
#include "stm32f10x.h"
#include "stm32f10x_usart.h"

#define HEAD_CMD 0x5AA5
#define MAXBUF 2048


void USART_config(void) ;
void pack_link(void) ;
void pack_detect(void)  ;
void pc_back(u16 cmd,u16 len,u16 addr,u16 *p);
