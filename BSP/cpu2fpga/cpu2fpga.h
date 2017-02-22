

//#include <math.h>
#include "stm32f10x.h"
//typedef unsigned short uint16_t;
//typedef unsigned	char uint8_t;

int cpu2fpga_config(void) ;
int sclk_gen(void) ;

int send_16bit_data(uint16_t data_in) ;
uint16_t rec_16bit_data(void) ;

// application function
int write_fpga(u16 addr, u16 *p,u16 len) ;
void read_fpga(u16 addr,u16 *pbuf ,u16 len) ;

uint8_t read_irq(void) ;

// head(16bit) + command(16bit) + datalength(16bit) + address(16bit) + data1(16bit) + ... datan(16bit) + checksum ;
// head = 0x5aa5
// command =  : 0x0001 д  0x0002 ��
// datalength = (���ݳ���)
// address = ��ʼ��ַ
// data1 ... datan  = : (n������)
// checksum = (0 - ǰ�����еĺͣ� �޷���


