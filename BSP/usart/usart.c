
#include "stm32f10x_usart.h"
#include "usart.h"
#include "led.h"
#include "cpu2fpga.h"
#include "flash.h"

extern  u16 GD_BUF[1024] ;
extern  u8 rx_buf[MAXBUF] ;
extern  u16 rx_wr_p ;
extern  u16 rx_rd_p ;
extern  u8 pack_buf[256] ;
extern  u16 pack_len ;
extern  u16 sendArray[256] ;
extern  u32 led_buf ;

extern u8 back_data ;
extern u16 flash_tab[512] ;

extern u8 user_id_update_flag ;
extern u8 user_time_update_flag ;

extern u8 flash_rd_limit    ;

u16 buf_tmp[128] ;
u16 buf_tmp2[128] ;

//////////////////////////////////////////////////////////////////////////////////////////////
void USART_config(void) 
{

    USART_InitTypeDef USART_InitStructure ;
    GPIO_InitTypeDef GPIO_InitStructure ;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE);	 //使能串口1时钟
    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				     //LED1    配置为通用推挽输出  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);		
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				     //LED1    配置为通用推挽输出  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
        
    USART_InitStructure.USART_BaudRate = 9600;						//速率115200bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
    USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
    
    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);							//配置串口参数函数
    
    
    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                    //使能接收中断
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);						//使能发送缓冲空中断   
    
    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);	
}



//////////////////////////////////////////////////////////////////////////////////////////////
// 组包部分
void usart_rec_func(void)
{
    rx_buf[rx_wr_p]  = (USART_ReceiveData(USART1) & 0xFF) ;
    rx_wr_p++ ;
    if(rx_wr_p==MAXBUF) {
        rx_wr_p = 0 ;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
///   head + command + datalen + address + data + checksum
///   5aa5   	0023   0045      0067      0089   00ab
int check_func(void)
{
    return 0 ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// 串口1 中断服务程序
void USART1_IRQHandler(void)  
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	   //判断读寄存器是否非空
  {
		usart_rec_func();
  }
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)                   //这段是为了避免STM32 USART 第一个字节发不出去的BUG 
  { 
     USART_ITConfig(USART1, USART_IT_TXE, DISABLE);					     //禁止发缓冲器空中断， 
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// 包 链路层
void pack_link(void) 
{
	//u16 i ;
    //  头 0xc000  ,  dataLen高位乘以512（忽略) ，dataLen低位*2, 控制位
    if((pack_len==(0xC000 + pack_buf[5]*2 + 10-1 ))\
	|| (pack_len >(0xC000+256+10-1))\
	|| (pack_buf[3]==0x02 && pack_len==(0xC000+10-1))\
	|| (pack_buf[3]==0x03 && pack_len==(0xC000+10-1))\
    || (pack_buf[3]==0x05 && pack_len==(0xC000+10-1)))
	//  1. 可变长度
	//  2. 最大长度
	//  3. 读固定长度
    //  4. 读取id固定长度
	{
        pack_buf[pack_len&0x0FFF] = rx_buf[rx_rd_p] ;
        if(!check_func()) {
/* 		       for(i=0;i<(pack_len&0xff + 10);i++) {
					USART_SendData(USART1,pack_buf[i]);
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
				} */
            pack_detect() ;
        } 
	//	back_data = 0x78 ;
		pack_len = 0 ;
    }
    else if((pack_len & 0xF000) == 0xC000) 
	// 1. 接收可用包数据
	{
		//back_data = 0x56 ;
        pack_buf[pack_len&0xFFF] = rx_buf[rx_rd_p] ;
    }
    else if((pack_len & (1<<15)) && rx_buf[rx_rd_p]==0xa5) {
	//  校验包头2
		//back_data = 0x34 ;
        pack_len = 0xC001 ;
		pack_buf[1] = 0xa5 ;
    }
    else if(rx_buf[rx_rd_p]==0x5a) {
	// 校验包头1
        pack_len = 0x8000 ;
		pack_buf[0] = 0x5a ;
	//	back_data = 0x12 ;
    }
    pack_len++ ;
    
    rx_buf[rx_rd_p] = 0 ;
    
    rx_rd_p++ ;
    if(rx_rd_p==MAXBUF) {
        rx_rd_p = 0 ;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// 包 应用层
void pack_detect(void) 
{
    int i ;	
    if(pack_buf[2]==0) { // 对外公开命令
            /// 写操作
            if(pack_buf[3]==0x01) {
                if(pack_buf[6]&(1<<7)) { // CPLD
                    for(i=0;i<(pack_buf[4]*256+pack_buf[5]);i++)
                    {
                        buf_tmp[i] = pack_buf[8+i*2]*256+pack_buf[9+i*2] ;
                    }
                    write_fpga((pack_buf[6]*256+pack_buf[7]),&buf_tmp[0],(pack_buf[4]*256+pack_buf[5])) ;// 写到FPGA
                    read_fpga((pack_buf[6]*256+pack_buf[7]),&buf_tmp2[0],(pack_buf[4]*256+pack_buf[5])) ;// 读FPGA
                    pc_back(2,(pack_buf[4]*256+pack_buf[5]),(pack_buf[6]*256+pack_buf[7]),&buf_tmp2[0]) ;// 上传到PC
                }
                else { // MCU 内存
                    if(pack_buf[5]>0) {
                        for(i=0;i<pack_buf[5];i++) {
                            if((GD_BUF[0x10]==0xA55A) \
                            || (GD_BUF[0x10]==0x5A0A && ((pack_buf[6]*256+pack_buf[7] + i)<0x60)) \
                            || (GD_BUF[0x10]==0xEF05 && ((pack_buf[6]*256+pack_buf[7] + i)<0x40)) \
                            || ((pack_buf[6]*256+pack_buf[7] + i)<0x30))
                                {
                                    GD_BUF[(pack_buf[6]*256+pack_buf[7] + i)%1024] = pack_buf[8+2*i]*256+pack_buf[9+2*i] ;
                                    
                                    // 更新FLASH 标识
                                    if((pack_buf[6]*256+pack_buf[7] + i)>0x3F && (pack_buf[6]*256+pack_buf[7] + i)<0x80) {
                                        user_id_update_flag = 1 ;
                                    }
                                    
                                    // 更新时间标识
                                    if((pack_buf[6]*256+pack_buf[7] + i)>0x2F && (pack_buf[6]*256+pack_buf[7] + i)<0x34) {
                                        user_time_update_flag = 1 ;
                                    }
                                }
                           }
                        }
                }
            }
            /// 读操作
            else if(pack_buf[3]==0x02) {
                if(pack_buf[6]&(1<<7)) { // CPLD
                    read_fpga((pack_buf[6]*256+pack_buf[7]),&sendArray[0],(pack_buf[4]*256+pack_buf[5]));
                    pc_back(2,(pack_buf[4]*256+pack_buf[5]),(pack_buf[6]*256+pack_buf[7]),&sendArray[0]) ;
                    if(i ==10) { // 若读取失败，则反馈BD
                        back_data = 0x01 ;
                    } 
                }
                else { // MCU 内存
                    if(pack_buf[5]>0) {
                        for(i=0;i<pack_buf[5];i++) { // 获取数据到发送buffer
                            if(flash_rd_limit) {
                                sendArray[i] = GD_BUF[pack_buf[6]*256+pack_buf[7]+i] ;
                            }
                            else if(GD_BUF[0x10]==0xA55A || GD_BUF[0x10]==0x5A0A) {
                                sendArray[i] = GD_BUF[pack_buf[6]*256+pack_buf[7]+i] ;
                            }
                            else if((pack_buf[6]*256+pack_buf[7]+i)<0x5F) {
                                sendArray[i] = GD_BUF[pack_buf[6]*256+pack_buf[7]+i] ;
                            }
                            else {
                                sendArray[i] = 0xFFFF ;
                            }
                        }                        
                        pc_back(2,(pack_buf[4]*256+pack_buf[5]),(pack_buf[6]*256+pack_buf[7]),&sendArray[0]) ;
                    }
                }
            }
            /// 固化参数
            else if(pack_buf[3]==0x03) {
                read_fpga(0x8008,&flash_tab[0],FLASH_FPGA_DATA_LEN) ; // 读FPGA参数
                flash_tab[FLASH_FPGA_DATA_LEN] = 0 ;
                for(i=0;i<FLASH_FPGA_DATA_LEN;i++) {    // 计算校验和
                    flash_tab[FLASH_FPGA_DATA_LEN] += flash_tab[i] ;
                }
                back_data = 0x03 ;
                flash_program() ; // 写参数到FLASH
                pc_back(0x0003,0x0000,0x8008,&buf_tmp[0]);
            }
            /// 读取flash
            else if(pack_buf[3]==0x04) {
                for(i=0;i<64;i++) {
                    buf_tmp[i] = *(u16*)(FLASH_BASE_ADDR+i*2) ;
                }
                pc_back(0x0002,(pack_buf[4]*256+pack_buf[5]),(pack_buf[6]*256+pack_buf[7]),&buf_tmp[0]) ;
            }
			/// 写flash
			else if(pack_buf[3]==0x05) {
				
			}
        }
    /// 异常
	else {
		back_data = 0x01;
	}
	
}

//////////////////////////////////////////////////////////////////////////////////////////////
// 包 生成 并且发送
void pc_back(u16 cmd,u16 len,u16 addr,u16 *p)
{
    u16 i ;
    u16 checkSum = 0 ;
    u8 data[1024] = {0} ;
	
	/// 头控制部分
    data[0] = 0x5a ;
    data[1] = 0xa5 ;
    data[2] = (cmd>>8)&0xFF ;
    data[3] = cmd&0xFF ;
    data[4] = (len>>8)&0xFF ;
    data[5] = len&0xFF ;
    data[6] = (addr>>8)&0xFF ;
    data[7] = addr&0xFF ;
	
	/// 数据部分
    for(i=0;i<len;i++) {
        data[8+i*2] = ((*p)>>8)&0xFF ;
		data[9+i*2] = (*p)&0xFF ;
        p++ ;
    }
    
	/// 校验和计算部分
    checkSum = 0 ;
    for(i=0;i<(len+4);i++) {
        checkSum += (data[2*i]*256 + data[2*i+1]) ;
    }
    
    data[len*2+8] = (checkSum>>8) & 0xFF ;
    data[len*2+9] = checkSum & 0xFF ;
    
	/// 发送部分
    for(i=0;i<(len*2+10);i++) {
        USART_SendData(USART1,data[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
    }
}


