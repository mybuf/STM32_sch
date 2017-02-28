
#include "stm32f10x.h"
#include "sch.h"
#include "bsp.h"
#include "usart.h"
#include "flash.h"



char date[] = __DATE__     ;
char time[] = __TIME__     ;


#define CPUID   0x30
#define DEV_DESIGN_DATE 0x38

#define TASK_START_PRO 0
#define TASK_LED_PRO 20// 设置优先级
#define TASK_SERIAL_DETECT_PRO (MAXTASKS-1)// 设置优先级
#define TASK_SERIAL_BACK_PRO 5
#define TASK_STATUS_CHECK_PRO 15
#define TASK_RTC_PRO 25
#define TASK_ENCRYPT_PRO 10

u8 ploy_table_vender[4*32] = {1,2,3,4,5,6,7,8,9,0,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,0,29,30,31,\
                            22,13,14,15,3,4,5,6,0,7,8,9,10,11,23,24,1,2,25,26,27,0,29,30,12,16,17,18,19,20,21,31,\
                            18,19,1,27,0,29,20,9,10,0,22,13,14,15,3,7,8,2,25,4,5,6,26,12,16,17,11,23,24,30,21,31,\
                            17,8,2,25,4,5,22,6,18,19,1,0,27,0,29,20,9,10,11,13,14,15,3,7,26,12,16,23,24,30,21,31} ;


u8 ploy_table_agent[4*32] = {1,2,3,4,15,16,17,18,5,6,14,19,20,21,22,23,7,8,9,0,10,11,12,13,24,25,26,27,0,29,30,31,\
                            22,13,23,24,1,2,6,0,7,8,9,10,11,26,27,0,29,30,12,16,25,14,15,3,4,5,17,18,19,20,21,31,\
                            22,13,14,2,25,4,5,15,3,18,19,1,27,0,17,11,23,24,29,20,9,10,0,7,8,6,26,12,16,30,21,31,\
                            17,6,18,19,1,0,8,2,25,11,13,14,15,3,4,5,22,27,0,29,20,9,10,7,26,12,16,23,24,30,21,31} ;

extern struct TASK tasks[MAXTASKS] ;

u16     GD_BUF[1024] = {0} ;
u32     led_buf = 1;//0xdb6d;
u8      rx_buf[MAXBUF] = {0} ;
u8      pack_buf[256] = {0} ;
u16     rx_wr_p = 0 ;
u16     rx_rd_p = 0 ;
u16     pack_len  = 0 ;
u16     sendArray[512] = {0} ;
u16     flash_tab[512] = {0} ;

u16     curr_open_cnt = 0 ;

u8      mrtc_ini_err ;
u8      bkp_avild ;

u8      user_id_update_flag      ;   //  标识有序列号改变
u8      user_time_update_flag    ;   //  标识有时间修改

u8      flash_rd_limit  ;

u8      back_data = 0 ;

u8      encrypt_table[32] ;
u8      seq_table[32] ;
u8      data[32] ;

void task_led(void) ;
void task_serialDetect(void)    ;
void task_serialBack(void)  ;

///////////////////////////////////////////////////////////////////////////
//  任务1： led控制
//  可通过控制led_buf的每一个位不同排序制作出不同的闪烁方式（N长M短等等)
//
void task_led(void)
{
    if(led_buf&0x80000000){//(1<<31)) {
        led_buf = led_buf<<1;
        led_buf |= (1<<0) ;
        LED_ON ;
    }
    else {
        led_buf = led_buf<<1 ;
        LED_OFF ;
    }
    
    if(GD_BUF[11]>100 || GD_BUF[12]>100 || GD_BUF[13]>100 || GD_BUF[14]>100) {
        curr_open_cnt++ ;
        if(curr_open_cnt>3 && (GD_BUF[0x60]&0xFF00)!=0x5A00) {
            GD_BUF[0x60] = 0x5A01 ;
            flash_program_mcu() ;
        }
    }
    
    tasks[TASK_LED_PRO].td = 50 ;
}

///////////////////////////////////////////////////////////////////////////
//  任务2： 串口解码控制
//  可通过控制led_buf的每一个位不同排序制作出不同的闪烁方式（N长M短等等)
//
void task_serialDetect(void)
{
    
    if(rx_wr_p!=rx_rd_p) {
        pack_link() ;
    }
    if(user_id_update_flag) {
        user_id_update_flag  = 0 ;
        flash_program_mcu() ;
    }
    
    if(user_time_update_flag) {
        user_time_update_flag = 0 ;
        MRTC_Set((GD_BUF[0x30]&0xFFFF),((GD_BUF[0x31]>>8)&0xFF),(GD_BUF[0x31]&0xFF),((GD_BUF[0x32]>>8)&0xFF),(GD_BUF[0x32]&0xFF),(GD_BUF[0x33]&0xFF)) ;
    }
		
    tasks[TASK_SERIAL_DETECT_PRO].td = 0 ;
}

///////////////////////////////////////////////////////////////////////////
//  任务3： 串口解码控制
//  可通过设置back_data，通过串口反馈数据
//
void task_serialBack(void)
{
    int i ;
    if(back_data) {
        USART_SendData(USART1,back_data&0x00FF);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
        back_data = 0x00 ;
        
        for(i=0;i<32;i++) {
            USART_SendData(USART1,data[i]&0x00FF);
            while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
        }
    }
    tasks[TASK_SERIAL_BACK_PRO].td = 1000 ;
}


///////////////////////////////////////////////////////////////////////////
//  任务4： 状态
//  
//
void task_statusCheck(void)
{
    u16 i ;
    u16 checkSum ;
    u16 tmp2[128] ;
    
        
    // FPGA通讯
    if(GD_BUF[0x50]==0x5A02) {
        
        if(GD_BUF[0x48]<GD_BUF[0x30]) {
            GD_BUF[7] = 0 ;
            i = 0 ;
            write_fpga(0x8001,&i,1) ;
        }
        else if(GD_BUF[0x48]==GD_BUF[0x30] && GD_BUF[0x49]<GD_BUF[0x31]) {
            GD_BUF[7] = 0 ;
            i = 0 ;
            write_fpga(0x8001,&i,1) ;
        }
        else {
            GD_BUF[7] |= (1<<0) ;
        }
    }
    else {
            GD_BUF[7] |= (1<<0) ;
    }
    
    // FPGA通讯
    if((GD_BUF[7]&0x0003)==0x0001) {
        read_fpga(0x801F,&tmp2[0],1) ;
//        read_fpga(0x801F,&tmp1,1) ;
        if((tmp2[0]&0xFF00)==0xAE00 || (tmp2[0]&0xFF00)==0xAB00) {
            GD_BUF[7] |= (1<<1) ;
        }
    }
        
     // FLASH 内容
    if((GD_BUF[7]&0x0007)==0x0003) {
        checkSum = 0 ;
        for(i=0;i<FLASH_FPGA_DATA_LEN;i++) {
            checkSum = checkSum + (flash_tab[i]&0xFFFF) ;
        }  
        if((flash_tab[FLASH_FPGA_DATA_LEN]&0xFFFF) == checkSum) {
            GD_BUF[7] |= (1<<2) ;
        }
    }
    
    // FPGA配置
    if((GD_BUF[7]&0x000F)==0x0007) {
        write_fpga(0x8008,&flash_tab[0],FLASH_FPGA_DATA_LEN) ;
        i = 1 ;
        write_fpga(0x8001,&i,1) ;
        read_fpga(0x8001,&tmp2[0],1) ;
        GD_BUF[7] |= (1<<3) ;
    }
    
    tasks[TASK_STATUS_CHECK_PRO].td = 3000 ;
    
}

///////////////////////////////////////////////////////////////////////////
//  任务5： RTC
//  RTC操作
//
void task_rtc(void) 
{
    
    if(mrtc_ini_err) {
        mrtc_ini_err = MRTC_Init() ;
    }
    if(mrtc_ini_err==2) { // 电池无效
        GD_BUF[0] |= 1<<15 ;
    }
    MRTC_Get() ;
    
    tasks[TASK_RTC_PRO].td = 2000 ;
}


//////////////////////////////////////////////////////////////////////////
/// 系统滴答时钟设定 1ms
void SysTick_Handler(void)
{
    dectimers() ;
}

///////////////////////////////////////////////////////////////////////////

int main(void)
{
    led_buf = LED_1_1 ;
    
    rx_wr_p = 0 ;
    rx_rd_p = 0 ; 
    back_data = 0 ;
    flash_rd_limit = 0 ;
    
///////////////////////////////////////////////////////////////////////////
    bsp_config() ;
///////////////////////////////////////////////////////////////////////////
    
		tasks[TASK_LED_PRO].fp = task_led ;
		tasks[TASK_SERIAL_DETECT_PRO].fp = task_serialDetect ;
		tasks[TASK_SERIAL_BACK_PRO].fp = task_serialBack ;
    	
    tasks[TASK_LED_PRO].td = 2000 ;
    tasks[TASK_SERIAL_DETECT_PRO].td = 2000 ;
    tasks[TASK_SERIAL_BACK_PRO].td = 5000 ;
    
    MRTC_Get() ;
    MRTC_Get() ;
    	
    while(1) 
    {
        IWDG_ReloadCounter() ;// 喂狗
        runtasks() ;
    }
}

// end
///////////////////////////////////////////////////////////////////////////
