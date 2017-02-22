
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
u8      user_encrypt_update_flag ;   //  加解密标识
u8      user_encrypt_agent_agent_flag  ;

u8      flash_rd_limit  ;

u8      back_data = 0 ;

u8      encrypt_table[32] ;
u8      seq_table[32] ;
u8      data[32] ;


void task_led(void) ;
void task_serialDetect(void)    ;
void task_serialBack(void)  ;
void task_encrypt_cal(void) ;
void encrypt_cal_funtion(void)  ;


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
    u8  i,j,k   ;
    u16 crc ;
    u16 encrypt_result[5] ;
    u8 chk_flag ;
    
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
    
    if(user_encrypt_update_flag) {  // 解密端口
        for(i=0;i<32;i++) data[i] = 0 ; //清空

        ////////////////////////////////////////////////////////////////
        // 获取序列数据
        for(i=0;i<8;i++) { // 序列号 0x40 - 0x47  
            data[2*i+0] = (GD_BUF[0x40+i]>>8)&0xFF ;   
            data[2*i+1] = (GD_BUF[0x40+i]>>0)&0xFF ;   
        }
        
        // 获取机器当前时间 
        data[16] = (GD_BUF[0x30]>>8)&0xFF ;   
        data[17] = (GD_BUF[0x30]>>0)&0xFF ;  
        data[18] = (GD_BUF[0x31]>>8)&0xFF ;   
        data[19] = (GD_BUF[0x31]>>0)&0xFF ;   
        
        // 加密时间(月份数）, 若大于250，则为永久使用
        data[20] = 0 ;
        data[21] = GD_BUF[0x2C]&0xFF ;
        
        if(data[21]==0) // 加密月份若为0，则结束
            return ;  
        
        for(i=0;i<4;i++)
        {
            for(j=0;j<32;j++) {   seq_table[j] = ploy_table_vender[i*32+j] ;   }   // 加载顺序表
            crc = 0xFFFF ;
       
            for(k=0;k<32;k++)
            {
               crc = crc^((((data[seq_table[k]])<<8)&0xFF00)&0xFFFF) ;
               for(j=0;j<8;j++)
               {
                   if(crc&0x8000)
                       crc = (crc<<1)^0x1021 ;
                   else
                       crc = (crc<<1) ;
               }
            }
            encrypt_result[i] = crc & 0xFFFF ;
         }
         
         // 判断计算结果是否与输入密码一致
         chk_flag = 0 ;
         for(i=0;i<4;i++) {
             if(encrypt_result[i] != GD_BUF[0x28+i]) {
                 chk_flag=1 ;
             }
         }
         
         // 计算结果一致，则开始确定密码时间
         if(chk_flag==0) { // 供应商（厂家）
            ////////////////////////////////////////////////////////////////
            //    计算解除1 供应商（厂家）
            ////////////////////////////////////////////////////////////////
                    if(data[21]==255) { // 厂家永久密码
                        GD_BUF[0x50] = 0x5A03 ;
                    }
                    else if(data[21]==254) {    //设置该密码，可以读取任何数据
                        flash_rd_limit = 0x01   ;
                    }
                    else if(data[21]==253) { // 清空用户密码（变成0000-0000-0000）
                        GD_BUF[0x60] = 0x0000   ;
                        GD_BUF[0x61] = 0x0000   ;
                        GD_BUF[0x62] = 0x0000   ;
                        GD_BUF[0x63] = 0x0000   ;
                    }
                    else {
                        GD_BUF[0x51] = data[21] ;
                        if((GD_BUF[0x50]&0xFFFF)==0x5A02) {
                            encrypt_cal_funtion() ;
                        }
                        else {
                            GD_BUF[0x50] = 0x5A01 ;
                        }
                    }
                    flash_program_mcu() ;
                }
         
         else { //代理商
             ////////////////////////////////////////////////////////////////
             //    计算解除2 代理商
             ////////////////////////////////////////////////////////////////
                 
                for(i=0;i<4;i++)
                {
                    for(j=0;j<32;j++) {   seq_table[j] = ploy_table_agent[i*32+j] ;   }   // 加载顺序表
                    crc = GD_BUF[0x60+i] ; //供应商密码
             
                    for(k=0;k<32;k++)
                    {
                       crc = crc^((((data[seq_table[k]])<<8)&0xFF00)&0xFFFF) ;
                       for(j=0;j<8;j++)
                       {
                           if(crc&0x8000)
                               crc = (crc<<1)^0x1021 ;
                           else
                               crc = (crc<<1) ;
                       }
                    }
                    encrypt_result[i] = crc & 0xFFFF ;
                 }
                 
                 // 判断计算结果是否与输入密码一致
                 chk_flag = 0 ;
                 for(i=0;i<4;i++) {
                     if(encrypt_result[i] != GD_BUF[0x28+i]) {
                         chk_flag=1 ;
                     }
                 }
                 
                 // 计算结果一致，则开始确定密码时间
                 if(chk_flag==0) {
                            encrypt_cal_funtion() ;
                            flash_program_mcu() ;
                  }
              }
        
        user_encrypt_update_flag = 0 ;   
    
    }
    if(user_encrypt_agent_agent_flag) {
        
        if(GD_BUF[0x0A]==2099 || GD_BUF[0x0A]<2016) {
            for(i=0;i<4;i++) {
                GD_BUF[0x60+i] = GD_BUF[0x15+i] ;
            }
            flash_program_mcu() ;
        }
        user_encrypt_agent_agent_flag = 0 ;
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


///////////////////////////////////////////////////////////////////////////
/// 任务6： 密码处理部分
/// 
void task_encrypt_cal(void)
{
    u16 tmp1 ;
    
    if(GD_BUF[0x50]==0x5A01) { //  已经出厂模式，但未开过激光，则检测激光状态
        read_fpga(0x8000,&tmp1,1) ;
        if((tmp1&0x03)==0x03) {  //已经开启过工作电流,则锁定加密目标时间：现在时间+加密的月份
            encrypt_cal_funtion() ;
        }
    }
    else if(GD_BUF[0x51]>250) {
        GD_BUF[0x50] = 0x5A03   ;
    }
    
    tasks[TASK_ENCRYPT_PRO].td = 2000 ;
}

///////////////////////////////////////////////////////////////////////////
/// 系统滴答时钟设定 1ms
void SysTick_Handler(void)
{
    dectimers() ;
}

///////////////////////////////////////////////////////////////////////////
int main(void)
{
    u16 i,j,k ;
    u16 checkSum ;
    u32 cpuid[4] ;
    led_buf = LED_1_1 ;
    
    
    rx_wr_p = 0 ;
    rx_rd_p = 0 ; 
    back_data = 0 ;
    flash_rd_limit = 0 ;
    
    user_id_update_flag      = 0  ;
    user_time_update_flag    = 0  ;
    user_encrypt_agent_agent_flag = 0 ;
    user_encrypt_update_flag = 0 ;
    
   // SCB->VTOR = FLASH_BASE | 0x4000;
    
    
    for(i=0;i<5;i++)
    {
        for(j=0;j<1000;j++)
            for(k=0;k<1000;k++) ;
    }
    
	
    
    // 获取FLASH存储参数
    for(i=0;i<64;i++) {
        flash_tab[i] = *(u16*)(FLASH_BASE_ADDR+i*2) ;
    }
    
    for(i=0x40;i<0x80;i++) {
        GD_BUF[i] = *(u16*)(MCU_BASE_ADDR+i*2) ;
    }
    
     cpuid[0] = *(u32*)(0x1FFFF7E8) ;
     cpuid[1] = *(u32*)(0x1FFFF7EC) ;
     cpuid[2] = *(u32*)(0x1FFFF7F0) ;
     
    GD_BUF[CPUID+0] = ((cpuid[0]<<8)&0xFF00) | ((cpuid[0]>>8)&0xFF) ;
    GD_BUF[CPUID+1] = ((cpuid[0]>>8)&0xFF00) | ((cpuid[0]>>24)&0xFF) ;
    GD_BUF[CPUID+2] = ((cpuid[1]<<8)&0xFF00) | ((cpuid[1]>>8)&0xFF) ;
    GD_BUF[CPUID+3] = ((cpuid[1]>>8)&0xFF00) | ((cpuid[1]>>24)&0xFF) ;
    GD_BUF[CPUID+4] = ((cpuid[2]<<8)&0xFF00) | ((cpuid[2]>>8)&0xFF) ;
    GD_BUF[CPUID+5] = ((cpuid[2]>>8)&0xFF00) | ((cpuid[2]>>24)&0xFF) ;
    GD_BUF[CPUID+6] = ((cpuid[3]<<8)&0xFF00) | ((cpuid[3]>>8)&0xFF) ;
    GD_BUF[CPUID+7] = ((cpuid[3]>>8)&0xFF00) | ((cpuid[3]>>24)&0xFF) ;
   
    
    //  year
    GD_BUF[DEV_DESIGN_DATE+0]   = ((date[7]-0x30)*1000) + ((date[8]-0x30)*100) + ((date[9]-0x30)*10) + date[10]-0x30 ; // year
    
    // month
    if(date[0]=='J'&&date[1]=='a'&&date[2]=='n')            GD_BUF[DEV_DESIGN_DATE+1]=1<<8 ;
    else if(date[0]=='F'&&date[1]=='e'&&date[2]=='b')       GD_BUF[DEV_DESIGN_DATE+1]=2<<8 ;
    else if(date[0]=='M'&&date[1]=='a'&&date[2]=='r')       GD_BUF[DEV_DESIGN_DATE+1]=3<<8 ;
    else if(date[0]=='A'&&date[1]=='p'&&date[2]=='r')       GD_BUF[DEV_DESIGN_DATE+1]=4<<8 ;
    else if(date[0]=='M'&&date[1]=='a'&&date[2]=='y')       GD_BUF[DEV_DESIGN_DATE+1]=5<<8 ;
    else if(date[0]=='J'&&date[1]=='u'&&date[2]=='n')       GD_BUF[DEV_DESIGN_DATE+1]=6<<8 ;
    else if(date[0]=='J'&&date[1]=='u'&&date[2]=='l')       GD_BUF[DEV_DESIGN_DATE+1]=7<<8 ;
    else if(date[0]=='A'&&date[1]=='u'&&date[2]=='g')       GD_BUF[DEV_DESIGN_DATE+1]=8<<8 ;
    else if(date[0]=='S'&&date[1]=='e'&&date[2]=='p')       GD_BUF[DEV_DESIGN_DATE+1]=9<<8 ;
    else if(date[0]=='O'&&date[1]=='c'&&date[2]=='t')       GD_BUF[DEV_DESIGN_DATE+1]=10<<8 ;
    else if(date[0]=='N'&&date[1]=='o'&&date[2]=='v')       GD_BUF[DEV_DESIGN_DATE+1]=11<<8 ;
    else if(date[0]=='D'&&date[1]=='e'&&date[2]=='c')       GD_BUF[DEV_DESIGN_DATE+1]=12<<8 ;
    else GD_BUF[DEV_DESIGN_DATE+1] = 0 ;
    
    // day
    GD_BUF[DEV_DESIGN_DATE+1] |= ((date[4]-0x30)*10+(date[5]-0x30))&0xFF ;
    
    // time
    GD_BUF[DEV_DESIGN_DATE+2] = ((((time[0]-0x30)*10)+(time[1]-0x30))<<8) + ((time[3]-0x30)*10) + (time[4]-0x30) ;
    
    
    ///////////////////////////////////////////////////////////////////////////
    /// 计算MCU参数是否正确，若不正确，则赋初值
    checkSum = 0 ;
    for(i=0;i<0x3F;i++) {
        checkSum += GD_BUF[0x40+i] ;
    }
    if(checkSum!=GD_BUF[0x7F]) {
        for(i=0x40;i<0x80;i++) {
            GD_BUF[i] = 0x00 ;
        }
        // 这里开始赋初值
        
        // 重新写flash
        flash_program_mcu() ;
    }
    
///////////////////////////////////////////////////////////////////////////
    bsp_config() ;
    
///////////////////////////////////////////////////////////////////////////
    
	tasks[TASK_LED_PRO].fp = task_led ;
	tasks[TASK_SERIAL_DETECT_PRO].fp = task_serialDetect ;
	tasks[TASK_SERIAL_BACK_PRO].fp = task_serialBack ;
    tasks[TASK_STATUS_CHECK_PRO].fp = task_statusCheck ;
    tasks[TASK_RTC_PRO].fp = task_rtc ;
    tasks[TASK_ENCRYPT_PRO].fp = task_encrypt_cal ;
    	
    tasks[TASK_LED_PRO].td = 2000 ;
    tasks[TASK_SERIAL_DETECT_PRO].td = 2000 ;
    tasks[TASK_SERIAL_BACK_PRO].td = 5000 ;
    tasks[TASK_STATUS_CHECK_PRO].td = 100;
    tasks[TASK_RTC_PRO].td = 100 ;
    tasks[TASK_ENCRYPT_PRO].td = 100 ;
    
    MRTC_Get() ;
    MRTC_Get() ;
    
    
    // 加密
    if(GD_BUF[0x50]==0x5A02) {
        
        if(GD_BUF[0x48]<GD_BUF[0x30]) {
            GD_BUF[7] = 0 ;
        }
        else if(GD_BUF[0x49]<GD_BUF[0x31]) {
            GD_BUF[7] = 0 ;
        }
        else {
            GD_BUF[7] = 1 ;
        }
    }
    else {
            GD_BUF[7] |= (1<<0) ;
    }
	
    while(1) 
    {
        IWDG_ReloadCounter() ;// 喂狗
        runtasks() ;
    }
}

// end
///////////////////////////////////////////////////////////////////////////

void encrypt_cal_funtion(void)
{
    u32 tmp_m ;
    if(GD_BUF[0x51]>250) { //永久解密
        GD_BUF[0x50]=0x5A03 ;
     }
     else { // 获取设定加密时间，然后计算，到期时间，写到相应的地址0X48 0x49
        tmp_m = ((GD_BUF[0x31]>>8)&0xFF) ;
        tmp_m += (GD_BUF[0x51]&0xFF) ;
         if(tmp_m%12==0) {
            GD_BUF[0x49] = 0x0C00 + (GD_BUF[0x31]&0xFF) ; // 
            GD_BUF[0x48] = ((tmp_m-1)/12)+GD_BUF[0x30] ;
         }
         else {     
            GD_BUF[0x49] = ((tmp_m%12)<<8) + (GD_BUF[0x31]&0xFF) ; // 
            GD_BUF[0x48] = (tmp_m/12)+GD_BUF[0x30] ;
         }
        GD_BUF[0x50] = 0x5A02 ;// 定时加密开始
        GD_BUF[0x09] = tmp_m ;
     }
    // flash_program_mcu() ;
}
