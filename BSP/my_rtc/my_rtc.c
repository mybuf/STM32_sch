

    #include "my_rtc.h"
    #include "stm32f10x_pwr.h"
    #include "stm32f10x_bkp.h"
    #include "stm32f10x_rtc.h"

    void RTC_Configuration(void) ;
    void Time_Adjust(void);

    extern u8 back_data ;
    extern u16 GD_BUF[1024] ;

    #define P_ONE_MIN  60
    #define P_ONE_HOUR (60*P_ONE_MIN)
    #define P_ONE_DAY  (24*P_ONE_HOUR)
    #define P_ONE_YEAR (365*P_ONE_DAY)

    const u8 mon_tab[12] = {31,28,31,30,31,30,31,31,30,31,30,31} ;

    u8 isleapyear(u16 year)
    {
        if((year%4==0 && year%100!=0) || year%400==0)
            return 1;
        else 
            return 0 ;
    }

    u8 MRTC_Init(void)
    {  
        u8 a = 1 ;
        if(BKP_ReadBackupRegister(BKP_DR1) != BKP_DR1_FLAG)		     //判断保存在备份寄存器的RTC标志是否已经被配置过
          {
              RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
              PWR_BackupAccessCmd(ENABLE);
              BKP_DeInit();
              RCC_RTCCLKCmd(ENABLE);
              RTC_WaitForSynchro();
              RTC_WaitForLastTask();
              a = 0 ;
        }
        else {
            a = 2 ;
        }
        return a ;
    }

    u8 MRTC_Set(u16 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec)
    {
        // 以2016年1月1日0时0分0秒 作为开始(此时计数器为0)
        u16 t;
        u16 i ;
        u32 seccount=0;
        
        
        /// year
        i  = BIRTH_YEAR ;
        if(year<BIRTH_YEAR)
            return 1;
        
        if(year>i) {
            for(;i<year;i++) {
                if(isleapyear(i)) {
                    seccount += P_ONE_YEAR + P_ONE_DAY;
                }
                else {
                    seccount += P_ONE_YEAR ;
                }
            }
        }
        
        /// month
        if(mon<1 || mon>12) return 1 ;
        if(mon>1) { // 2月到12月，则计算
            for(t=0;t<(mon-1);t++) { // 必须少算当月
                seccount += (u32)mon_tab[t]*P_ONE_DAY ;
                if(t==1 && isleapyear(year)) {
                    seccount += P_ONE_DAY ;
                }
            }
        }
        
        /// day
        if(day<1 || day>mon_tab[mon-1]) {
                return 1 ;
            }
        seccount += P_ONE_DAY*(day-1) ;
        
        /// hour
        if(hour>23) return 1 ;
        seccount += (u32)hour * P_ONE_HOUR ;
        
        /// min
        if(min>59) return 1 ;
        seccount += (u32)min*P_ONE_MIN ;
        
        /// second
        if(sec>59) return 1 ;
        seccount += (u32)sec ;
            
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP,ENABLE);
        PWR_BackupAccessCmd(ENABLE);
        RTC_SetCounter(seccount);
        RTC_WaitForLastTask();
        //////PWR_BackupAccessCmd(DISABLE);
		GD_BUF[8] = (seccount>>16)&0xFFFF ;
		GD_BUF[9] = seccount&0xFFFF ;
        return 0;
    }

    void MRTC_Get(void)
    {
        u32 timecount=0,tmp;
        int i ;
        u16 current_year ;
        u8  current_mon ;
        u8  current_day ;
        u8  current_hour ;
        u8  current_min  ;
        u8  current_sec ;
        
        timecount = RTC_GetCounter();
        tmp = timecount/86400;     //天数
        
        /// year 
        current_year = BIRTH_YEAR ;
        if(tmp>=(isleapyear(current_year)+365)) {
            for(;(tmp>=(isleapyear(current_year)+365));) {
                tmp -= (365+isleapyear(current_year)) ;
				current_year++ ;
            }
        }

        
        /// month
        current_mon = 1;
        current_day = 1 ;
		
        if(tmp>(mon_tab[0]-1)) {
            current_mon++ ;
            tmp -= mon_tab[0] ;
            for(i=1;i<12;i++) {
				
				if(i==1&&isleapyear(current_year)) {
					if(tmp>29) {
                    tmp -= 29 ;
                    current_mon++ ;
					}
				}
				else if(tmp>(mon_tab[i]-1)) {
                    tmp -= mon_tab[i];
                    current_mon++ ;
                }
                else {
                    break ;
                }
            }
            current_day += tmp ;
        }
        else {
			current_mon = 1 ;
            if(tmp==0)
                current_day = 1 ;
            else
                current_day = tmp+1 ;
        }
        
        
        tmp = timecount%86400 ; // 获取秒数
        current_hour = tmp/(60*60) ;
        current_min  = (tmp%(60*60))/60 ;
        current_sec  = (tmp%(60*60))%60 ;
        
        GD_BUF[0x30] = current_year ;
        GD_BUF[0x31] = (((current_mon<<8)&0xFF00) | (current_day&0xFF))&0xFFFF ;
        GD_BUF[0x32] = (((current_hour<<8)&0xFF00) | (current_min&0xFF))&0xFFFF ;
        GD_BUF[0x33] = current_sec&0x00FF ;
        
        GD_BUF[0x03] =  timecount>>16 ;
        GD_BUF[0x04] =  timecount&0xFFFF ;
        GD_BUF[0x05] = timecount/86400;
        //return ;
        
    }

    
    /****************************************************************************
    * 名    称：void Clock_ini(void)
    * 功    能：实时时钟初始化
    * 入口参数：无
    * 出口参数：无
    * 说    明：
    * 调用方法：无 
    ****************************************************************************/ 
    void Clock_ini(void)
   {
      if(BKP_ReadBackupRegister(BKP_DR1) != BKP_DR1_FLAG)		     //判断保存在备份寄存器的RTC标志是否已经被配置过
      {
         RTC_Configuration();							      //RTC初始化	 
         MRTC_Set(2019,3,10,16,17,10) ;
         BKP_WriteBackupRegister(BKP_DR1, BKP_DR1_FLAG);    	      //RTC设置后，将已配置标志写入备份数据寄存器 
      }
      else
      {	     
         if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)	  //检查是否掉电重启
         {
           //printf("\r\n\n Power On Reset occurred....");
         }												     
         else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) //检查是否reset复位
         {
           //printf("\r\n\n External Reset occurred....");
         }
         //printf("\r\n No need to configure RTC....");  
         RTC_WaitForSynchro();								   //等待RTC寄存器被同步 
         RTC_ITConfig(RTC_IT_SEC, ENABLE);					   //使能秒中断
         RTC_WaitForLastTask();								   //等待写入完成
      }
      RCC_ClearFlag();										   //清除复位标志
    }
    
    void RTC_Configuration(void)
    { 
      /* 使能 PWR 和 BKP 的时钟 */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
      
      /* 允许访问BKP区域 */
      PWR_BackupAccessCmd(ENABLE);

      /* 复位BKP */
      BKP_DeInit();

    #ifdef RTCClockSource_LSI
      /* 使能内部RTC时钟 */ 
      RCC_LSICmd(ENABLE);
      /* 等待RTC内部时钟就绪 */
      while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
      {
      }
      /* 选择RTC内部时钟为RTC时钟 */
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);  
    #elif defined	RTCClockSource_LSE  
      /* 使能RTC外部时钟 */
      RCC_LSEConfig(RCC_LSE_ON);
      /* 等待RTC外部时钟就绪 */
      while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
      {	    
      }

      /* 选择RTC外部时钟为RTC时钟 */
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
    #endif
      /* 使能RTC时钟 */
      RCC_RTCCLKCmd(ENABLE);


    #ifdef RTCClockOutput_Enable  
      /* Disable the Tamper Pin */
      BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
                                   functionality must be disabled */
                                   
      /* 使能在TAMPER脚输出RTC时钟 */
      BKP_RTCCalibrationClockOutputCmd(ENABLE);
    #endif 

      /* 等待RTC寄存器同步 */
      RTC_WaitForSynchro();

      /* 等待写RTC寄存器完成 */
      RTC_WaitForLastTask();
      
      /* 使能RTC秒中断 */  
      RTC_ITConfig(RTC_IT_SEC, ENABLE);

      /* 等待写RTC寄存器完成 */
      RTC_WaitForLastTask();
      
      /* 设置RTC预分频 */
    #ifdef RTCClockSource_LSI
      RTC_SetPrescaler(31999);            /* RTC period = RTCCLK/RTC_PR = (32.000 KHz)/(31999+1) */
    #elif defined	RTCClockSource_LSE
      RTC_SetPrescaler(32767);            /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
    #endif
      
      /* 等待写RTC寄存器完成 */
      RTC_WaitForLastTask();
    }
