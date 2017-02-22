

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
        if(BKP_ReadBackupRegister(BKP_DR1) != BKP_DR1_FLAG)		     //�жϱ����ڱ��ݼĴ�����RTC��־�Ƿ��Ѿ������ù�
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
        // ��2016��1��1��0ʱ0��0�� ��Ϊ��ʼ(��ʱ������Ϊ0)
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
        if(mon>1) { // 2�µ�12�£������
            for(t=0;t<(mon-1);t++) { // �������㵱��
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
        tmp = timecount/86400;     //����
        
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
        
        
        tmp = timecount%86400 ; // ��ȡ����
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
    * ��    �ƣ�void Clock_ini(void)
    * ��    �ܣ�ʵʱʱ�ӳ�ʼ��
    * ��ڲ�������
    * ���ڲ�������
    * ˵    ����
    * ���÷������� 
    ****************************************************************************/ 
    void Clock_ini(void)
   {
      if(BKP_ReadBackupRegister(BKP_DR1) != BKP_DR1_FLAG)		     //�жϱ����ڱ��ݼĴ�����RTC��־�Ƿ��Ѿ������ù�
      {
         RTC_Configuration();							      //RTC��ʼ��	 
         MRTC_Set(2019,3,10,16,17,10) ;
         BKP_WriteBackupRegister(BKP_DR1, BKP_DR1_FLAG);    	      //RTC���ú󣬽������ñ�־д�뱸�����ݼĴ��� 
      }
      else
      {	     
         if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)	  //����Ƿ��������
         {
           //printf("\r\n\n Power On Reset occurred....");
         }												     
         else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) //����Ƿ�reset��λ
         {
           //printf("\r\n\n External Reset occurred....");
         }
         //printf("\r\n No need to configure RTC....");  
         RTC_WaitForSynchro();								   //�ȴ�RTC�Ĵ�����ͬ�� 
         RTC_ITConfig(RTC_IT_SEC, ENABLE);					   //ʹ�����ж�
         RTC_WaitForLastTask();								   //�ȴ�д�����
      }
      RCC_ClearFlag();										   //�����λ��־
    }
    
    void RTC_Configuration(void)
    { 
      /* ʹ�� PWR �� BKP ��ʱ�� */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
      
      /* �������BKP���� */
      PWR_BackupAccessCmd(ENABLE);

      /* ��λBKP */
      BKP_DeInit();

    #ifdef RTCClockSource_LSI
      /* ʹ���ڲ�RTCʱ�� */ 
      RCC_LSICmd(ENABLE);
      /* �ȴ�RTC�ڲ�ʱ�Ӿ��� */
      while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
      {
      }
      /* ѡ��RTC�ڲ�ʱ��ΪRTCʱ�� */
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);  
    #elif defined	RTCClockSource_LSE  
      /* ʹ��RTC�ⲿʱ�� */
      RCC_LSEConfig(RCC_LSE_ON);
      /* �ȴ�RTC�ⲿʱ�Ӿ��� */
      while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
      {	    
      }

      /* ѡ��RTC�ⲿʱ��ΪRTCʱ�� */
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
    #endif
      /* ʹ��RTCʱ�� */
      RCC_RTCCLKCmd(ENABLE);


    #ifdef RTCClockOutput_Enable  
      /* Disable the Tamper Pin */
      BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
                                   functionality must be disabled */
                                   
      /* ʹ����TAMPER�����RTCʱ�� */
      BKP_RTCCalibrationClockOutputCmd(ENABLE);
    #endif 

      /* �ȴ�RTC�Ĵ���ͬ�� */
      RTC_WaitForSynchro();

      /* �ȴ�дRTC�Ĵ������ */
      RTC_WaitForLastTask();
      
      /* ʹ��RTC���ж� */  
      RTC_ITConfig(RTC_IT_SEC, ENABLE);

      /* �ȴ�дRTC�Ĵ������ */
      RTC_WaitForLastTask();
      
      /* ����RTCԤ��Ƶ */
    #ifdef RTCClockSource_LSI
      RTC_SetPrescaler(31999);            /* RTC period = RTCCLK/RTC_PR = (32.000 KHz)/(31999+1) */
    #elif defined	RTCClockSource_LSE
      RTC_SetPrescaler(32767);            /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
    #endif
      
      /* �ȴ�дRTC�Ĵ������ */
      RTC_WaitForLastTask();
    }
