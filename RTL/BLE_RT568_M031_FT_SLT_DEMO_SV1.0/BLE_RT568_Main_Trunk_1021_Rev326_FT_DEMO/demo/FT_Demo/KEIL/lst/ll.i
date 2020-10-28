#line 1 "..\\..\\..\\source\\LL.c"











 
#pragma push

#pragma Ospace




 
#line 1 "..\\..\\FT_Demo\\BleAppSetting.h"



#line 1 "..\\..\\..\\porting\\mcu_definition.h"



#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 5 "..\\..\\..\\porting\\mcu_definition.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\NuMicro.h"
 





 



#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
 








 







































 




 
 
 



 






 



 
typedef enum IRQn
{
     
    NonMaskableInt_IRQn       = -14,       
    HardFault_IRQn            = -13,       
    SVCall_IRQn               = -5,        
    PendSV_IRQn               = -2,        
    SysTick_IRQn              = -1,        

     
    BOD_IRQn                  = 0,         
    WDT_IRQn                  = 1,         
    EINT024_IRQn              = 2,         
    EINT135_IRQn              = 3,         
    GPIO_PAPB_IRQn            = 4,         
    GPIO_PAPBPGPH_IRQn        = 4,         
    GPIO_PCPDPEPF_IRQn        = 5,         
    PWM0_IRQn                 = 6,         
    PWM1_IRQn                 = 7,         
    TMR0_IRQn                 = 8,         
    TMR1_IRQn                 = 9,         
    TMR2_IRQn                 = 10,        
    TMR3_IRQn                 = 11,        
    UART02_IRQn               = 12,        
    UART1_IRQn                = 13,        
    UART13_IRQn               = 13,        
    SPI0_IRQn                 = 14,        
    QSPI0_IRQn                = 15,        
    ISP_IRQn                  = 16,        
    UART57_IRQn               = 17,        
    I2C0_IRQn                 = 18,        
    I2C1_IRQn                 = 19,        
    BPWM0_IRQn                = 20,        
    BPWM1_IRQn                = 21,        
    USCI_IRQn                 = 22,        
    USCI01_IRQn               = 22,        
    USBD_IRQn                 = 23,        
    ACMP01_IRQn               = 25,        
    PDMA_IRQn                 = 26,        
    UART46_IRQn               = 27,        
    PWRWU_IRQn                = 28,        
    ADC_IRQn                  = 29,        
    CKFAIL_IRQn               = 30,        
    RTC_IRQn                  = 31,        
} IRQn_Type;






 

 




   

#line 1 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
 




 

























 











#line 45 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

















 




 



 

 













#line 120 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"



 







#line 162 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

#line 1 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 263 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\cmsis_armcc.h"


#line 297 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\cmsis_armcc.h"



 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 



__attribute__((always_inline)) static __inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;
  int32_t s = 4   * 8 - 1;  

  result = value;                       
  for (value >>= 1U; value; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;                         
  return(result);
}








 



#line 649 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\cmsis_armcc.h"

   


 



 

#line 731 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\cmsis_armcc.h"
 


#line 54 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"

 
#line 84 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

#line 164 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 
#line 84 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

#line 165 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
















 
#line 198 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

 






 
#line 214 "..\\..\\..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

 




 










 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:28;               
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t _reserved1:3;                
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t _reserved0:1;                
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 



 







 



 
typedef struct
{
  volatile uint32_t ISER[1U];                
        uint32_t RESERVED0[31U];
  volatile uint32_t ICER[1U];                
        uint32_t RSERVED1[31U];
  volatile uint32_t ISPR[1U];                
        uint32_t RESERVED2[31U];
  volatile uint32_t ICPR[1U];                
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  volatile uint32_t IP[8U];                  
}  NVIC_Type;

 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
        uint32_t RESERVED0;
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
        uint32_t RESERVED1;
  volatile uint32_t SHP[2U];                 
  volatile uint32_t SHCSR;                   
} SCB_Type;

 















 



























 















 









 






 



 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 








 
 







 






 







 


 







 

 










 









 


 



 





 

 
 









 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
  else
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 2) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 










#line 134 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\system_M031Series.h"
 









 







 
 
 











 



 






extern uint32_t SystemCoreClock;     
extern uint32_t CyclesPerUs;         
extern uint32_t PllClock;            

#line 65 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\system_M031Series.h"












 
extern void SystemInit(void);











 
extern void SystemCoreClockUpdate(void);







 
#line 135 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"



#pragma anon_unions











 
extern void SystemInit(void);



 
 
 

#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\acmp_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

































































































 
    volatile uint32_t CTL[2];                 
    volatile uint32_t STATUS;                 
    volatile uint32_t VREF;                   
    volatile uint32_t CALCTL;                 
    volatile const  uint32_t CALSR;                  

} ACMP_T;




 










































































































   
   
   


#pragma no_anon_unions


#line 160 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\adc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    



























































































































































































































 
    volatile const  uint32_t ADDR[30];               
    volatile const  uint32_t RESERVE1[2];
    volatile uint32_t ADCR;                   
    volatile uint32_t ADCHER;                 
    volatile uint32_t ADCMPR[2];              
    volatile uint32_t ADSR0;                  
    volatile const  uint32_t ADSR1;                  
    volatile const  uint32_t ADSR2;                  
    volatile const  uint32_t RESERVE2[1];
    volatile uint32_t ESMPCTL;                
    volatile uint32_t CFDCTL;                 
    volatile const  uint32_t RESERVE3[22];
    volatile const  uint32_t ADPDMA;                 
    volatile const  uint32_t RESERVE4[31];
    volatile uint32_t ADCALR;                 
    volatile uint32_t ADCALSTSR;              
    volatile uint32_t ADCALDBR;               
} ADC_T;




 






















































































































   
   
   


#pragma no_anon_unions


#line 161 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\clk_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    








































































































































































































































































































































































































































































































 
    volatile uint32_t PWRCTL;                 
    volatile uint32_t AHBCLK;                 
    volatile uint32_t APBCLK0;                
    volatile uint32_t APBCLK1;                
    volatile uint32_t CLKSEL0;                
    volatile uint32_t CLKSEL1;                
    volatile uint32_t CLKSEL2;                
    volatile uint32_t CLKSEL3;                
    volatile uint32_t CLKDIV0;                
    volatile const  uint32_t RESERVE0[3];
    volatile uint32_t CLKDIV4;                
    volatile uint32_t PCLKDIV;                
    volatile const  uint32_t RESERVE1[2];
    volatile uint32_t PLLCTL;                 
    volatile const  uint32_t RESERVE2[3];
    volatile const  uint32_t STATUS;                 
    volatile const  uint32_t RESERVE3[3];
    volatile uint32_t CLKOCTL;                
    volatile const  uint32_t RESERVE4[3];
    volatile uint32_t CLKDCTL;                
    volatile uint32_t CLKDSTS;                
    volatile uint32_t CDUPB;                  
    volatile uint32_t CDLOWB;                 
    volatile uint32_t LDOCTL;                 
    volatile const  uint32_t RESERVE5[12];
    volatile uint32_t HXTFSEL;                
    volatile const  uint32_t RESERVE9[14];
    volatile uint32_t TESTCLK;                

} CLK_T;




 



































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 162 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\crc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

































































 
    volatile uint32_t CTL;                    
    volatile uint32_t DAT;                    
    volatile uint32_t SEED;                   
    volatile const  uint32_t CHECKSUM;               

} CRC_T;




 



































   
   
   


#pragma no_anon_unions


#line 163 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\ebi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    




































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t TCTL0;                  
    volatile const  uint32_t RESERVE0[2];
    volatile uint32_t CTL1;                   
    volatile uint32_t TCTL1;                  

} EBI_T;




 








































   
   
   


#pragma no_anon_unions


#line 164 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\fmc_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    




















































































































































 
    volatile uint32_t ISPCTL;                 
    volatile uint32_t ISPADDR;                
    volatile uint32_t ISPDAT;                 
    volatile uint32_t ISPCMD;                 
    volatile uint32_t ISPTRG;                 
    volatile const  uint32_t DFBA;                   
    volatile uint32_t FTCTL;                  
    volatile uint32_t ICPCTL;                 
    volatile const  uint32_t RESERVE0[8];
    volatile uint32_t ISPSTS;                 
    volatile const  uint32_t RESERVE1[15];
    volatile uint32_t MPDAT0;                 
    volatile uint32_t MPDAT1;                 
    volatile uint32_t MPDAT2;                 
    volatile uint32_t MPDAT3;                 
    volatile const  uint32_t RESERVE2[12];
    volatile const  uint32_t MPSTS;                  
    volatile const  uint32_t MPADDR;                 
    volatile const  uint32_t RESERVE3[0x3CD];
    volatile const  uint32_t VERSION;                
} FMC_T;




 







































































































   
   
   


#pragma no_anon_unions


#line 165 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\gpio_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

























































































































 

    volatile uint32_t MODE;                
    volatile uint32_t DINOFF;              
    volatile uint32_t DOUT;                
    volatile uint32_t DATMSK;              
    volatile const  uint32_t PIN;                 
    volatile uint32_t DBEN;                
    volatile uint32_t INTTYPE;             
    volatile uint32_t INTEN;               
    volatile uint32_t INTSRC;              
} GPIO_T;

typedef struct
{


    





























 

    volatile uint32_t DBCTL;             
} GPIO_DBCTL_T;





 













































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 166 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\hdiv_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    








 
    volatile uint32_t DIVIDEND;

    









 
    volatile uint32_t DIVISOR;

    








 
    volatile uint32_t QUOTIENT;

    









 
    volatile uint32_t REM;

    












 
    volatile const  uint32_t STATUS;

} HDIV_T;




 
















   
   
   


#pragma no_anon_unions





   
   
#line 167 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\i2c_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

































































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t ADDR0;                  
    volatile uint32_t DAT;                    
    volatile const  uint32_t STATUS0;                
    volatile uint32_t CLKDIV;                 
    volatile uint32_t TOCTL;                  
    volatile uint32_t ADDR1;                  
    volatile uint32_t ADDR2;                  
    volatile uint32_t ADDR3;                  
    volatile uint32_t ADDRMSK0;               
    volatile uint32_t ADDRMSK1;               
    volatile uint32_t ADDRMSK2;               
    volatile uint32_t ADDRMSK3;               
    volatile const  uint32_t RESERVE0[2];
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t CTL1;                   
    volatile uint32_t STATUS1;                
    volatile uint32_t TMCTL;                  
    volatile uint32_t BUSCTL;                 
    volatile uint32_t BUSTCTL;                
    volatile uint32_t BUSSTS;                 
    volatile uint32_t PKTSIZE;                
    volatile const  uint32_t PKTCRC;                 
    volatile uint32_t BUSTOUT;                
    volatile uint32_t CLKTOUT;                
} I2C_T;




 





































































































































































































































   
   
   


#pragma no_anon_unions


#line 168 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\pdma_reg.h"
 





 




#pragma anon_unions





 




 


typedef struct
{


    
















































































 
    volatile uint32_t CTL;       
    volatile uint32_t SA;        
    volatile uint32_t DA;        
    volatile uint32_t NEXT;      

} DSCT_T;


typedef struct
{


    

































































































































































































































































































































 
    DSCT_T        DSCT[9];                
    volatile const  uint32_t RESERVE0[28];
    volatile const  uint32_t CURSCAT[9];             
    volatile const  uint32_t RESERVE1[183];
    volatile uint32_t CHCTL;                  
    volatile  uint32_t PAUSE;                  
    volatile  uint32_t SWREQ;                  
    volatile const  uint32_t TRGSTS;                 
    volatile uint32_t PRISET;                 
    volatile  uint32_t PRICLR;                 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t ABTSTS;                 
    volatile uint32_t TDSTS;                  
    volatile uint32_t ALIGN;                  
    volatile const  uint32_t TACTSTS;                
    volatile uint32_t TOUTPSC;                
    volatile uint32_t TOUTEN;                 
    volatile uint32_t TOUTIEN;                
    volatile uint32_t SCATBA;                 
    volatile uint32_t TOC0_1;                 
    volatile const  uint32_t RESERVE2[7];
    volatile uint32_t CHRST;                  
    volatile const  uint32_t RESERVE3[7];
    volatile uint32_t REQSEL0_3;              
    volatile uint32_t REQSEL4_7;              
    volatile uint32_t REQSEL8;                
} PDMA_T;




 


























































































































































































   
   
   


#pragma no_anon_unions


#line 169 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\pwm_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{
    















































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    volatile const  uint32_t RESERVE0[2];
    volatile uint32_t CLKSRC;                 
    volatile uint32_t CLKPSC[3];              
    volatile uint32_t CNTEN;                  
    volatile uint32_t CNTCLR;                 
    volatile const  uint32_t RESERVE1[2];
    volatile uint32_t PERIOD[6];             
    volatile const  uint32_t RESERVE2[2];
    volatile uint32_t CMPDAT[6];              
    volatile const  uint32_t RESERVE3[2];
    volatile uint32_t DTCTL[3];               
    volatile const  uint32_t RESERVE4[5];
    volatile const  uint32_t CNT[6];                 
    volatile const  uint32_t RESERVE5[2];
    volatile uint32_t WGCTL0;                 
    volatile uint32_t WGCTL1;                 
    volatile uint32_t MSKEN;                  
    volatile uint32_t MSK;                    
    volatile uint32_t BNF;                    
    volatile uint32_t FAILBRK;                
    volatile uint32_t BRKCTL[3];              
    volatile uint32_t POLCTL;                 
    volatile uint32_t POEN;                   
    volatile  uint32_t SWBRK;                  
    volatile uint32_t INTEN0;                 
    volatile uint32_t INTEN1;                 
    volatile uint32_t INTSTS0;                
    volatile uint32_t INTSTS1;                
    volatile const  uint32_t RESERVE6[2];
    volatile uint32_t ADCTS0;                 
    volatile uint32_t ADCTS1;                 
    volatile const  uint32_t RESERVE7[4];
    volatile uint32_t SSCTL;                  
    volatile  uint32_t SSTRG;                  
    volatile const  uint32_t RESERVE8[2];
    volatile uint32_t STATUS;                 
    volatile const  uint32_t RESERVE9[55];
    volatile uint32_t CAPINEN;                
    volatile uint32_t CAPCTL;                 
    volatile const  uint32_t CAPSTS;                 
    volatile const  uint32_t RCAPDAT0;               
    volatile const  uint32_t FCAPDAT0;               
    volatile const  uint32_t RCAPDAT1;               
    volatile const  uint32_t FCAPDAT1;               
    volatile const  uint32_t RCAPDAT2;               
    volatile const  uint32_t FCAPDAT2;               
    volatile const  uint32_t RCAPDAT3;               
    volatile const  uint32_t FCAPDAT3;               
    volatile const  uint32_t RCAPDAT4;               
    volatile const  uint32_t FCAPDAT4;               
    volatile const  uint32_t RCAPDAT5;               
    volatile const  uint32_t FCAPDAT5;               
    volatile uint32_t PDMACTL;                
    volatile const  uint32_t PDMACAP0_1;             
    volatile const  uint32_t PDMACAP2_3;             
    volatile const  uint32_t PDMACAP4_5;             
    volatile const  uint32_t RESERVE10[1];
    volatile uint32_t CAPIEN;                 
    volatile uint32_t CAPIF;                  
    volatile const  uint32_t RESERVE11[43];
    volatile const  uint32_t PBUF[6];                
    volatile const  uint32_t CMPBUF[6];              
} PWM_T;




 



























































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 170 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\bpwm_reg.h"
 





 





    #pragma anon_unions





 




 
typedef struct
{
    














 
    volatile uint32_t RCAPDAT;  
    volatile uint32_t FCAPDAT;  
} BCAPDAT_T;

typedef struct
{


    






































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t CTL0;                   
    volatile uint32_t CTL1;                   
    volatile const  uint32_t RESERVE0[2];
    volatile uint32_t CLKSRC;                 
    volatile uint32_t CLKPSC;                 
    volatile const  uint32_t RESERVE1[2];
    volatile uint32_t CNTEN;                  
    volatile uint32_t CNTCLR;                 
    volatile const  uint32_t RESERVE2[2];
    volatile uint32_t PERIOD;                 
    volatile const  uint32_t RESERVE3[7];
    volatile uint32_t CMPDAT[6];              
    volatile const  uint32_t RESERVE4[10];
    volatile const  uint32_t CNT;                    
    volatile const  uint32_t RESERVE5[7];
    volatile uint32_t WGCTL0;                 
    volatile uint32_t WGCTL1;                 
    volatile uint32_t MSKEN;                  
    volatile uint32_t MSK;                    
    volatile const  uint32_t RESERVE6[5];
    volatile uint32_t POLCTL;                 
    volatile uint32_t POEN;                   
    volatile const  uint32_t RESERVE7[1];
    volatile uint32_t INTEN;                  
    volatile const  uint32_t RESERVE8[1];
    volatile uint32_t INTSTS;                 
    volatile const  uint32_t RESERVE9[3];
    volatile uint32_t EADCTS0;                
    volatile uint32_t EADCTS1;                
    volatile const  uint32_t RESERVE10[4];
    volatile uint32_t SSCTL;                  
    volatile  uint32_t SSTRG;                  
    volatile const  uint32_t RESERVE11[2];
    volatile uint32_t STATUS;                 
    volatile const  uint32_t RESERVE12[55];
    volatile uint32_t CAPINEN;                
    volatile uint32_t CAPCTL;                 
    volatile const  uint32_t CAPSTS;                 
    BCAPDAT_T CAPDAT[6];                  
    volatile const  uint32_t RESERVE13[5];
    volatile uint32_t CAPIEN;                 
    volatile uint32_t CAPIF;                  
    volatile const  uint32_t RESERVE14[43];
    volatile const  uint32_t PBUF;                   
    volatile const  uint32_t RESERVE15[5];
    volatile const  uint32_t CMPBUF[6];              

} BPWM_T;




 






















































































































































































































































































































































































































































































































































































































   
   
   


    #pragma no_anon_unions


#line 171 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\qspi_reg.h"
 





 




#pragma anon_unions





 




 
typedef struct
{


    









































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t CLKDIV;                 
    volatile uint32_t SSCTL;                  
    volatile uint32_t PDMACTL;                
    volatile uint32_t FIFOCTL;                
    volatile uint32_t STATUS;                 
    volatile const  uint32_t RESERVE0[2];
    volatile  uint32_t TX;                     
    volatile const  uint32_t RESERVE1[3];
    volatile const  uint32_t RX;                     

} QSPI_T;




 









































































































































































































   
   
   


#pragma no_anon_unions


#line 172 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\spi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    
































































































































































































































































































































































































































































 

    volatile uint32_t CTL;                    
    volatile uint32_t CLKDIV;                 
    volatile uint32_t SSCTL;                  
    volatile uint32_t PDMACTL;                
    volatile uint32_t FIFOCTL;                
    volatile uint32_t STATUS;                 
    volatile const  uint32_t RESERVE0[2];
    volatile  uint32_t TX;                     
    volatile const  uint32_t RESERVE1[3];
    volatile const  uint32_t RX;                     
    volatile const  uint32_t RESERVE2[11];
    volatile uint32_t I2SCTL;                 
    volatile uint32_t I2SCLK;                 
    volatile uint32_t I2SSTS;                 

} SPI_T;




 


















































































































































































































































































   
   
   


#pragma no_anon_unions


#line 173 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\sys_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    



































































































































































































































































































































































































































































































































































 
    volatile const  uint32_t PDID;                   
    volatile uint32_t RSTSTS;                 
    volatile uint32_t IPRST0;                 
    volatile uint32_t IPRST1;                 
    volatile uint32_t IPRST2;                 
    volatile const  uint32_t RESERVE0[1];
    volatile uint32_t BODCTL;                 
    volatile const  uint32_t RESERVE1[2];
    volatile uint32_t PORCTL;                 
    volatile const  uint32_t RESERVE2[2];
    volatile uint32_t GPA_MFPL;               
    volatile uint32_t GPA_MFPH;               
    volatile uint32_t GPB_MFPL;               
    volatile uint32_t GPB_MFPH;               
    volatile uint32_t GPC_MFPL;               
    volatile uint32_t GPC_MFPH;               
    volatile uint32_t GPD_MFPL;               
    volatile uint32_t GPD_MFPH;               
    volatile uint32_t GPE_MFPL;               
    volatile uint32_t GPE_MFPH;               
    volatile uint32_t GPF_MFPL;               
    volatile uint32_t GPF_MFPH;               
    volatile uint32_t GPG_MFPL;               
    volatile uint32_t GPG_MFPH;               
    volatile uint32_t GPH_MFPL;               
    volatile uint32_t GPH_MFPH;               
    volatile const  uint32_t RESERVE3[2];
    volatile uint32_t LPLDOCTL;               
    volatile const  uint32_t RESERVE4[17];
    volatile uint32_t MODCTL;                 
    volatile const  uint32_t RESERVE5[3];
    volatile uint32_t SRAM_BISTCTL;           
    volatile const  uint32_t SRAM_BISTSTS;           
    volatile uint32_t SRAM_PARITY;            
    volatile uint32_t SRAM_INTCTL;            
    volatile uint32_t SRAM_STATUS;            
    volatile const  uint32_t SRAM_ERRADDR;           
    volatile const  uint32_t RESERVE6[2];
    volatile uint32_t HIRCTRIMCTL;            
    volatile uint32_t HIRCTRIMIEN;            
    volatile uint32_t HIRCTRIMSTS;            
    volatile const  uint32_t RESERVE7[1];
    volatile  uint32_t REGLCTL;                
    volatile const  uint32_t RESERVE8[5];
    volatile uint32_t HIRCADJ;                
    volatile const  uint32_t RESERVE9[1];
    volatile const  uint32_t LDOTRIM;                
    volatile const  uint32_t LVR16TRIM;              
    volatile const  uint32_t RESERVE10[4];
    volatile const  uint32_t LIRCT;                  
    volatile const  uint32_t RESERVE11[5];
    volatile const  uint32_t LVR17TRIM;              
    volatile const  uint32_t LVR20TRIM;              
    volatile const  uint32_t LVR25TRIM;              
    volatile const  uint32_t uLDOVITRIM;             
    volatile uint32_t LVRITRIMSEL;            
    volatile const  uint32_t RESERVE12[9];
    volatile uint32_t HIRCTCTL;               
    volatile uint32_t ADCCHIP;                
    volatile uint32_t HXTTCTL;                
    volatile const  uint32_t RESERVE13[22];
    volatile uint32_t PORDISAN;               
} SYS_T;

typedef struct
{


    






























































































 
    volatile uint32_t NMIEN;                  
    volatile const  uint32_t NMISTS;                 

} NMI_T;




 




















































































































































































































































































































































































































































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 174 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\rtc_reg.h"
 





 







 

 



 

typedef struct
{


    












































































































































































































 
    volatile uint32_t INIT;                   
    volatile const  uint32_t RESERVE0[1];
    volatile uint32_t FREQADJ;                
    volatile uint32_t TIME;                   
    volatile uint32_t CAL;                    
    volatile uint32_t CLKFMT;                 
    volatile uint32_t WEEKDAY;                
    volatile uint32_t TALM;                   
    volatile uint32_t CALM;                   
    volatile const  uint32_t LEAPYEAR;               
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t TICK;                   
    volatile uint32_t TAMSK;                  
    volatile uint32_t CAMSK;                  
    volatile const  uint32_t RESERVE1[49];           
    volatile uint32_t LXTCTL;                 

} RTC_T;




 





























































































































































   
   
   

#line 175 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\timer_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    





















































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t CMP;                    
    volatile uint32_t INTSTS;                 
    volatile const  uint32_t CNT;                    
    volatile const  uint32_t CAP;                    
    volatile uint32_t EXTCTL;                 
    volatile uint32_t EINTSTS;                
} TIMER_T;




 






























































































   
   
   


#pragma no_anon_unions


#line 176 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\uart_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    






























































































































































































































































































































































































































































































































































































































































































 
    volatile uint32_t DAT;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t FIFO;                   
    volatile uint32_t LINE;                   
    volatile uint32_t MODEM;                  
    volatile uint32_t MODEMSTS;               
    volatile uint32_t FIFOSTS;                
    volatile uint32_t INTSTS;                 
    volatile uint32_t TOUT;                   
    volatile uint32_t BAUD;                   
    volatile uint32_t IRDA;                   
    volatile uint32_t ALTCTL;                 
    volatile uint32_t FUNCSEL;                
    volatile const  uint32_t RESERVE0[2];
    volatile uint32_t BRCOMP;                 
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t DWKCOMP;                

} UART_T;





 





























































































































































































































































































































































   
   
   


#pragma no_anon_unions


#line 177 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\ui2c_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    















































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile const  uint32_t RESERVE0[1];
    volatile uint32_t BRGEN;                  
    volatile const  uint32_t RESERVE1[8];
    volatile uint32_t LINECTL;                
    volatile  uint32_t TXDAT;                  
    volatile const  uint32_t RXDAT;                  
    volatile const  uint32_t RESERVE2[3];
    volatile uint32_t DEVADDR0;               
    volatile uint32_t DEVADDR1;               
    volatile uint32_t ADDRMSK0;               
    volatile uint32_t ADDRMSK1;               
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                
    volatile const  uint32_t RESERVE3[8];
    volatile uint32_t ADMAT;                  
    volatile uint32_t TMCTL;                  

} UI2C_T;




 






































































































































































   
   


   


#pragma no_anon_unions


#line 178 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\usbd_reg.h"
 





 




#pragma anon_unions





 




 



typedef struct
{

    































































 
    volatile uint32_t BUFSEG;                 
    volatile uint32_t MXPLD;                  
    volatile uint32_t CFG;                    
    volatile uint32_t CFGP;                   

} USBD_EP_T;

typedef struct
{
    

























































































































































































































































 
    volatile uint32_t INTEN;                  
    volatile uint32_t INTSTS;                 
    volatile uint32_t FADDR;                  
    volatile const  uint32_t EPSTS;                  
    volatile uint32_t ATTR;                   
    volatile const  uint32_t VBUSDET;                
    volatile uint32_t STBUFSEG;               
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile const  uint32_t EPSTS0;                 
    
    volatile const  uint32_t RESERVE1[25];
    
    volatile const  uint32_t LPMATTR;                
    volatile const  uint32_t FN;                     
    volatile uint32_t SE0;                    
    
    volatile const  uint32_t RESERVE2[283];
    
    USBD_EP_T EP[8];                     

} USBD_T;




 

















































































































































































   
   
   


#pragma no_anon_unions




#line 179 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\uspi_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

















































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t BRGEN;                  
    volatile const  uint32_t RESERVE0[1];
    volatile uint32_t DATIN0;                 
    volatile const  uint32_t RESERVE1[3];
    volatile uint32_t CTLIN0;                 
    volatile const  uint32_t RESERVE2[1];
    volatile uint32_t CLKIN;                  
    volatile uint32_t LINECTL;                
    volatile  uint32_t TXDAT;                  
    volatile const  uint32_t RXDAT;                  
    volatile uint32_t BUFCTL;                 
    volatile  uint32_t BUFSTS;                 
    volatile uint32_t PDMACTL;                
    volatile const  uint32_t RESERVE3[4];
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                

} USPI_T;




 










































































































































































































   
   


   


#pragma no_anon_unions


#line 180 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\uuart_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    

































































































































































































































































































































































































 
    volatile uint32_t CTL;                    
    volatile uint32_t INTEN;                  
    volatile uint32_t BRGEN;                  
    volatile const  uint32_t RESERVE0[1];
    volatile uint32_t DATIN0;                 
    volatile const  uint32_t RESERVE1[3];
    volatile uint32_t CTLIN0;                 
    volatile const  uint32_t RESERVE2[1];
    volatile uint32_t CLKIN;                  
    volatile uint32_t LINECTL;                
    volatile  uint32_t TXDAT;                  
    volatile const  uint32_t RXDAT;                  
    volatile uint32_t BUFCTL;                 
    volatile uint32_t BUFSTS;                 
    volatile uint32_t PDMACTL;                
    volatile const  uint32_t RESERVE3[4];
    volatile uint32_t WKCTL;                  
    volatile uint32_t WKSTS;                  
    volatile uint32_t PROTCTL;                
    volatile uint32_t PROTIEN;                
    volatile uint32_t PROTSTS;                

} UUART_T;




 



















































































































































































































   
   


   


#pragma no_anon_unions


#line 181 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\wdt_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{

    























































































 
    volatile uint32_t CTL;                    
    volatile uint32_t ALTCTL;                 
    volatile  uint32_t RSTCNT;                 

} WDT_T;




 





































   
   
   


#pragma no_anon_unions




#line 182 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\wwdt_reg.h"
 





 




#pragma anon_unions





 




 

typedef struct
{


    





































































 
    volatile  uint32_t RLDCNT;                 
    volatile uint32_t CTL;                    
    volatile uint32_t STATUS;                 
    volatile const  uint32_t CNT;                    

} WWDT_T;




 




























   
   
   


#pragma no_anon_unions


#line 183 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"


 
 
 



 
 






 






#line 217 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"



























#line 252 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"









   

 
 
 




 
#line 280 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"

#line 289 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"























































   







 

typedef volatile unsigned char  vu8;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 


   

 
 
 




 













 
#line 535 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"


 










 
#line 556 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"

   

 
 
 
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 








 










 



 



 

 
 
 





#line 59 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 66 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
 
 






#line 84 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

#line 93 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
 
 








 
 
#line 114 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 124 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 136 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 150 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 161 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 171 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 182 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 193 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 203 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 212 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 221 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 230 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 





 





 




 




 
#line 265 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 278 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 292 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 307 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 321 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 335 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 349 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 363 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 374 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 385 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 396 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 407 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 422 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 437 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 452 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 467 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 478 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 491 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 500 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 509 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 518 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 527 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 538 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 549 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 558 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 






 






 
#line 581 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 590 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 600 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 608 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 

 
#line 618 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 






 
#line 633 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 642 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 





 




 





 





 





 





 




 




 
#line 697 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 




 






 





 
#line 723 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 731 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 740 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 749 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 758 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 767 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 






 





 
#line 788 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 796 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 804 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 813 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 






 
#line 830 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 





 





 
#line 851 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 860 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 870 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 879 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 888 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 899 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 






 





 





 





 





 





 

 

 
#line 949 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
#line 957 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 

 

 






 






 




 

 

 

 

 




 




 





 





 





 





 





 

 

 

 

 





 





 




 




 






 






 






 
#line 1087 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 

 

 

 


   



 








 









 









 









 









 










 









 









 









 











 









 









 









 









 









 









 









 









 









 









 









 
















 










 
static __inline void SYS_UnlockReg(void)
{
    do {
        ((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL = 0x59;
        ((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL = 0x16;
        ((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL = 0x88;
    } while (((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL == 0);
}








 
static __inline void SYS_LockReg(void)
{
    ((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL = 0;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);


   

   

   








 
#line 563 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"
 








 










 



 



 


#line 48 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
#line 60 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 






#line 76 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"




 
 
 







#line 97 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 104 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 111 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 118 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 125 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 132 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 139 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 



























 
 
 
#line 179 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 186 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 193 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 200 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 207 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 214 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"


 
 
 






 
 
 
#line 234 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 












 
 
 










#line 273 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 
 

#line 289 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 298 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"













































































































































   



 

 
 
 







 
static __inline uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq;
    uint32_t u32FIN, u32NF, u32NR, u32NO;
    uint8_t au8NoTbl[4] = {1, 2, 2, 4};  
    uint32_t u32Reg;

    u32PllFreq = 0;
    u32Reg = ((CLK_T *) ((( uint32_t)0x40000000) + 0x00200))->PLLCTL;

    if ((u32Reg & ((0x1ul << (16)) | (0x1ul << (18)))) == 0)
    {
         
        if (u32Reg & (0x1ul << (19)))
        {
            u32FIN = ((48000000UL) >> 2);
        } else
            u32FIN = (32000000UL);

        if (u32Reg & (0x1ul << (17)))
        {
             
            u32PllFreq = u32FIN;
        }
        else
        {
             
            u32NO = au8NoTbl[((u32Reg & (0x3ul << (14))) >> (14))];
            u32NF  = ((u32Reg & (0x1fful << (0))) >> (0)) + 2;
            u32NR  = ((u32Reg & (0x1ful << (9))) >> (9)) + 2;
             
            u32PllFreq = (((u32FIN >> 2) * u32NF) / (u32NR * u32NO) << 2);
        }
    }

    return u32PllFreq;
}










 
static __inline void CLK_SysTickDelay(uint32_t us)
{
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x00);
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) | (1UL );

     
    while ((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == 0);

     
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0;
}







 
static __inline uint32_t CLK_GetUARTFreq(void)
{
    uint32_t u32Freqout, u32AHBDivider, u32ClkSel, PCLK0Div;

    u32Freqout = 0;
    u32ClkSel = ((CLK_T *) ((( uint32_t)0x40000000) + 0x00200))->CLKSEL1 & (0x7ul << (24)) ;

    if (u32ClkSel == (0x0UL<<(24)))   
    {
        u32Freqout = (32000000UL);
    }
    else if(u32ClkSel == (0x1UL<<(24)))       
    {
        u32Freqout = CLK_GetPLLClockFreq();
    }
    else if(u32ClkSel == (0x2UL<<(24)))       
    {
        u32Freqout = (32768UL);
    }
    else if(u32ClkSel == (0x3UL<<(24)))      
    {
        u32Freqout = (48000000UL);
    }
    else if(u32ClkSel == (0x4UL<<(24)))     
    {
        PCLK0Div = (((CLK_T *) ((( uint32_t)0x40000000) + 0x00200))->PCLKDIV & (0x7ul << (0))) >> (0);
        u32Freqout = (SystemCoreClock >> PCLK0Div);
    }
    else if(u32ClkSel == (0x5UL<<(24)))      
    {
        u32Freqout = (38400UL);
    }

    u32AHBDivider = (((CLK_T *) ((( uint32_t)0x40000000) + 0x00200))->CLKDIV0 & (0xful << (8))) + 1 ;

    return (u32Freqout/u32AHBDivider);
}


uint32_t CLK_WaitClockReady(uint32_t);
void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHXTFreq(void);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_DisablePLL(void);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_DisableSysTick(void);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_PowerDown(void);
void CLK_Idle(void);

   

   

   







 
#line 564 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\acmp.h"
 








 



 
 
 
#line 1 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
 








 







































 

#line 588 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"

 
#line 18 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\acmp.h"









 



 




 


 
 
 
#line 62 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\acmp.h"

 
 
 




   




 

 
 
 









 









 













 









 









 










 









 









 









 









 









 









 









 









 









 














 









 









 


















 












 











 













 












 









 
















 









 



 
void ACMP_Open(ACMP_T *, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *, uint32_t u32ChNum);



   

   

   








 
#line 565 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"
 








 











 



 



 

 
 
 



























 
 
 
#line 71 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"

 
 
 




 
 
 





 
 
 





 
 
 





   



 









 













 













 









 










 










 








 








 
















 
#line 222 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"







 
















 
#line 256 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"







 










 












 








 













 









 








 







 













 













 



void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_SetExtendSampleTime(ADC_T *adc,
                             uint32_t u32ModuleNum,
                             uint32_t u32ExtendSampleTime);


   

   

   







 
#line 566 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\crc.h"









 
 











 



 



 
 
 
 





 
 
 





 
 
 




   




 













 











 











 


void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen);
uint32_t CRC_GetChecksum(void);

   

   

   







 
#line 567 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"
 








 











 



 



 
 
 
 




 
 
 



 
 
 



 
 
 



 
 
 
#line 65 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"

#line 73 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"




   




 









 











 










 











 










 











 










 











 










 











 










 











 










 










 


void EBI_Open(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel);
void EBI_Close(uint32_t u32Bank);
void EBI_SetBusTiming(uint32_t u32Bank, uint32_t u32TimingConfig, uint32_t u32MclkDiv);

   

   

   







 
#line 568 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"
 









 










 



 




 


	
 
 
 
#line 45 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"





#line 57 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"



 
 
 
#line 78 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"







   




 


 
 
 

#line 111 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"

 

static __inline uint32_t FMC_ReadCID(void);
static __inline uint32_t FMC_ReadPID(void);
static __inline uint32_t FMC_ReadUID(uint8_t u8Index);
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index);
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
static __inline uint32_t FMC_GetVECMAP(void);








 
static __inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPSTS & (0x1ffffful << (9)));
}






 
static __inline uint32_t FMC_ReadCID(void)
{
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x0BUL;            
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = 0x0u;                          
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = (0x1ul << (0));           



    while(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG & (0x1ul << (0))) {}  

    return ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadPID(void)
{
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x0CUL;           
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = 0x04u;                        
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = (0x1ul << (0));          



    while(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG & (0x1ul << (0))) {}  

    return ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadUID(uint8_t u8Index)
{
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x04UL;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = ((uint32_t)u8Index << 2u);
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT = 0u;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = 0x1u;



    while(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG) {}

    return ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT;
}






 
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x04UL;             
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = (0x04u * u32Index) + 0x10u;     
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = (0x1ul << (0));            



    while(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG & (0x1ul << (0))) {}   

    return ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT;
}








 
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x2EUL;   
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = u32PageAddr;        
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = 0x1u;                



    while(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG) {}              
}


 
 
 

extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_Erase_SPROM(void);
extern int32_t FMC_Erase_Bank(uint32_t u32BankAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetBootSource(int32_t i32BootSrc);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t  FMC_Write8Bytes(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1);
extern int32_t  FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count);
extern int32_t  FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count);
extern uint32_t FMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern uint32_t  FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);
extern int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len);
extern int32_t FMC_RemapBank(uint32_t u32BankIdx);

   

   

   







 
#line 569 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"
 








 



#line 15 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"








 



 



 

















 
#line 67 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

#line 84 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

#line 100 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

#line 117 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

#line 134 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

#line 149 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

#line 160 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

#line 169 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

 
 
 






 
 
 






 
 
 



 
 
 






#line 219 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"


   



 













 














 














 














 














 














 














 















 































 








 









 








 




















 














 



void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);


   

   

   







 
#line 570 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\i2c.h"
 





 











 



 



 

 
 
 
#line 41 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\i2c.h"

 
 
 



 
 
 





   



 










 











 











 











 












 











 












 












 











 












 











 












 












 











 












 












 












 












 












 











 












 











 











 











 











 











 











 








 








 








 








 








 








 








 


 
 
 

 
static __inline void I2C_STOP(I2C_T *i2c);









 
static __inline void I2C_STOP(I2C_T *i2c)
{

    (i2c)->CTL0 |= ((0x1ul << (3)) | (0x1ul << (4)));
    while(i2c->CTL0 & (0x1ul << (4)))
    {
    }
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
uint8_t I2C_WriteByte(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data);
uint32_t I2C_WriteMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_WriteByteOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_WriteByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_ReadByte(I2C_T *i2c, uint8_t u8SlaveAddr);
uint32_t I2C_ReadMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t I2C_ReadByteOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t I2C_ReadMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t I2C_ReadByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t I2C_ReadMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint32_t I2C_SMBusGetStatus(I2C_T *i2c);
void I2C_SMBusClearInterruptFlag(I2C_T *i2c, uint8_t u8SMBusIntFlag);
void I2C_SMBusSetPacketByteCount(I2C_T *i2c, uint32_t u32PktSize);
void I2C_SMBusOpen(I2C_T *i2c, uint8_t u8HostDevice);
void I2C_SMBusClose(I2C_T *i2c);
void I2C_SMBusPECTxEnable(I2C_T *i2c, uint8_t u8PECTxEn);
uint8_t I2C_SMBusGetPECValue(I2C_T *i2c);
void I2C_SMBusIdleTimeout(I2C_T *i2c, uint32_t us, uint32_t u32Hclk);
void I2C_SMBusTimeout(I2C_T *i2c, uint32_t ms, uint32_t u32Pclk);
void I2C_SMBusClockLoTimeout(I2C_T *i2c, uint32_t ms, uint32_t u32Pclk);

   

   

   







 
#line 571 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"
 





 











 



 



 


 
 
 




 
 
 




 
 
 





 
 
 



#line 66 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"

 
 
 



 
 
 
#line 116 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"


 
 
 





   



 










 











 












 











 












 











 












 












 













 













 













 













 













 












 












 


 
 
 
void PDMA_Open(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_Close(PDMA_T *pdma);
void PDMA_SetTransferCnt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_SetBurstType(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize);
void PDMA_EnableTimeout(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_DisableTimeout(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_SetTimeOut(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void PDMA_Trigger(PDMA_T *pdma, uint32_t u32Ch);
void PDMA_EnableInt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Mask);


   

   

   







 
#line 572 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"
 








 











 



 



 
#line 38 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"

 
 
 




 
 
 



 
 
 





 
 
 



 
 
 
#line 76 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"

 
 
 
#line 88 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"

#line 97 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"




 
 
 







 
 
 



 
 
 



 
 
 
#line 132 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"

 
 
 
#line 144 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"

 
 
 







   




 







 








 













 










 
#line 211 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"








 










 









 









 












 
















 











 











 









 












 









 













 
#line 360 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"









 
#line 378 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"
 




























 
#line 420 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"












 












 




 
 
 
uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrake(PWM_T *pwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnablePDMA(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32RisingFirst, uint32_t u32Mode);
void PWM_DisablePDMA(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_DisableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_ClearFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
uint32_t PWM_GetFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableZeroInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_DisableZeroInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearZeroIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetZeroIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableLoadMode(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void PWM_DisableLoadMode(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void PWM_SetClockSource(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
void PWM_EnableBrakeNoiseFilter(PWM_T *pwm, uint32_t u32BrakePinNum, uint32_t u32ClkCnt, uint32_t u32ClkDivSel);
void PWM_DisableBrakeNoiseFilter(PWM_T *pwm, uint32_t u32BrakePinNum);
void PWM_EnableBrakePinInverse(PWM_T *pwm, uint32_t u32BrakePinNum);
void PWM_DisableBrakePinInverse(PWM_T *pwm, uint32_t u32BrakePinNum);
void PWM_SetBrakePinSource(PWM_T *pwm, uint32_t u32BrakePinNum, uint32_t u32SelAnotherModule);
uint32_t PWM_GetWrapAroundFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearWrapAroundFlag(PWM_T *pwm, uint32_t u32ChannelNum);

   

   

   







 
#line 573 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\bpwm.h"









 











 



 




 
#line 39 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\bpwm.h"

 
 
 




 
 
 



 
 
 





 
 
 





 
 
 
#line 79 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\bpwm.h"

 
 
 



 
 
 



 
 
 



 
 
 






   




 














 










 









 









 








 








 












 













 











 










 









 











 









 













 









 






























 
#line 326 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\bpwm.h"


 
 
 
uint32_t BPWM_ConfigCaptureChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void BPWM_DisableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t BPWM_GetADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_DisableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_ClearCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t BPWM_GetCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_DisableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_DisableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_SetClockSource(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
uint32_t BPWM_GetWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);


   

   

   







 
#line 574 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\qspi.h"
 





 











 



 



 













 
#line 53 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\qspi.h"

 
#line 63 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\qspi.h"

   




 







 








 








 








 








 








 









 









 









 








 









 








 








 








 








 










 








 








 









 









 








 








 








 








 








 








 







 







 







 







 







 







 





 
uint32_t QSPI_Open(QSPI_T *qspi, uint32_t u32MasterSlave, uint32_t u32QSPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void QSPI_Close(QSPI_T *qspi);
void QSPI_ClearRxFIFO(QSPI_T *qspi);
void QSPI_ClearTxFIFO(QSPI_T *qspi);
void QSPI_DisableAutoSS(QSPI_T *qspi);
void QSPI_EnableAutoSS(QSPI_T *qspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t QSPI_SetBusClock(QSPI_T *qspi, uint32_t u32BusClock);
void QSPI_SetFIFO(QSPI_T *qspi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t QSPI_GetBusClock(QSPI_T *qspi);
void QSPI_EnableInt(QSPI_T *qspi, uint32_t u32Mask);
void QSPI_DisableInt(QSPI_T *qspi, uint32_t u32Mask);
uint32_t QSPI_GetIntFlag(QSPI_T *qspi, uint32_t u32Mask);
void QSPI_ClearIntFlag(QSPI_T *qspi, uint32_t u32Mask);
uint32_t QSPI_GetStatus(QSPI_T *qspi, uint32_t u32Mask);


   

   

   







 
#line 575 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\spi.h"









 











 



 



 
#line 41 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\spi.h"

 
#line 53 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\spi.h"

 
#line 63 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\spi.h"


 





 



 





 



 




 





 



 



 
#line 112 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\spi.h"

   




 






 







 







 







 







 







 








 








 








 







 








 







 







 







 







 









 







 







 








 








 







 







 











 
static __inline void SPII2S_ENABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == (0ul))
        i2s->I2SCTL |= (0x1ul << (16));
    else
        i2s->I2SCTL |= (0x1ul << (17));
}









 
static __inline void SPII2S_DISABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == (0ul))
        i2s->I2SCTL &= ~(0x1ul << (16));
    else
        i2s->I2SCTL &= ~(0x1ul << (17));
}






 







 







 







 







 







 







 







 







 







 







 







 










 
static __inline void SPII2S_SET_MONO_RX_CHANNEL(SPI_T *i2s, uint32_t u32Ch)
{
    u32Ch == (0x1ul << (23)) ?
    (i2s->I2SCTL |= (0x1ul << (23))) :
    (i2s->I2SCTL &= ~(0x1ul << (23)));
}







 







 








 









 







 







 




 
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);

uint32_t SPII2S_Open(SPI_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat);
void SPII2S_Close(SPI_T *i2s);
void SPII2S_EnableInt(SPI_T *i2s, uint32_t u32Mask);
void SPII2S_DisableInt(SPI_T *i2s, uint32_t u32Mask);
uint32_t SPII2S_EnableMCLK(SPI_T *i2s, uint32_t u32BusClock);
void SPII2S_DisableMCLK(SPI_T *i2s);
void SPII2S_SetFIFO(SPI_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);


   

   

   







 
#line 576 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"









 












 



 



 
 
 
 


 
 
 
#line 72 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"

 
 
 





 
 
 
#line 92 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"

 
 
 
#line 103 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"

 
 
 



 
 
 



   




 


 
typedef struct
{
    uint32_t u32Year;            
    uint32_t u32Month;           
    uint32_t u32Day;             
    uint32_t u32DayOfWeek;       
    uint32_t u32Hour;            
    uint32_t u32Minute;          
    uint32_t u32Second;          
    uint32_t u32TimeScale;       
    uint32_t u32AmPm;            
} S_RTC_TIME_DATA_T;

   




 











 











 











 












 












 











 











 











 











 












 


void RTC_Open(S_RTC_TIME_DATA_T *psPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX10000);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDateMask(uint8_t u8IsTenYMsk, uint8_t u8IsYMsk, uint8_t u8IsTenMMsk, uint8_t u8IsMMsk, uint8_t u8IsTenDMsk, uint8_t u8IsDMsk);
void RTC_SetAlarmTimeMask(uint8_t u8IsTenHMsk, uint8_t u8IsHMsk, uint8_t u8IsTenMMsk, uint8_t u8IsMMsk, uint8_t u8IsTenSMsk, uint8_t u8IsSMsk);
uint32_t RTC_GetDayOfWeek(void);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);

   

   

   







 
#line 577 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\hdiv.h"
 








 











 



 



 











 
static __inline int32_t HDIV_Div(int32_t x, int16_t y)
{
    uint32_t *p32;

    p32 = (uint32_t *)((( uint32_t)0x40000000) + 0x14000);
    *p32++ = x;
    *p32++ = y;
    return *p32;
}











 
static __inline int16_t HDIV_Mod(int32_t x, int16_t y)
{
    uint32_t *p32;

    p32 = (uint32_t *)((( uint32_t)0x40000000) + 0x14000);
    *p32++ = x;
    *p32++ = y;
    return p32[1];
}

   

   

   







 


#line 578 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\timer.h"
 








 











 



 



 

 
 
 


































   




 















 














 













 















 












 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (30));
}











 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (30));
}













 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (23));
}











 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (23));
}











 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (6));
}











 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (6));
}











 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (7));
}











 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (7));
}











 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (29));
}











 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (29));
}











 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (5));
}











 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (5));
}












 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return ((timer->INTSTS & (0x1ul << (0))) ? 1 : 0);
}











 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (0));
}












 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->EINTSTS;
}











 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->EINTSTS = (0x1ul << (0));
}












 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->INTSTS & (0x1ul << (1)) ? 1 : 0);
}











 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (1));
}











 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->CAP;
}











 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->CNT;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);
void TIMER_EnableFreqCounter(TIMER_T *timer,
                             uint32_t u32DropCount,
                             uint32_t u32Timeout,
                             uint32_t u32EnableInt);
void TIMER_DisableFreqCounter(TIMER_T *timer);
void TIMER_SetTriggerSource(TIMER_T *timer, uint32_t u32Src);
void TIMER_SetTriggerTarget(TIMER_T *timer, uint32_t u32Mask);

   

   

   







 
#line 579 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\uart.h"






 












 



 



 

 
 
 

#line 42 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\uart.h"

 
 
 











 
 
 
















 
 
 




 
 
 




 
 
 







 
 
 




   




 












 













 













 












 













 













 














 












 













 













 













 













 













 












 

























 

























 









































 












 













 


 
static __inline void UART_CLEAR_RTS(UART_T *uart);
static __inline void UART_SET_RTS(UART_T *uart);










 
static __inline void UART_CLEAR_RTS(UART_T *uart)
{
    uart->MODEM |= (0x1ul << (9));
    uart->MODEM &= ~(0x1ul << (1));
}










 
static __inline void UART_SET_RTS(UART_T *uart)
{
    uart->MODEM |= (0x1ul << (9)) | (0x1ul << (1));
}


void UART_ClearIntFlag(UART_T *uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T *uart);
void UART_DisableFlowCtrl(UART_T *uart);
void UART_DisableInt(UART_T  *uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T *uart);
void UART_EnableInt(UART_T  *uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T *uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T *uart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T *uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T *uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T *uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T *uart, uint32_t u32Mode, uint32_t u32Addr);
uint32_t UART_Write(UART_T *uart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void UART_SelectSingleWireMode(UART_T *uart);



   

   

   







 
#line 580 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 








  











 



 



 
typedef struct s_usbd_info
{
    uint8_t *gu8DevDesc;             
    uint8_t *gu8ConfigDesc;          
    uint8_t **gu8StringDesc;         
    uint8_t **gu8HidReportDesc;      
    uint8_t *gu8BosDesc;             
    uint32_t *gu32HidReportSize;     
    uint32_t *gu32ConfigHidDescIdx;  

} S_USBD_INFO_T;   

extern const S_USBD_INFO_T gsInfo;

   






 



#line 65 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 
 




 
#line 84 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 
#line 97 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 



 



 
#line 117 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 







 


 

 
 
 

#line 145 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"







#line 167 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

#line 174 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"












   




 











 













 












 











 











 











 











 











 











 











 














 











 














 











 















 












 











 












 












 













 











 













 













 











 











 











 












 















 
static __inline void USBD_MemCopy(uint8_t dest[], uint8_t src[], uint32_t size)
{
    uint32_t volatile i=0ul;

    while(size--)
    {
        dest[i] = src[i];
        i++;
    }
}










 
static __inline void USBD_SetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 8ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg | (0x1ul << (1)));
            break;
        }
    }
}









 
static __inline void USBD_ClearStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 8ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg & ~(0x1ul << (1)));
            break;
        }
    }
}











 
static __inline uint32_t USBD_GetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 8ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFGP;  
            break;
        }
    }

    return ((*((volatile uint32_t *)(u32CfgAddr))) & (0x1ul << (1)));
}


extern volatile uint8_t g_usbd_RemoteWakeupEn;


typedef void (*VENDOR_REQ)(void);            
typedef void (*CLASS_REQ)(void);             
typedef void (*SET_INTERFACE_REQ)(uint32_t u32AltInterface);     
typedef void (*SET_CONFIG_CB)(void);        


 
void USBD_Open(const S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface);
void USBD_Start(void);
void USBD_GetSetupPacket(uint8_t *buf);
void USBD_ProcessSetupPacket(void);
void USBD_StandardRequest(void);
void USBD_PrepareCtrlIn(uint8_t pu8Buf[], uint32_t u32Size);
void USBD_CtrlIn(void);
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlOut(void);
void USBD_SwReset(void);
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq);
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback);
void USBD_LockEpStall(uint32_t u32EpBitmap);

   

   

   







 
#line 581 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usci_i2c.h"
 





 











 



 



 

 
 
 
enum UI2C_MASTER_EVENT
{
    MASTER_SEND_ADDRESS = 10u,     
    MASTER_SEND_H_WR_ADDRESS,     
    MASTER_SEND_H_RD_ADDRESS,     
    MASTER_SEND_L_ADDRESS,        
    MASTER_SEND_DATA,             
    MASTER_SEND_REPEAT_START,     
    MASTER_READ_DATA,             
    MASTER_STOP,                  
    MASTER_SEND_START             
};

 
 
 
enum UI2C_SLAVE_EVENT
{
    SLAVE_ADDRESS_ACK = 100u,       
    SLAVE_H_WR_ADDRESS_ACK,        
    SLAVE_L_WR_ADDRESS_ACK,        
    SLAVE_GET_DATA,                
    SLAVE_SEND_DATA,               
    SLAVE_H_RD_ADDRESS_ACK,        
    SLAVE_L_RD_ADDRESS_ACK         
};

 
 
 





 
 
 



 
 
 



 
 
 
#line 89 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usci_i2c.h"

   




 











 











 











 











 












 












 












 











 











 











 











 

















 

















 

















 



uint32_t UI2C_Open(UI2C_T *ui2c, uint32_t u32BusClock);
void UI2C_Close(UI2C_T *ui2c);
void UI2C_ClearTimeoutFlag(UI2C_T *ui2c);
void UI2C_Trigger(UI2C_T *ui2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Ptrg, uint8_t u8Ack);
void UI2C_DisableInt(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_EnableInt(UI2C_T *ui2c, uint32_t u32Mask);
uint32_t UI2C_GetBusClockFreq(UI2C_T *ui2c);
uint32_t UI2C_SetBusClockFreq(UI2C_T *ui2c, uint32_t u32BusClock);
uint32_t UI2C_GetIntFlag(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_ClearIntFlag(UI2C_T *ui2c , uint32_t u32Mask);
uint32_t UI2C_GetData(UI2C_T *ui2c);
void UI2C_SetData(UI2C_T *ui2c, uint8_t u8Data);
void UI2C_SetSlaveAddr(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddr, uint8_t u8GCMode);
void UI2C_SetSlaveAddrMask(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddrMask);
void UI2C_EnableTimeout(UI2C_T *ui2c, uint32_t u32TimeoutCnt);
void UI2C_DisableTimeout(UI2C_T *ui2c);
void UI2C_EnableWakeup(UI2C_T *ui2c, uint8_t u8WakeupMode);
void UI2C_DisableWakeup(UI2C_T *ui2c);
uint8_t UI2C_WriteByte(UI2C_T *ui2c, uint8_t u8SlaveAddr, const uint8_t data);
uint32_t UI2C_WriteMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, const uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, const uint8_t data);
uint32_t UI2C_WriteMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, const uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t data);
uint32_t UI2C_WriteMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_ReadByte(UI2C_T *ui2c, uint8_t u8SlaveAddr);
uint32_t UI2C_ReadMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t UI2C_ReadMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t UI2C_ReadMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *rdata, uint32_t u32rLen);

   

   

   







 
#line 582 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usci_spi.h"
 





 











 



 



 













 
#line 52 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usci_spi.h"

 
#line 60 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usci_spi.h"

   




 






 







 









 









 









 







 








 








 












 












 







 







 








 
#line 197 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usci_spi.h"








 









 







 







 
















 







 










 












 












 










 










 












 












 








 








 








 








 


uint32_t USPI_Open(USPI_T *uspi, uint32_t u32MasterSlave, uint32_t u32SPIMode,  uint32_t u32DataWidth, uint32_t u32BusClock);
void USPI_Close(USPI_T *uspi);
void USPI_ClearRxBuf(USPI_T *uspi);
void USPI_ClearTxBuf(USPI_T *uspi);
void USPI_DisableAutoSS(USPI_T *uspi);
void USPI_EnableAutoSS(USPI_T *uspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t USPI_SetBusClock(USPI_T *uspi, uint32_t u32BusClock);
uint32_t USPI_GetBusClock(USPI_T *uspi);
void USPI_EnableInt(USPI_T *uspi, uint32_t u32Mask);
void USPI_DisableInt(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetIntFlag(USPI_T *uspi, uint32_t u32Mask);
void USPI_ClearIntFlag(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetStatus(USPI_T *uspi, uint32_t u32Mask);
void USPI_EnableWakeup(USPI_T *uspi);
void USPI_DisableWakeup(USPI_T *uspi);


   

   

   







 
#line 583 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usci_uart.h"
 





 












 



 



 

 
 
 





 
 
 







 
 
 
#line 58 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\usci_uart.h"


   




 












 












 













 













 














 














 












 













 













 













 













 















 















 














 














 

















 

















 












 






















 












 














 













 












 












 












 












 












 



void UUART_ClearIntFlag(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_GetIntFlag(UUART_T* uuart, uint32_t u32Mask);
void UUART_Close(UUART_T* uuart);
void UUART_DisableInt(UUART_T*  uuart, uint32_t u32Mask);
void UUART_EnableInt(UUART_T*  uuart, uint32_t u32Mask);
uint32_t UUART_Open(UUART_T* uuart, uint32_t u32baudrate);
uint32_t UUART_Read(UUART_T* uuart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
uint32_t UUART_SetLine_Config(UUART_T* uuart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t u32stop_bits);
uint32_t UUART_Write(UUART_T* uuart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void UUART_EnableWakeup(UUART_T* uuart, uint32_t u32WakeupMode);
void UUART_DisableWakeup(UUART_T* uuart);
void UUART_EnableFlowCtrl(UUART_T* uuart);
void UUART_DisableFlowCtrl(UUART_T* uuart);


   

   

   







 
#line 584 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\wdt.h"
 








 











 



 



 
 
 
 
#line 43 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\wdt.h"

 
 
 





 
 
 


   




 










 











 











 












 












 












 














 


static __inline void WDT_Close(void);
static __inline void WDT_EnableInt(void);
static __inline void WDT_DisableInt(void);








 
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL = 0UL;
    while(((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL & (0x1ul << (30))); 
    return;
}









 
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL |= (0x1ul << (6));
    while(((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL & (0x1ul << (30))); 
    return;
}









 
static __inline void WDT_DisableInt(void)
{
     
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL &= ~((0x1ul << (6)) | (0x1ul << (2)) | (0x1ul << (3)) | (0x1ul << (5)));
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

   

   

   







 
#line 585 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"
#line 1 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\wwdt.h"
 








 











 



 



 
 
 
 
#line 50 "..\\..\\..\\..\\..\\..\\Library\\StdDriver\\inc\\wwdt.h"

 
 
 


   




 










 











 












 












 











 














 


void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);

   

   

   







 
#line 586 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\M031Series.h"



 
#line 12 "..\\..\\..\\..\\..\\..\\Library\\Device\\Nuvoton\\M031\\Include\\NuMicro.h"



 
#line 6 "..\\..\\..\\porting\\mcu_definition.h"



 
#line 16 "..\\..\\..\\porting\\mcu_definition.h"






#line 31 "..\\..\\..\\porting\\mcu_definition.h"

 









#line 5 "..\\..\\FT_Demo\\BleAppSetting.h"



 

















#line 32 "..\\..\\FT_Demo\\BleAppSetting.h"
























#line 22 "..\\..\\..\\source\\LL.c"
#line 1 "..\\..\\..\\include\\rf_phy.h"
 







 




#line 15 "..\\..\\..\\include\\rf_phy.h"
#line 1 "..\\..\\..\\include\\ble_cmd.h"
 








 




#line 1 "..\\..\\..\\include\\host.h"
 








 




#line 16 "..\\..\\..\\include\\host.h"
#line 1 "..\\..\\..\\include\\ble_basicType.h"
 








 









 
typedef unsigned char           Uint8;
typedef unsigned short int      Uint16;
typedef unsigned int            Uint32;
typedef signed char             Sint8;
typedef signed short int        Sint16;
typedef signed int              Sint32;

 

 
 
 





 








 





#line 17 "..\\..\\..\\include\\host.h"
#line 1 "..\\..\\..\\include\\ble_gapDef.h"
 








 







 








 
#line 44 "..\\..\\..\\include\\ble_gapDef.h"
 







 



#line 81 "..\\..\\..\\include\\ble_gapDef.h"
 







 


#line 99 "..\\..\\..\\include\\ble_gapDef.h"
 


#line 18 "..\\..\\..\\include\\host.h"
#line 1 "..\\..\\..\\include\\ble_uuid.h"
 








 


















 







 







 

#line 57 "..\\..\\..\\include\\ble_uuid.h"


 
  













 

#line 105 "..\\..\\..\\include\\ble_uuid.h"

 






 







 







 
#line 136 "..\\..\\..\\include\\ble_uuid.h"
 


 














 

#line 257 "..\\..\\..\\include\\ble_uuid.h"


 








 

#line 280 "..\\..\\..\\include\\ble_uuid.h"


 







 
#line 298 "..\\..\\..\\include\\ble_uuid.h"
 

   










 

#line 421 "..\\..\\..\\include\\ble_uuid.h"

 




#line 19 "..\\..\\..\\include\\host.h"
#line 1 "..\\..\\..\\include\\ble_att_gatt.h"
 











 











 

#line 35 "..\\..\\..\\include\\ble_att_gatt.h"
 







 


 







 


 







 

 







 

 








 
#line 115 "..\\..\\..\\include\\ble_att_gatt.h"
 







 

 








 

#line 150 "..\\..\\..\\include\\ble_att_gatt.h"
 







 


 






 




 







 
#line 204 "..\\..\\..\\include\\ble_att_gatt.h"
 








 
#line 242 "..\\..\\..\\include\\ble_att_gatt.h"
 









 
#line 288 "..\\..\\..\\include\\ble_att_gatt.h"
 



#line 20 "..\\..\\..\\include\\host.h"
#line 1 "..\\..\\..\\include\\ble_serviceDef.h"
 








 














 
#line 50 "..\\..\\..\\include\\ble_serviceDef.h"
 












 
#line 70 "..\\..\\..\\include\\ble_serviceDef.h"
 














 

#line 167 "..\\..\\..\\include\\ble_serviceDef.h"
 








 
#line 187 "..\\..\\..\\include\\ble_serviceDef.h"
 









 

#line 310 "..\\..\\..\\include\\ble_serviceDef.h"
 








 

#line 409 "..\\..\\..\\include\\ble_serviceDef.h"
 








 

#line 570 "..\\..\\..\\include\\ble_serviceDef.h"
 







 
#line 629 "..\\..\\..\\include\\ble_serviceDef.h"
 








 
#line 651 "..\\..\\..\\include\\ble_serviceDef.h"
 











extern const Uint8 ATT_HDL_INVALID[];
extern const Uint8 ATT_HDL_GAP_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_DEVICE_NAME[];
extern const Uint8 ATT_HDL_GAP_DEVICE_NAME_INIT[];
extern Uint8 att_HDL_GAP_DEVICE_NAME[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_APPEARANCE[];
extern const Uint8 ATT_HDL_GAP_APPEARANCE[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_RECONNECTION_ADDRESS[];
extern const Uint8 ATT_HDL_GAP_RECONNECTION_ADDRESS_INIT[];
extern Uint8 att_HDL_GAP_RECONNECTION_ADDRESS[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG[];
extern const Uint8 ATT_HDL_GAP_PERIPHERAL_PRIVACY_FLAG[];
extern const Uint8 ATT_HDL_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[];
extern const Uint8 ATT_HDL_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[];

extern const Uint8 ATT_HDL_GATT_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_GATT_CHARACTERISTIC_SERVICE_CHANGED[];
extern const Uint8 ATT_HDL_GATT_SERVICE_CHANGED_INIT[];
extern Uint8 att_HDL_GATT_SERVICE_CHANGED[];



extern const Uint8 ATT_HDL_DIS_PRIMARY_SERVICE[];
extern const Uint8 ATT_HDL_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING[];
extern const Uint8 ATT_HDL_DIS_SERIAL_NUMBER_STRING[];
extern const Uint8 ATT_HDL_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT[];
extern const Uint8 ATT_HDL_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING[];
extern const Uint8 ATT_HDL_DIS_MANUFACTURER_NAME_STRING[];
extern const Uint8 ATT_HDL_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT[];
extern const Uint8 ATT_HDL_DIS_CHARACTERISTIC_SYSTEM_ID[];
extern const Uint8 ATT_HDL_DIS_SYSTEM_ID[];
extern const Uint8 ATT_HDL_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING[];
extern const Uint8 ATT_HDL_DIS_FIRMWARE_REVISION_STRING[];
extern const Uint8 ATT_HDL_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT[];


#line 783 "..\\..\\..\\include\\ble_serviceDef.h"

#line 807 "..\\..\\..\\include\\ble_serviceDef.h"

#line 828 "..\\..\\..\\include\\ble_serviceDef.h"

#line 854 "..\\..\\..\\include\\ble_serviceDef.h"

#line 894 "..\\..\\..\\include\\ble_serviceDef.h"

#line 905 "..\\..\\..\\include\\ble_serviceDef.h"

#line 921 "..\\..\\..\\include\\ble_serviceDef.h"


#line 1018 "..\\..\\..\\include\\ble_serviceDef.h"


















































#line 21 "..\\..\\..\\include\\host.h"
#line 1 "..\\..\\..\\include\\blestack_status.h"
 








 















 
typedef Uint8 BleStackStatus;

 


 


 


 


 


 


 


 


 


 


 


 
 


#line 22 "..\\..\\..\\include\\host.h"



 


 







 



 













 

 






 


 





 


 






 


 






 
#line 100 "..\\..\\..\\include\\host.h"
 






 
#line 115 "..\\..\\..\\include\\host.h"
 







 
#line 275 "..\\..\\..\\include\\host.h"
 






 
#line 290 "..\\..\\..\\include\\host.h"
 






 
#line 305 "..\\..\\..\\include\\host.h"
 






 
#line 320 "..\\..\\..\\include\\host.h"
 






 
#line 335 "..\\..\\..\\include\\host.h"
 






 
#line 350 "..\\..\\..\\include\\host.h"
 







 
#line 366 "..\\..\\..\\include\\host.h"
 







 
#line 382 "..\\..\\..\\include\\host.h"
 







 
#line 398 "..\\..\\..\\include\\host.h"
 







 
#line 414 "..\\..\\..\\include\\host.h"
 







 
#line 430 "..\\..\\..\\include\\host.h"
 






 
#line 445 "..\\..\\..\\include\\host.h"
 







 
#line 460 "..\\..\\..\\include\\host.h"
 






 
#line 474 "..\\..\\..\\include\\host.h"
 







 
#line 489 "..\\..\\..\\include\\host.h"
 







 
#line 504 "..\\..\\..\\include\\host.h"
 






 
#line 518 "..\\..\\..\\include\\host.h"

 


 









 
#line 558 "..\\..\\..\\include\\host.h"
 











 


















 
#line 622 "..\\..\\..\\include\\host.h"

#line 634 "..\\..\\..\\include\\host.h"

 




#line 648 "..\\..\\..\\include\\host.h"















 








 
#line 694 "..\\..\\..\\include\\host.h"
 






 




 

extern const Uint8 *ATTRIBUTE_SERVER[];              
extern const Uint8 *ATTRIBUTE_SERVER_PARAM[];        
extern const Uint8 *ATTRIBUTE_SERVER_PERMISSION[];   


extern const Uint8 *ATTRIBUTE_SERVER_BOND[(0+1+0+0+0+0+0+0+0+0+0+0+0+0+0+0+0+0+0)];             
extern const Uint8 *ATTRIBUTE_SERVER_BOND_PARAM[(0+1+0+0+0+0+0+0+0+0+0+0+0+0+0+0+0+0+0)];       



extern Uint8 size_ATTRIBUTE_SERVER;                                                


extern Uint8 size_ATTRIBUTE_SERVER_BOND;                                           


 



 





 














 
BleStackStatus ATT_HDL_Notify(uint8_t connID, uint8_t *addrATT_HDL_INIT, uint8_t *addrATT_HDL_clientCFG, uint8_t *data, uint8_t dataLength);















 
BleStackStatus ATT_HDL_Indicate(uint8_t connID, uint8_t *addrATT_HDL_INIT, uint8_t *addrATT_HDL_clientCFG, uint8_t *data, uint8_t dataLength);










 
void ATT_HDL_ValueSet(uint8_t *addr_attHDL, uint8_t *data, uint8_t dataLength);

#line 16 "..\\..\\..\\include\\ble_cmd.h"
#line 17 "..\\..\\..\\include\\ble_cmd.h"



 




















 









 
typedef Uint8 BleCmdEvent;

 




 



 





 




 







 






 





 





 





 





 






 






 

 
 





 

 
 



 

 
 




 

 
 




 

 
 




 

 
 









 
typedef Uint8 BleDisConnReason;
#line 195 "..\\..\\..\\include\\ble_cmd.h"
 
 












 
typedef Uint8 BleConnRole;


 
 










 
typedef Uint8 BleAdvChannelMap;

 


 


 


 


 


 


 

 
 










 
typedef Uint8 BleAdvFilterPolicy;

 


 


 


 

 
 











 
typedef Uint8 BleAdvIntervalRange;



 




 

 
 










 
typedef Uint8 BleAdvType;






 
 











 
typedef Uint8 BleScanType;


 
 











 
typedef Uint8 BleScanIntervalRange;



 




 

 
 










 
typedef Uint8 BleScanWindowRange;



 




 

 
 










 
typedef Uint8 BleScanFilterPolicy;

 


 

 
 










 
typedef Uint8 BleConnectionIntervalRange;



 




 

 
 










 
typedef Uint8 BleConnectionLatencyRange;

 


 

 
 










 
typedef Uint8 BleConnectionSupTimeoutRange;



 




 

 
 










 
typedef Uint8 BleAddrType;





 
 










 
typedef Uint8 BlePhy;


 
 











 
typedef Sint8 BleTxPowerLevel;



 
 











 
typedef Uint8 BleMode;





 
 










 
typedef Uint8 BleRF_Mode;


 
 









 
typedef Uint8 BleSTK_GenMethod;


 
 










 
typedef Uint8 BleConnUpdateStatus;



 
 










 
typedef Uint8 BleAuthStatus;
#line 625 "..\\..\\..\\include\\ble_cmd.h"
 
 











 
typedef Uint8 BlePhyUpdateStatus;




 
 











 
typedef Uint8 BleAdvCompleteStatus;



 
 











 
typedef Uint8 IOCaps;





 
 










 
typedef Uint8 BondingFlags;


 
 









 
typedef Uint8 BleConnStatus;





 
 


 





 




 
typedef struct BLE_Addr_Param
{

    

 
    BleAddrType addrType;

     
    Uint8 addr[6];

} BLE_Addr_Param;






 
typedef struct BLE_Adv_Param
{
    

 
    BleAdvType Adv_Type;

    


 
    Uint16 Adv_Interval_Min;

    


 
    Uint16 Adv_Interval_Max;


     
    BLE_Addr_Param Adv_DirectAddr_Param;

    

 
    BleAdvChannelMap    Adv_Channel_Map;

    

 
    BleAdvFilterPolicy  Adv_Filter_Policy;
} BLE_Adv_Param;





 
typedef struct BLE_Scan_Param
{

    

 
    BleScanType           Scan_Type;

    


 
    Uint16                Scan_Interval;

    


 
    Uint16                Scan_Window;

    

 
    BleScanFilterPolicy   Scan_FilterPolicy;

} BLE_Scan_Param;





 
typedef struct BLE_Conn_Param
{

    


 
    Uint16 Conn_IntervalMin;

    


 
    Uint16 Conn_IntervalMax;

    

 
    Uint16 Conn_Latency;

    


 
    Uint16 Conn_SvisionTimeout;

} BLE_Conn_Param;




 
typedef struct BLE_ConnUpdate_Param
{
    Uint8           connId;            
    BLE_Conn_Param  connUpdateParam;   

} BLE_ConnUpdate_Param;




 
typedef struct BLE_IO_Caps
{
    IOCaps          IO_Param;          

} BLE_IO_Caps;






 
typedef struct BLE_Bonding_Flags
{
    BondingFlags         Flags;          

} BLE_Bonding_Flags;






 
typedef struct BLE_DataLength_Param
{
    uint8_t           connId;            
    uint16_t          TxMaxOctets;       
    uint16_t          RxMaxOctets;       

} BLE_DataLength_Param;





 
typedef struct BLE_Phy_Param
{
    BlePhy             tx_Phy;    
    BlePhy             rx_Phy;    

} BLE_Phy_Param;







 
typedef struct BLE_Event_ADV_Complete_Param
{
    BleAdvCompleteStatus  status;                

} BLE_Event_ADV_Complete_Param;







 
typedef struct BLE_Event_ConnParam
{
    BleConnStatus      status;                
    Uint8              connId;                
    BleConnRole        connRole;              
    BLE_Addr_Param     peerAddr;              
    Uint16             connInterval;          
    Uint16             connLatency;           
    Uint16             supervisionTimeout;    

} BLE_Event_ConnParam;





 
typedef struct BLE_Event_DisconnParam
{
    Uint8              connId;                
    BleDisConnReason   disconnectReason;      

} BLE_Event_DisconnParam;







 
typedef struct BLE_Event_ConnUpdateParam
{
    Uint8               connId;                
    BleConnUpdateStatus status;                
    Uint16              connInterval;          
    Uint16              connLatency;           
    Uint16              supervisionTimeout;    

} BLE_Event_ConnUpdateParam;






 
typedef struct BLE_Event_Rssi_Param
{
    Uint8 connID;  
    Sint8 rssi;    
} BLE_Event_Rssi_Param;








 
typedef struct BLE_Event_Stk_Gen_Method
{
    Uint8              connId;           
    BleSTK_GenMethod   Stk_Gen_Method;   

} BLE_Event_Stk_Gen_Method;







 
typedef struct BLE_Event_PassKey_Confirm
{
    Uint8              connId;    
} BLE_Event_PassKey_Confirm;







 
typedef struct BLE_Event_Auth_Status
{
    Uint8              connId;    
    BleAuthStatus      status;    

} BLE_Event_Auth_Status;





 
typedef struct BLE_Event_Phy_Param
{
    Uint8              connId;    
    BlePhy             tx_Phy;    
    BlePhy             rx_Phy;    

} BLE_Event_Phy_Param;





 
typedef struct BLE_Event_Phy_Update_Param
{
    BlePhyUpdateStatus     status;     
    BLE_Event_Phy_Param    phyParam;   

} BLE_Event_Phy_Update_Param;






 
typedef struct BLE_Event_Scan_Report
{
    BleAdvType      Rpt_Type;                  
    BLE_Addr_Param  Rpt_peerAddr;              
    Uint8           Rpt_DataLength;            
    Uint8           Rpt_Data[31];              
    Sint8           Rpt_Rssi;                  

} BLE_Event_Scan_Report;






 
typedef struct BLE_Event_CreateConn_Param
{
    BleConnStatus      status;   

} BLE_Event_CreateConn_Param;






 
typedef struct BLE_Event_Mtu
{
    uint8_t      mtu_size;

} BLE_Event_Mtu;






 
typedef struct BLE_Event_Data_Length_Param
{
    BLE_DataLength_Param    param;

} BLE_Event_Data_Length_Param;


 





 




















 
BleStackStatus setBLE_BleDeviceAddr(BLE_Addr_Param *addrParam);













 
void getBLE_BleDeviceAddr(BLE_Addr_Param *addrParam);








 
void setBLE_WhiteList(void);

















 
BleStackStatus setBLE_AdvParam(BLE_Adv_Param advParam);














 
BleStackStatus setBLE_AdvData(Uint8 *advData, Uint8 length);














 
BleStackStatus setBLE_ScanRspData(Uint8 *scanRspData, Uint8 length);











 
BleStackStatus setBLE_AdvEnable(void);











 
BleStackStatus setBLE_AdvDisable(void);















 
BleStackStatus setBLE_ScanParam(BLE_Scan_Param scanParam);












 
BleStackStatus setBLE_ScanEnable(BleScanFilterPolicy scanFilter);










 
BleStackStatus setBLE_ScanDisable(void);












 
BleStackStatus setBLE_Disconnect(Uint8 connID);















 
BleStackStatus setBLE_ConnCreate(BLE_Addr_Param peerAddrParam, BLE_Scan_Param scanParam,BLE_Conn_Param connParam);












 
BleStackStatus setBLE_ConnCancel(void);





















 
BleStackStatus setBLE_ConnUpdate(BLE_ConnUpdate_Param connParam);
























 
BleStackStatus setBLE_TxPower(BleTxPowerLevel txPower, BleMode bleMode);
















 
BleStackStatus setBLE_Phy(Uint8 connID, BLE_Phy_Param blePhy);













 
BleStackStatus getBLE_Phy(Uint8 connID);













 
BleStackStatus getBLE_RssiValue(Uint8 connID);










 
void setBLE_RssiValueOffset(uint8_t offset);

















 
BleStackStatus setBLE_SecurityRequest(Uint8 connID);















 
BleStackStatus setBLE_Pairing_PassKey(Uint8 connID, Uint32 hexPasskey);














 
BleStackStatus setBLE_IOCapabilities(BLE_IO_Caps *Param);













 
BleStackStatus setBLE_BondingFlags(BLE_Bonding_Flags *Param);















 
BleStackStatus setBLE_DataLength(BLE_DataLength_Param *Param);















 
BleStackStatus setBLE_exchangeMtuRequest(uint8_t connID, uint8_t client_rx_mtu);





#line 16 "..\\..\\..\\include\\rf_phy.h"




 






 




 


 





 






 










 


 









 





 































 





 





 







 






 





 
void RF_Init(void);











 
BleStackStatus Ble_Kernel_Root(void);






 
uint32_t LLTimeline_Get(void); 










 
uint8_t ChipId_Get(void);










 
uint32_t BleTxFIFOAddr_Get(void);











 
BleRF_Mode BleRFMode_Get(void);














 
BleStackStatus setRF_Enable16MHzOutput(void);











 
BleStackStatus setRF_EnterDeepSleep(void);











 
BleStackStatus setRF_WakeUpFromDeepSleep(void);
















 
BleStackStatus setMCU_WakeupRetentionTime(uint8_t retentionTime);

#line 23 "..\\..\\..\\source\\LL.c"
#line 1 "..\\..\\..\\include\\rf_include\\knl_pblc.h"
















 



 




#line 27 "..\\..\\..\\include\\rf_include\\knl_pblc.h"
#line 28 "..\\..\\..\\include\\rf_include\\knl_pblc.h"
#line 1 "..\\..\\..\\include\\rf_include\\_ble_host.h"
 







 





#line 16 "..\\..\\..\\include\\rf_include\\_ble_host.h"
#line 17 "..\\..\\..\\include\\rf_include\\_ble_host.h"
#line 18 "..\\..\\..\\include\\rf_include\\_ble_host.h"


 





 










 















#line 61 "..\\..\\..\\include\\rf_include\\_ble_host.h"



#line 71 "..\\..\\..\\include\\rf_include\\_ble_host.h"


#line 93 "..\\..\\..\\include\\rf_include\\_ble_host.h"



#line 104 "..\\..\\..\\include\\rf_include\\_ble_host.h"










#line 122 "..\\..\\..\\include\\rf_include\\_ble_host.h"














#line 147 "..\\..\\..\\include\\rf_include\\_ble_host.h"































#line 186 "..\\..\\..\\include\\rf_include\\_ble_host.h"



#line 195 "..\\..\\..\\include\\rf_include\\_ble_host.h"


#line 208 "..\\..\\..\\include\\rf_include\\_ble_host.h"

typedef Uint8 CmdTimerEvent;
 








 
typedef struct BLE_Device_Param
{
    uint8_t         ble_deviceChipId;      
    BLE_Addr_Param  ble_deviceAddr_param;  
} BLE_Device_Param;

extern BLE_Device_Param ble_device_param;



#line 246 "..\\..\\..\\include\\rf_include\\_ble_host.h"

























#line 279 "..\\..\\..\\include\\rf_include\\_ble_host.h"





#line 299 "..\\..\\..\\include\\rf_include\\_ble_host.h"

#line 308 "..\\..\\..\\include\\rf_include\\_ble_host.h"



#line 321 "..\\..\\..\\include\\rf_include\\_ble_host.h"

















#line 350 "..\\..\\..\\include\\rf_include\\_ble_host.h"


extern const Uint8 RSP_PAIRING_REQUEST[];
extern const Uint8 SKDS_FIXED[];
extern const Uint8 IVS_FIXED[];
extern const Uint8 LTK_FIXED[16];
extern const Uint8 EDIV_FIXED[2];
extern const Uint8 RAND_FIXED[8];
extern const Uint8 IRK_FIXED[][16];
extern const Uint8 CSRK_FIXED[][16];
extern const Uint8 SEL_STK_GEN_MTHD[5][5];

extern const Uint8 k_AES[16];
extern const Uint8 r_AES_INI[16];
extern const Uint8 P1_AES[16];
extern const Uint8 P2_AES[16];




extern Uint8 smp_Authorization;
extern Uint8 smp_Encryption;
extern Uint8 smp_Phase;
extern Uint8 smp_TempDef;
extern Uint8 smp_Temp16[16];
extern Uint8 smp_Rand[8];
extern Uint8 smp_EDIV[2];
extern Uint8 smp_IRKm[16];
extern Uint8 smp_CSRKm[16];
extern Uint8 smp_SKDm[8];
extern Uint8 smp_IVm[4];
extern Uint8 smp_Security_Mode;

extern Uint8 smp_PReq_IO_Capab;
extern Uint8 smp_PReq_OOB_data;
extern Uint8 smp_PReq_Init_Key_Distr;
extern Uint8 smp_PReq_Resp_Key_Distr;

extern Uint8 k_AES_passkey[16];
extern Uint8 r_AES[16];
extern Uint8 LTK[16];
extern Uint8 LTK_SEED[8+2];
extern Uint8 smp_Para_Bond[(36+1)];


extern Uint8 smp_Para_Bond_tmp[70];
extern Uint8 SECBuffer[42];
extern Uint8 rsp_pairing_request_table[11];


extern void smpC1(Uint8 * k, Uint8 * r, Uint8 * smpC1_out);
extern void smpS1(Uint8 * k, Uint8 * r1, Uint8 * r2, Uint8 * smpS1_out);
extern void smpPasskey_init(void);
extern Uint8 smpPasskey_set(Uint32 hexPasskey, Uint8 connID);

extern Uint8 setBLE_ConnTxData_SMP(Uint8 connID, Uint8 L2CAP_Code_SMP, Uint8 * L2CAP_RspDataSMP, Uint8 Length);
extern Uint8 setBLE_ConnTxData_SMP_Isr(Uint8 connID, Uint8 L2CAP_Code_SMP, Uint8 * L2CAP_RspDataSMP, Uint8 Length);
extern Uint8 PassEC_BLE_CODE_SMP(Uint8 connID, Uint8 Primitive, Uint8 *source, Uint8 length);









extern Uint8 SPIBuffer[251];
extern Uint8 aTT_MTU[];

extern const Uint8 UUID_BLUETOOTH_BASE[16];
extern Uint8 q_att_HDL_PreWr[45+3];

extern Uint8 setBLE_ConnTxData(Uint8 connID, Uint8 * ScanRspData, Uint8 Length);



extern Uint8 setBLE_ConnTxData_ATT_ERROR_RESPONSE(Uint8 connID, Uint8 Request_Opcode_In_Error, Uint8 ATT_handle_in_error_L, Uint8 ATT_handle_in_error_H, Uint8 Error_code);

extern void init_q_att_HDL_PreWr(void);
extern void initBLE_AttServParamter(void);

extern Uint8 chkBLE_ATTRIBUTE_PERMISSION_Read(Uint8 ATT_Handle);
extern Uint8 chkBLE_ATTRIBUTE_PERMISSION_Write(Uint8 ATT_Handle);
extern Uint8 getBLE_ATTRIBUTE_TYPE_Size(Uint8 ATT_Handle);
extern Uint8 getBLE_ATTRIBUTE_Value_DynmcLngth(Uint8 AttHandle, Uint8 SizeIdx);
extern Uint8 chkIfBLE_ATT_Chr_Acss_ByUser(Uint8 AttHandle);
extern Uint8 chkBLE_ATT_Chr_AcssRd_ByUser(Uint8 *AddrAttHandle);
extern Uint8 chkBLE_ATT_Chr_AcssWr_ByUser(Uint8 *AddrAttHandle, Uint8 *wrDataBuf);






#line 29 "..\\..\\..\\include\\rf_include\\knl_pblc.h"


 
 
 
      
#line 45 "..\\..\\..\\include\\rf_include\\knl_pblc.h"














#line 66 "..\\..\\..\\include\\rf_include\\knl_pblc.h"









 


typedef struct CmdTimerMessage
{
    Uint8   Event;
    Uint32  Current_T;
    Uint32  TimeOut_Base;
} CmdTimerMessage;


typedef union Udat32
{
    Uint32 Val32;
    Uint16 Val16[2];
    Uint8 Val8[4];
} Udat32;


typedef union Udat16
{
    Uint16 Val16;
    Uint8 Val8[2];
} Udat16;

typedef struct TBLK_LL
{
    Uint8   TmrId;       
    Uint8   ConnId;       
    Uint8   Next;        
    Uint32  Ticks;       
} TBLK_LL;
typedef TBLK_LL TBLK_LLx;



#line 140 "..\\..\\..\\include\\rf_include\\knl_pblc.h"





struct HCLL_Disconnect_Para                              
{
    
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Reason;
};


struct HCLL_Read_Remote_Ver_Info_Para                    
{
    
    Uint8 HCI_Conn_Hdl_L;
};


struct HCLL_Set_Event_Mask_Para                          
{
    
    Uint8 HCI_Event_Mask[8];
};


struct HCLL_Reset_Para                                   
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_Read_Transmit_Pwr_Level_Para                 
{
    
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Type;
};


struct HCLL_Set_Ctrler_To_Host_Flow_Ctrl_Para            
{
    
    Uint8 HCI_FlowCtrlEn;
};


struct HCLL_Host_Buffer_Size_Para                        
{
    
    Uint8 HCI_HostACLDataPcktLengthL;
    Uint8 HCI_HostACLDataPcktLengthH;
    Uint8 HCI_HostSyncDataPcktLength;
    Uint8 HCI_HostTotalNumACLDataPcktL;
    Uint8 HCI_HostTotalNumACLDataPcktH;
    Uint8 HCI_HostTotalNumSyncDataPcktL;
    Uint8 HCI_HostTotalNumSyncDataPcktH;
};


struct HCLL_Host_Num_Of_Completed_Packets_Para           
{
    
    Uint8 HCI_Num_Of_Hdl;
    Uint8 HCI_Conn_Hdl_L;
    Uint16 HCI_HostNumCpltdPkt0;
};


struct HCLL_Set_Event_Mask_Page_2_Para                   
{
    
    Uint8 HCI_Event_Mask_Page2[8];
};


struct HCLL_Read_Le_Host_Support_Para                    
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_Write_Le_Host_Support_Para                   
{
    
    Uint8 HCI_LE_Supp_Host;
    Uint8 HCI_Simul_LE_Host;
};


struct HCLL_Read_Authen_Payload_Timeout_Para             
{
    
    Uint8 HCI_Conn_Hdl_L;
};


struct HCLL_Write_Authen_Payload_Timeout_Para            
{
    
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_AuthenPayloadTimeoutL;
    Uint8 HCI_AuthenPayloadTimeoutH;
};


struct HCLL_Read_Local_Ver_Info_Para                     
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_Read_Local_Supported_Cmd_Para                
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_Read_Local_Supported_Feat_Para               
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_Read_Buffer_Size_Para                        
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_Read_Bd_Addr_Para                            
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_Read_Rssi_Para                               
{
    
    Uint8 HCI_Conn_Hdl_L;
};


struct HCLL_LE_Set_Event_Mask_Para                       
{
    
    Uint8 HCI_LE_Event_Mask[8];
};


struct HCLL_LE_Read_Buffer_Size_Para                     
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Read_Local_Supported_Feat_Para            
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Set_Random_Addr_Para                   
{
    
    Uint8 HCI_Random_Addr[6];
};


struct HCLL_LE_Set_Adv_Param_Para                        
{
    
    Uint16 HCI_Adv_Interval_Min;
    Uint16 HCI_Adv_Interval_Max;
    Uint8 HCI_Adv_Type;
    Uint8 HCI_Own_Addr_Type;
    Uint8 HCI_Direct_Addr_Type;
    Uint8 HCI_Direct_Addr[6];
    Uint8 HCI_Adv_Channel_Map;
    Uint8 HCI_Adv_Filter_Policy;
};


struct HCLL_LE_Read_Adv_Ch_Tx_Pwr_Para                   
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Set_Adv_Data_Para                         
{
    
    Uint8 HCI_Adv_Data_Length;
    Uint8 HCI_Adv_Data[31];
};


struct HCLL_LE_Set_Scan_Response_Data_Para               
{
    
    Uint8 HCI_Scan_Rsp_Length;
    Uint8 HCI_Scan_Rsp[31];
};


struct HCLL_LE_Set_Advertise_Enable_Para                
{
    
    Uint8 HCI_Adv_En;
};


struct HCLL_LE_Set_Scan_Param_Para                      
{
    
    Uint8 HCI_LEScanType;
#line 373 "..\\..\\..\\include\\rf_include\\knl_pblc.h"
    Uint8 Padding0;
    Uint16 HCI_LE_Scan_Interval;
    Uint16 HCI_LE_Scan_Window;
    Uint8 HCI_Own_Addr_Type;
    Uint8 HCI_ScannFilterPolicy;

};


struct HCLL_LE_Set_Scan_Enable_Para                     
{
    
    Uint8 HCI_LE_Scan_En;
    Uint8 HCI_Filter_Dupl;
};


struct HCLL_LE_Create_Conn_Para                          
{
    
    Uint16 HCI_LE_Scan_Interval;
    Uint16 HCI_LE_Scan_Window;
    Uint8 HCI_InitFilterPolicy;
    Uint8 HCI_PeerAddrType;
    Uint8 HCI_PeerAddr[6];
    Uint8 HCI_Own_Addr_Type;
#line 407 "..\\..\\..\\include\\rf_include\\knl_pblc.h"
    Uint8 Padding0;
    Uint16 HCI_ConnIntervalMin;
    Uint16 HCI_ConnIntervalMax;
    Uint16 HCI_ConnLatency;
    Uint16 HCI_SvisionTimeout;
    Uint16 HCI_MinCELength;
    Uint16 HCI_MaxCELength;

};


struct HCLL_LE_Create_Conn_Cancel_Para                   
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Read_White_List_Size_Para                 
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Clear_White_List_Para                     
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Add_Device_To_White_List_Para             
{
    
    Uint8 HCI_Addr_Type;
    Uint8 HCI_Addr[6];
};


struct HCLL_LE_Rmv_Device_From_White_List_Para           
{
    
    Uint8 HCI_Addr_Type;
    Uint8 HCI_Addr[6];
};


struct HCLL_LE_Conn_Update_Para                          
{
    
    Uint8 HCI_Conn_Hdl_L;

    Uint8 padding0;

    Uint16 HCI_ConnIntervalMin;
    Uint16 HCI_ConnIntervalMax;
    Uint16 HCI_ConnLatency;
    Uint16 HCI_SvisionTimeout;
    Uint16 HCI_MinCELength;
    Uint16 HCI_MaxCELength;
};


struct HCLL_LE_Set_Host_Ch_Classification_Para           
{
    




    Uint8 padding0;
    Uint8 HCI_Channel_Map[5];
    Uint16 LL_EventCounterUpd;

};


struct HCLL_LE_Read_Ch_map_Para                          
{
    
    Uint8 HCI_Conn_Hdl_L;
};


struct HCLL_LE_Read_Remote_Used_Feat_Para                
{
    
    Uint8 HCI_Conn_Hdl_L;
};


struct HCLL_LE_Encrypt_Para                              
{
    
    Uint8 HCI_Key[16];
    Uint8 HCI_Plain_Data[16];
};


struct HCLL_LE_Rand_Para                                 
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Start_Encryption_Para                     
{
    
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Random_Num[8];
    Uint8 HCI_Encry_Div[2];
    Uint8 HCI_LongTermKey[16];
};


struct HCLL_LE_Long_Term_key_Req_Reply_Para              
{
    
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_LongTermKey[16];
};


struct HCLL_LE_Long_Term_key_Req_Neg_Reply_Para          
{
    
    Uint8 HCI_Conn_Hdl_L;
};


struct HCLL_LE_Read_Supported_States_Para                
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Receiver_Test_Para                        
{
    
    Uint8 HCI_RX_Channel;
};


struct HCLL_LE_Transmitter_Test_Para                     
{
    
    Uint8 HCI_TX_Channel;
    Uint8 HCI_LengthOfTestData;
    Uint8 HCI_Packet_Payload;
};


struct HCLL_LE_Test_End_Para                             
{
    
    Uint8 HCI_ResvMem[((6+31+7)-0)];
};


struct HCLL_LE_Remote_Conn_Param_Req_Reply_Para          
{
    
    Uint8 HCI_Conn_Hdl_L;
    Uint16 HCI_ConnIntervalMin;
    Uint16 HCI_ConnIntervalMax;
    Uint16 HCI_ConnLatency;
    Uint16 HCI_SvisionTimeout;
    Uint16 HCI_MinCELength;
    Uint16 HCI_MaxCELength;
};




struct HCLL_LE_Remote_Conn_Param_Req_Neg_Reply_Para      
{
    
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Reason;
};



struct HCLL_LE_ACL_Data_Pkt_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[((6+31+7)-2-1)];
};


struct HCLL_LE_ACL_Data_Pkt_Extend_Para             
{
    Uint8 HCI_Data_Idx1; Uint8 HCI_Data_Idx0;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[((6+31+7)-2-1)];
};


struct LLHC_LE_Ctrl_Pkt_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_CtrlPkt_Opcode;
    Uint8 HCI_CtrlPkt_Data[(6+31+7)-2-1-1];
};


struct HCLL_LE_Ctrl_Pkt_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_CtrlPkt_Opcode;
    Uint8 HCI_CtrlPkt_Data[(6+31+7)-2-1-1];
};


struct HCLL_LE_Ctrl_Pkt_Conn_Update_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 Padding0;
    Uint8 HCI_CtrlPkt_Opcode;
    Uint8 HCI_CtrlPkt_WinSize;
    Uint16 HCI_CtrlPkt_WinOffset;
    Uint16 HCI_CtrlPkt_Interval;
    Uint16 HCI_CtrlPkt_Latency;
    Uint16 HCI_CtrlPkt_Timeout;
    Uint16 HCI_CtrlPkt_Instant;
};


struct HCLL_LE_Ctrl_Pkt_Set_Host_Ch_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_CtrlPkt_Opcode;
    Uint8 HCI_CtrlPkt_ChM[5];




    Uint8 Padding0;
    Uint16 HCI_CtrlPkt_Instant;

    Uint8 HCI_ResvMem[(6+31+7)-2-1-9];
};



struct MLL_LE_Conn_Para_SMP
{
    Uint8 LL_Random_Num[8];
    Uint8 LL_Encry_Div[2];
    Uint8 LL_SMP_SKD[8];                
    Uint8 LL_SMP_IV[(4+4)];             
    Uint8 LL_SMP_Key[16];               
};


struct MLL_LL_Enc_Req_Para
{
    Uint8 LL_Conn_No;
    Uint8 ResvMem[((6+31+7)-1)];
};


struct MLL_LE_CCM_Manual_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 *mblk_LL_ConnDataQ;                         
};


struct LLEC_LE_CCM_Manual_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[7];          
    Uint32 LL_SMP_packetCounterT;       
    Uint8  LL_SMP_packetCounterTd;
    Uint8 LL_SMP_IV[8];                 
    Uint8 HCI_DataPkt_Length_CCM;       
    Uint8 HCI_DataPkt_Data_CCM[16];     
};


struct MHC_Cmd_Complete_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Num_Cmd_Pckts;
    Uint8 By_Primitive;
    Uint8 HCI_EventPara[((6+31+7)-6)];
};

struct MHC_Cmd_Status_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    Uint8 HCI_Num_Cmd_Pckts;
    Uint8 By_Primitive;
    Uint8 HCI_ResvMem[((6+31+7)-7)];
};

struct MHC_Disconn_Complete_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Reason;
    Uint8 HCI_ResvMem[((6+31+7)-7)];
};

struct MHC_Num_Of_Completed_Pckts_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Num_of_Hdl;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_HC_Num_Cmplt_Pkt_L;
    Uint8 HCI_HC_Num_Cmplt_Pkt_H;
    Uint8 HCI_ResvMem[((6+31+7)-8)];
};


struct MHC_Read_Remote_Ver_Info_Complete_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Ver;
    Uint8 HCI_Manufct_Name_L;
    Uint8 HCI_Manufct_Name_H;
    Uint8 HCI_SubVer_L;
    Uint8 HCI_SubVer_H;
    Uint8 HCI_ResvMem[((6+31+7)-11)];
};

struct MHC_Encrypt_Change_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Encrpt_En;
    Uint8 HCI_ResvMem[((6+31+7)-7)];
};

struct MHC_Encrypt_Key_Refresh_Complete_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Status;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_ResvMem[((6+31+7)-6)];
};

struct MHC_Data_Buffer_Overflow_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Link_Type;
    Uint8 HCI_ResvMem[((6+31+7)-4)];
};

struct MHC_Hardware_Error_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_EventPara;
    Uint8 HCI_ResvMem[((6+31+7)-4)];
};

struct MHC_Le_Conn_Complete_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Status;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Role;
    Uint8 HCI_PeerAddrType;
    Uint8 HCI_PeerAddr[6];
    Uint8 HCI_Conn_IntervalL;
    Uint8 HCI_Conn_IntervalH;
    Uint8 HCI_ConnLatencyL;
    Uint8 HCI_ConnLatencyH;
    Uint8 HCI_SvisionTimeoutL;
    Uint8 HCI_SvisionTimeoutH;
    Uint8 HCI_Master_Clk_Accuracy;
    Uint8 HCI_ResvMem[((6+31+7)-22)];
};

struct MHC_Le_Adv_Report_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Num_Reports;
    Uint8 HCI_Event_Type0;
    Uint8 HCI_Addr_Type0;
    Uint8 HCI_Addr0[6];
    Uint8 HCI_Length0;
    Uint8 HCI_Data0[31+1];      
    
};

struct MHC_Le_Conn_Update_Complete_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Status;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_ConnIntervalL;
    Uint8 HCI_ConnIntervalH;
    Uint8 HCI_ConnLatencyL;
    Uint8 HCI_ConnLatencyH;
    Uint8 HCI_SvisionTimeoutL;
    Uint8 HCI_SvisionTimeoutH;
    Uint8 HCI_ResvMem[((6+31+7)-13)];
};

struct MHC_Le_Read_Remote_Used_Feat_Complete_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Status;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_LE_Feat[8];
    Uint8 HCI_ResvMem[((6+31+7)-15)];
};

struct MHC_Le_Long_Term_Key_Req_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_Random_Num[8];
    Uint8 HCI_Encry_Div[2];
    Uint8 HCI_ResvMem[((6+31+7)-16)];
};


struct MHC_Le_Remote_Conn_param_Req_Event_Para     
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_ConnIntervalMinL;
    Uint8 HCI_ConnIntervalMinH;
    Uint8 HCI_ConnIntervalMaxL;
    Uint8 HCI_ConnIntervalMaxH;
    Uint8 HCI_ConnLatencyL;
    Uint8 HCI_ConnLatencyH;
    Uint8 HCI_SvisionTimeoutL;
    Uint8 HCI_SvisionTimeoutH;
    Uint8 HCI_ResvMem[((6+31+7)-14)];
};


struct HCLL_Le_Read_Phy_Para
{
    
    Uint8 HCI_Conn_Hdl_L;
};

struct HCLL_Le_Set_Default_Phy_Para
{
    
    Uint8 HCI_AllPhys;
    Uint8 HCI_TxPhys;
    Uint8 HCI_RxPhys;
};

struct HCLL_Le_Set_Phy_Para
{
    
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_AllPhys;
    Uint8 HCI_TxPhys;
    Uint8 HCI_RxPhys;
    Uint8 HCI_PhyOptions;
};


struct HCLL_LE_Set_Data_Length_Para
{
    
    Uint8 HCI_Conn_Hdl_L;

    Uint16 Padding;

    Uint16 HCI_RxOctets0;
    Uint16 HCI_RxTime;
    Uint16 HCI_TxOctets0;
    Uint16 HCI_TxTime;
};


struct MHC_Authen_Payload_Timeout_Expired_Para     
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_ResvMem[((6+31+7)-5)];
};


struct MHC_Event_HCI_Raw_Para
{
    Uint8 HCI_Para_Length;
    Uint8 Data[((6+31+7)-1)];
};

#line 947 "..\\..\\..\\include\\rf_include\\knl_pblc.h"
struct MHC_Le_Data_Length_Change_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 Padding;
    Uint16 HCI_MaxTxOctets;
    Uint16 HCI_MaxRxOctets;
    Uint8 HCI_ResvMem[((6+31+7)-7-1-0)];
};



struct MHC_Le_PHY_Update_Complete_Para
{
    
    Uint8 HCI_Para_Length;
    Uint8 HCI_Subevent_Code;
    Uint8 HCI_Status;
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_TX_PHY;
    Uint8 HCI_RX_PHY;
    Uint8 HCI_ResvMem[((6+31+7)-5-1-0)];
};


struct MLL_LL_CHN_Map_Upd_Para
{
    Uint8 LL_Conn_No;
    Uint8 ResvMem[((6+31+7)-1)];
};


struct HSPF_Att_Write_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[(6+31+7)-2-1];
};


struct HSPF_Att_Read_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[(6+31+7)-2-1];
};


struct HSPF_Att_Notify_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[(6+31+7)-2-1];
};


struct HSPF_Att_Indicate_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[(6+31+7)-2-1];
};


struct PFHS_Att_Chk_Client_Tab_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[(6+31+7)-2-1];
};


struct HSPF_Att_Cfm_Client_Tab_Para
{
    Uint8 HCI_Conn_Hdl_L; Uint8 HCI_Conn_Hdl_H;
    Uint8 HCI_DataPkt_LthL;
    Uint8 HCI_DataPkt_Data[(6+31+7)-2-1];
};

struct MHC_Le_Stk_Gen_Para
{
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_STK_GEN_MTHD;
};
struct MHC_Le_Auth_status_Para
{
    Uint8 HCI_Conn_Hdl_L;
    Uint8 HCI_AUTH_STATUS;
};

 
typedef struct MBLK
{
    void  *Next;        
    Uint8 Primitive;   
    union
    {
        Uint8 Data[(6+31+7)];
        struct HCLL_Disconnect_Para                         HCLL_Disconnect_Para;
        struct HCLL_Read_Remote_Ver_Info_Para               HCLL_Read_Remote_Ver_Info_Para;
        struct HCLL_Set_Event_Mask_Para                     HCLL_Set_Event_Mask_Para;
        struct HCLL_Reset_Para                              HCLL_Reset_Para;
        struct HCLL_Read_Transmit_Pwr_Level_Para            HCLL_Read_Transmit_Pwr_Level_Para;
        struct HCLL_Set_Ctrler_To_Host_Flow_Ctrl_Para       HCLL_Set_Ctrler_To_Host_Flow_Ctrl_Para;
        struct HCLL_Host_Buffer_Size_Para                   HCLL_Host_Buffer_Size_Para;
        struct HCLL_Host_Num_Of_Completed_Packets_Para      HCLL_Host_Num_Of_Completed_Packets_Para;
        struct HCLL_Set_Event_Mask_Page_2_Para              HCLL_Set_Event_Mask_Page_2_Para;
        struct HCLL_Read_Le_Host_Support_Para               HCLL_Read_Le_Host_Support_Para;
        struct HCLL_Write_Le_Host_Support_Para              HCLL_Write_Le_Host_Support_Para;
        struct HCLL_Read_Authen_Payload_Timeout_Para        HCLL_Read_Authen_Payload_Timeout_Para;
        struct HCLL_Write_Authen_Payload_Timeout_Para       HCLL_Write_Authen_Payload_Timeout_Para;
        struct HCLL_Read_Local_Ver_Info_Para                HCLL_Read_Local_Ver_Info_Para;
        struct HCLL_Read_Local_Supported_Cmd_Para           HCLL_Read_Local_Supported_Cmd_Para;
        struct HCLL_Read_Local_Supported_Feat_Para          HCLL_Read_Local_Supported_Feat_Para;
        struct HCLL_Read_Buffer_Size_Para                   HCLL_Read_Buffer_Size_Para;
        struct HCLL_Read_Bd_Addr_Para                       HCLL_Read_Bd_Addr_Para;
        struct HCLL_Read_Rssi_Para                          HCLL_Read_Rssi_Para;
        struct HCLL_LE_Set_Event_Mask_Para                  HCLL_LE_Set_Event_Mask_Para;
        struct HCLL_LE_Read_Buffer_Size_Para                HCLL_LE_Read_Buffer_Size_Para;
        struct HCLL_LE_Read_Local_Supported_Feat_Para       HCLL_LE_Read_Local_Supported_Feat_Para;
        struct HCLL_LE_Set_Random_Addr_Para                 HCLL_LE_Set_Random_Addr_Para;
        struct HCLL_LE_Set_Adv_Param_Para                   HCLL_LE_Set_Adv_Param_Para;
        struct HCLL_LE_Read_Adv_Ch_Tx_Pwr_Para              HCLL_LE_Read_Adv_Ch_Tx_Pwr_Para;
        struct HCLL_LE_Set_Adv_Data_Para                    HCLL_LE_Set_Adv_Data_Para;
        struct HCLL_LE_Set_Scan_Response_Data_Para          HCLL_LE_Set_Scan_Response_Data_Para;
        struct HCLL_LE_Set_Advertise_Enable_Para            HCLL_LE_Set_Advertise_Enable_Para;
        struct HCLL_LE_Set_Scan_Param_Para                  HCLL_LE_Set_Scan_Param_Para;
        struct HCLL_LE_Set_Scan_Enable_Para                 HCLL_LE_Set_Scan_Enable_Para;
        struct HCLL_LE_Create_Conn_Para                     HCLL_LE_Create_Conn_Para;
        struct HCLL_LE_Create_Conn_Cancel_Para              HCLL_LE_Create_Conn_Cancel_Para;
        struct HCLL_LE_Read_White_List_Size_Para            HCLL_LE_Read_White_List_Size_Para;
        struct HCLL_LE_Clear_White_List_Para                HCLL_LE_Clear_White_List_Para;
        struct HCLL_LE_Add_Device_To_White_List_Para        HCLL_LE_Add_Device_To_White_List_Para;
        struct HCLL_LE_Rmv_Device_From_White_List_Para      HCLL_LE_Rmv_Device_From_White_List_Para;
        struct HCLL_LE_Conn_Update_Para                     HCLL_LE_Conn_Update_Para;
        struct HCLL_LE_Set_Host_Ch_Classification_Para      HCLL_LE_Set_Host_Ch_Classification_Para;
        struct HCLL_LE_Read_Ch_map_Para                     HCLL_LE_Read_Ch_map_Para;
        struct HCLL_LE_Read_Remote_Used_Feat_Para           HCLL_LE_Read_Remote_Used_Feat_Para;
        struct HCLL_LE_Encrypt_Para                         HCLL_LE_Encrypt_Para;
        struct HCLL_LE_Rand_Para                            HCLL_LE_Rand_Para;
        struct HCLL_LE_Start_Encryption_Para                HCLL_LE_Start_Encryption_Para;
        struct HCLL_LE_Long_Term_key_Req_Reply_Para         HCLL_LE_Long_Term_key_Req_Reply_Para;
        struct HCLL_LE_Long_Term_key_Req_Neg_Reply_Para     HCLL_LE_Long_Term_key_Req_Neg_Reply_Para;
        struct HCLL_LE_Read_Supported_States_Para           HCLL_LE_Read_Supported_States_Para;
        struct HCLL_LE_Receiver_Test_Para                   HCLL_LE_Receiver_Test_Para;
        struct HCLL_LE_Transmitter_Test_Para                HCLL_LE_Transmitter_Test_Para;
        struct HCLL_LE_Test_End_Para                        HCLL_LE_Test_End_Para;
        struct HCLL_LE_Remote_Conn_Param_Req_Reply_Para     HCLL_LE_Remote_Conn_Param_Req_Reply_Para;
        struct HCLL_LE_Remote_Conn_Param_Req_Neg_Reply_Para HCLL_LE_Remote_Conn_Param_Req_Neg_Reply_Para;
        struct HCLL_LE_ACL_Data_Pkt_Para                    HCLL_LE_ACL_Data_Pkt_Para;
        struct HCLL_LE_ACL_Data_Pkt_Extend_Para             HCLL_LE_ACL_Data_Pkt_Extend_Para;
        struct LLHC_LE_Ctrl_Pkt_Para                        LLHC_LE_Ctrl_Pkt_Para;
        struct HCLL_LE_Ctrl_Pkt_Para                        HCLL_LE_Ctrl_Pkt_Para;
        struct HCLL_LE_Ctrl_Pkt_Conn_Update_Para            HCLL_LE_Ctrl_Pkt_Conn_Update_Para;
        struct HCLL_LE_Ctrl_Pkt_Set_Host_Ch_Para            HCLL_LE_Ctrl_Pkt_Set_Host_Ch_Para;
        struct MLL_LE_Conn_Para_SMP                         MLL_LE_Conn_Para_SMP;
        struct MLL_LL_Enc_Req_Para                          MLL_LL_Enc_Req_Para;
        struct MLL_LE_CCM_Manual_Para                       MLL_LE_CCM_Manual_Para;
        struct LLEC_LE_CCM_Manual_Para                      LLEC_LE_CCM_Manual_Para;
        struct MHC_Cmd_Complete_Para                        MHC_Cmd_Complete_Para;
        struct MHC_Cmd_Status_Para                          MHC_Cmd_Status_Para;
        struct MHC_Disconn_Complete_Para                    MHC_Disconn_Complete_Para;
        struct MHC_Num_Of_Completed_Pckts_Para              MHC_Num_Of_Completed_Pckts_Para;
        struct MHC_Read_Remote_Ver_Info_Complete_Para       MHC_Read_Remote_Ver_Info_Complete_Para;
        struct MHC_Encrypt_Change_Para                      MHC_Encrypt_Change_Para;
        struct MHC_Encrypt_Key_Refresh_Complete_Para        MHC_Encrypt_Key_Refresh_Complete_Para;
        struct MHC_Data_Buffer_Overflow_Para                MHC_Data_Buffer_Overflow_Para;
        struct MHC_Hardware_Error_Para                      MHC_Hardware_Error_Para;
        struct MHC_Le_Conn_Complete_Para                    MHC_Le_Conn_Complete_Para;
        struct MHC_Le_Adv_Report_Para                       MHC_Le_Adv_Report_Para;
        struct MHC_Le_Conn_Update_Complete_Para             MHC_Le_Conn_Update_Complete_Para;
        struct MHC_Le_Read_Remote_Used_Feat_Complete_Para   MHC_Le_Read_Remote_Used_Feat_Complete_Para;
        struct MHC_Le_Long_Term_Key_Req_Para                MHC_Le_Long_Term_Key_Req_Para;
        struct MHC_Le_Remote_Conn_param_Req_Event_Para      MHC_Le_Remote_Conn_param_Req_Event_Para;
        struct MHC_Authen_Payload_Timeout_Expired_Para      MHC_Authen_Payload_Timeout_Expired_Para;
        struct MHC_Event_HCI_Raw_Para                       MHC_Event_HCI_Raw_Para;
        struct MHC_Le_Data_Length_Change_Para               MHC_Le_Data_Length_Change_Para;
        struct MHC_Le_PHY_Update_Complete_Para              MHC_Le_PHY_Update_Complete_Para;    
        struct HCLL_Le_Read_Phy_Para                        HCLL_Le_Read_Phy_Para;              
        struct HCLL_Le_Set_Default_Phy_Para                 HCLL_Le_Set_Default_Phy_Para;       
        struct HCLL_Le_Set_Phy_Para                         HCLL_Le_Set_Phy_Para;               
        struct HCLL_LE_Set_Data_Length_Para                 HCLL_LE_Set_Data_Length_Para;       
        struct MLL_LL_CHN_Map_Upd_Para                      MLL_LL_CHN_Map_Upd_Para;
        struct HSPF_Att_Write_Para                          HSPF_Att_Write_Para;
        struct HSPF_Att_Read_Para                           HSPF_Att_Read_Para;
        struct HSPF_Att_Notify_Para                         HSPF_Att_Notify_Para;
        struct HSPF_Att_Indicate_Para                       HSPF_Att_Indicate_Para;
        struct PFHS_Att_Chk_Client_Tab_Para                 PFHS_Att_Chk_Client_Tab_Para;
        struct HSPF_Att_Cfm_Client_Tab_Para                 HSPF_Att_Cfm_Client_Tab_Para;
        struct MHC_Le_Stk_Gen_Para                          MHC_Le_Stk_Gen_Para;
        struct MHC_Le_Auth_status_Para                      MHC_Le_Auth_status_Para;
    } Para;              
} MBLK;



 
typedef struct MQUEUE
{
    MBLK *QIn;
    MBLK *QOut;
} MQUEUE;


extern MBLK *GetMsgBlk(void);
extern MBLK *GetMsgBlk_wSize(Uint8 len);
extern MBLK *GetMsgBlk_L2_wSize(Uint8 len);
extern MBLK *GetMsgBlk_Isr(void);
extern MBLK *GetMsgBlk_L2_wSize_Isr(Uint8 len);
extern MBLK *GetMsgBlk_L1_wSize_Isr(Uint8 len);
extern Uint8 CheckMsgBlk_L1_wSizeUsed_Isr(Uint8 len);
extern Uint8 CheckMsgBlk_L2_wSizeUsed_Isr(Uint8 len);
extern void FreeMsgBlk(MBLK *pMsgBlk);
extern void FreeMsgBlk_Isr(MBLK *pMsgBlk);
extern void SndMsgBlks_Isr(MBLK *pMsgBlk, Uint8 QueueId);
extern void FreeMsgBlks_Isr(MBLK *pMsgBlk);
extern void SndMsgBlk(MBLK *pMsgBlk, Uint8 QueueId);
extern void SndMsgBlk_Isr(MBLK *pMsgBlk, Uint8 QueueId);
extern void SndMsgBlks(MBLK *pMsgBlk, Uint8 QueueId);
extern void FreeMsgBlks(MBLK *pMsgBlk);
extern MBLK *RcvMsgBlk(Uint8 QueueId);
extern Uint8 CheckMsgQueue(void);


extern void Knl_MemCpy(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_MemCpy_Fwd(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_MemCpyInv(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_CodeCpy(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern void Knl_CodeCpyInv(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern Uint8 Knl_MemComp(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern Uint8 Knl_CodeComp(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern void Knl_MemCpy_Isr(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_MemCpyInv_Isr(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern void Knl_CodeCpy_Isr(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern void Knl_CodeCpyInv_Isr(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern Uint8 Knl_MemComp_Isr(Uint8 *pDst, Uint8 *pSrc, Uint8 len);
extern Uint8 Knl_CodeComp_Isr(Uint8 *pDst, Uint8 const *pSrc, Uint8 len);
extern Uint16 EndianSwap16_Isr(Uint16 value);
extern void EndianSwap16addr_Isr(Uint16 *value);
extern void EndianSwap16addrM8_Isr(Uint16 *value);

extern void Knl_Delay(Uint8 ticks);
extern Uint16 EndianSwap16(Uint16 value);
extern Uint32 EndianSwap32(Uint32 value);
extern void EndianSwap16addr(Uint16 *value);
extern void EndianSwap32addr(Uint32 *value);
extern void EndianSwap16addrM4(Uint16 *value);
extern void EndianSwap16addrM8(Uint16 *value);
extern void ErrorEntry(Uint32 errID);
extern void KernelCollect(void);
#line 24 "..\\..\\..\\source\\LL.c"
#line 1 "..\\..\\..\\include\\rf_include\\LL.h"













 




#line 20 "..\\..\\..\\include\\rf_include\\LL.h"













#line 39 "..\\..\\..\\include\\rf_include\\LL.h"

enum
{
    _TICK_BASE_125P00_ = 0U,
    _TICK_BASE_156P25_ = 1U
};














 
#line 120 "..\\..\\..\\include\\rf_include\\LL.h"
 
#line 128 "..\\..\\..\\include\\rf_include\\LL.h"







 
#line 150 "..\\..\\..\\include\\rf_include\\LL.h"







#line 167 "..\\..\\..\\include\\rf_include\\LL.h"








#line 187 "..\\..\\..\\include\\rf_include\\LL.h"

#line 194 "..\\..\\..\\include\\rf_include\\LL.h"




#line 247 "..\\..\\..\\include\\rf_include\\LL.h"











#line 265 "..\\..\\..\\include\\rf_include\\LL.h"












#line 283 "..\\..\\..\\include\\rf_include\\LL.h"

#line 290 "..\\..\\..\\include\\rf_include\\LL.h"


extern uint8_t LL_Tmr_Ticks_RSV_Wakeup;















#line 336 "..\\..\\..\\include\\rf_include\\LL.h"


#line 344 "..\\..\\..\\include\\rf_include\\LL.h"

#line 353 "..\\..\\..\\include\\rf_include\\LL.h"


#line 364 "..\\..\\..\\include\\rf_include\\LL.h"


#line 374 "..\\..\\..\\include\\rf_include\\LL.h"






#line 386 "..\\..\\..\\include\\rf_include\\LL.h"






#line 420 "..\\..\\..\\include\\rf_include\\LL.h"

#line 433 "..\\..\\..\\include\\rf_include\\LL.h"












#line 456 "..\\..\\..\\include\\rf_include\\LL.h"











#line 473 "..\\..\\..\\include\\rf_include\\LL.h"







#line 502 "..\\..\\..\\include\\rf_include\\LL.h"














#line 522 "..\\..\\..\\include\\rf_include\\LL.h"















































#line 577 "..\\..\\..\\include\\rf_include\\LL.h"

#line 587 "..\\..\\..\\include\\rf_include\\LL.h"

#line 611 "..\\..\\..\\include\\rf_include\\LL.h"






#line 624 "..\\..\\..\\include\\rf_include\\LL.h"








typedef struct Adv_Param
{
    Uint8 LL_AdvMap_ID;             
    Uint8 LL_AdvConn_ID;            
    Uint8 LL_Tx_PowerLevel;         
    Uint16 LL_Adv_Interval_MinI;    
    Uint16 LL_Adv_Interval_MaxI;    
    Uint8 LL_Adv_Type;              
    Uint8 LL_Own_Addr_Type;         
    Uint8 LL_DirectAddr_Type;       
    Uint8 LL_DirectAddr[6];   
    Uint8 LL_Adv_Channel_Map;           
    Uint8 LL_Adv_Filter_Policy;         
    Uint32 LL_Adv_Interval_Min;     
    Uint32 LL_Adv_Interval_Max;     
} Adv_Param;



typedef struct Adv_Para
{
    Uint8 LL_AdvMap_ID;             
    Uint8 LL_AdvConn_ID;            
    Uint8 LL_Tx_PowerLevel;         
    Uint16 LL_Adv_Interval_MinI;    
    Uint16 LL_Adv_Interval_MaxI;    
    Uint8 LL_Adv_Type;              
    Uint8 LL_Own_Addr_Type;         
    Uint8 LL_DirectAddr_Type;       
    Uint8 LL_DirectAddr[6];   
    Uint8 LL_Adv_Channel_Map;           
    Uint8 LL_Adv_Filter_Policy;         
    Uint32 LL_Adv_Interval_Min;     
    Uint32 LL_Adv_Interval_Max;     
    Uint8 LL_Adv_Data[31];   
    Uint8 LL_Adv_Data_Length;           
    Uint8 LL_ScanRsp_Data[31];   
    Uint8 LL_ScanRsp_Data_Length;       
} Adv_Para;


typedef struct Adv_Data_Buf
{
    Uint8 LL_Adv_Data[31];   
    Uint8 LL_Adv_Data_Length;           
    Uint8 LL_ScanRsp_Data[31];   
    Uint8 LL_ScanRsp_Data_Length;       
} Adv_Data_Buf;

typedef struct LE_Set_Scan_Para
{
    Uint8 LL_AdvMap_ID;         
    Uint8 LL_LE_ScanType;       
    Uint16 LL_LE_Scan_Interval; 
    Uint16 LL_LE_Scan_Window;   
    Uint8 LL_Own_Addr_Type;     
    Uint8 LL_ScanFilterPolicy;  
    Uint16 LL_LE_ScanAccl;      
    Uint8 LL_Tx_PowerLevel;     
} LE_Set_Scan_Para;




typedef struct LE_Conn_Para
{
    Uint8 LL_Conn_ID;                   
    Uint8 LL_Feature;                   
    Uint8 LL_Feature1;                  
    Uint8 LL_Tx_PowerLevel;             
    Uint32 LL_AccessAddr;               
    Uint8 LL_CRC_Init[3];    
    Uint8 WinSize_DataHdr;              
    Uint16 WinOffset_LtcyAccu;          
    Uint16 LL_ConnInterval;             
    Uint16 LL_ConnLatency;              
    Uint16 LL_SvisionTimeout;           
    Uint8 LL_ChMapReM[5];   
    Uint8 LL_HopIncrement;              
    Uint16 LL_EventCounter;             
    Uint8 LL_CurrentCH;                 
    Sint8 RF_Rssi;
    Uint16 LL_ConnIntervalOrg;          
    Uint16 LL_ConnIntervalOrgUpd;       
    Uint8 ErrCode_DisConn;              
    Uint8 WinSizeUpd;                   
    Uint16 WinOffsetUpd;                
    Uint16 LL_ConnIntervalUpd;          
    Uint16 LL_ConnLatencyUpd;           
    Uint16 LL_SvisionTimeoutUpd;        
    Uint16 LL_EventCounterUpd;          
    Uint8 LL_RF_Data_Ch_ReM[37];    
    Uint8 LL_SMP_Gate;                  
    Uint8 LL_SMP_IV_DUMP[(8-4)];   
    Uint8 LL_SMP_IV[(4+4)];         
    Uint8 LL_SMP_Key[16];                      
    Uint32 LL_SMP_packetCounterT;                       
    Uint8 LL_SMP_packetCounterTd;                       
    Uint8 LL_SMP_DataCh;                                
    Uint8 LL_Tx_PHYsUpd;                                
    Uint8 LL_Rx_PHYsUpd;                                
    Uint32 LL_SMP_packetCounterR;                       
    Uint8 LL_SMP_packetCounterRd;                       
    Uint8 LL_Tx_PHYS;                                   
    Uint8 LL_Rx_PHYS;                                   
    Uint8 LL_Rx_length_1M;                              
    Uint8 LL_Rx_length_2M;                              
    Uint8 LL_Tx_length_1M;                              
    Uint8 LL_Tx_length_2M;                              
    Uint8 LL_SCA;                                       
    Uint16 LL_SvToutAccu;                               
    Uint16 LL_PrToutAccu;                               
} LE_Conn_Para;         



struct Conn_Req_Para
{
    Uint8 IniAddr[6];     
    Uint8 AdvAddr[6];     
    Uint32 AccessAddr;              
    Uint8 CRC_Init[3];   
    Uint8 trWinSize;                
    Uint16 trWinOffset;             
    Uint16 ConnInterval;            
    Uint16 ConnLatency;             
    Uint16 SvisionTimeout;          
    Uint8 ChMap[5];     
    Uint8 Hop:5;                    
    Uint8 mSCA:3;                   
};



struct Conn_Req_Para_Alt
{
    Uint8 IniAddr[6];     
    Uint8 AdvAddr[6];     
    Uint32 AccessAddr;              
    Uint8 CRC_Init[3];   
    Uint8 trWinSize;                
    Uint16 trWinOffset;             
    Uint16 ConnInterval;            
    Uint16 ConnLatency;             
    Uint16 SvisionTimeout;          
    Uint8 ChMap[5];     
    Uint8 Hop_mSCA;                 
};


struct LE_Init_Para
{
    Uint8 LL_Conn_ID;           
    Uint8 LL_AdvMap_ID;         
    Uint16 LL_LE_ScanAccl;      
    Uint16 LL_LE_Scan_Interval; 
    Uint16 LL_LE_Scan_Window;   
    Uint8 LL_InitFilterPolicy;  
    Uint8 LL_PeerAddrType;      
    Uint8 LL_PeerAddr[6];
    Uint8 LL_Own_Addr_Type;     
    Uint8 LL_Tx_PowerLevel;     
    Uint16 LL_CE_LenMin;        
    Uint16 LL_CE_LenMax;
    Uint16 LL_ConnInterval;
    Uint16 LL_ConnLatency;
    Uint16 LL_SvisionTimeout;
};                              



typedef struct LE_WhiteList_Para
{
    Uint8 AddrType;             
    Uint8 Addr[6];    
} LE_WhiteList_Para;



typedef union LL_WhiteList
{
    Uint8 Data[sizeof(LE_WhiteList_Para)];
    LE_WhiteList_Para LE_WhiteList_Para;
} LL_WhiteList;


typedef union LL_Adv
{
    Uint8 Data[sizeof(Adv_Para)];
    Adv_Para Adv_Para;
} LL_Adv;


typedef union LL_Scan
{
    Uint8 Data[(sizeof(LE_Set_Scan_Para))];
    LE_Set_Scan_Para LE_Set_Scan_Para;
} LL_Scan;


typedef union LL_Conn
{
    Uint8 Data[(sizeof(LE_Conn_Para))];
    LE_Conn_Para LE_Conn_Para;
} LL_Conn;


typedef union LL_Init
{
    Uint8 Data[(1*6+2*8+6)];
    struct LE_Init_Para LE_Init_Para;
} LL_Init;


typedef struct LL_Para_Itrvl
{
    Uint8 DataIdxF;
    Uint8 DataIdxN;
    Uint8 HeaderSts;
    Uint8 HeaderLen;
    Uint8 Data[32];
} LL_Para_Itrvl;


typedef struct LL_Para_ItrvlR
{
    Uint8 DataIdxF;
    Uint8 DataIdxN;
    Uint8 HeaderSts;
    Uint8 HeaderLen;
    Uint8 Data[255];
} LL_Para_ItrvlR;

typedef struct LL_Para_Header
{
    Uint8 HeaderSts;
    Uint8 HeaderLen;
    Uint8 Padding0;
    Uint8 Padding1;
} LL_Para_Header;


typedef struct LL_Para_Default_PHY
{
    Uint8 All_PHYs;
    Uint8 Tx_PHYs;
    Uint8 Rx_PHYs;
    Uint8 PHY_options;
} LL_Para_Default_PHY;


typedef struct BufProcess
{
    MBLK *BufPrcsF;
    MBLK *BufPrcsN;
} BufProcess;




#line 25 "..\\..\\..\\source\\LL.c"
#line 1 "..\\..\\..\\include\\rf_include\\hci.h"



#line 5 "..\\..\\..\\include\\rf_include\\hci.h"
#line 6 "..\\..\\..\\include\\rf_include\\hci.h"
#line 7 "..\\..\\..\\include\\rf_include\\hci.h"
#line 8 "..\\..\\..\\include\\rf_include\\hci.h"

#line 36 "..\\..\\..\\include\\rf_include\\hci.h"









#line 51 "..\\..\\..\\include\\rf_include\\hci.h"


#line 60 "..\\..\\..\\include\\rf_include\\hci.h"



#line 87 "..\\..\\..\\include\\rf_include\\hci.h"














#line 109 "..\\..\\..\\include\\rf_include\\hci.h"


#line 118 "..\\..\\..\\include\\rf_include\\hci.h"


#line 126 "..\\..\\..\\include\\rf_include\\hci.h"


#line 179 "..\\..\\..\\include\\rf_include\\hci.h"






typedef uint8_t HCIStatus;

#line 249 "..\\..\\..\\include\\rf_include\\hci.h"


#line 484 "..\\..\\..\\include\\rf_include\\hci.h"



#line 543 "..\\..\\..\\include\\rf_include\\hci.h"






#line 559 "..\\..\\..\\include\\rf_include\\hci.h"



#line 573 "..\\..\\..\\include\\rf_include\\hci.h"

#line 586 "..\\..\\..\\include\\rf_include\\hci.h"


#line 595 "..\\..\\..\\include\\rf_include\\hci.h"













#line 623 "..\\..\\..\\include\\rf_include\\hci.h"

extern Uint8 Event_Mask[5];
extern Uint8 Event_Mask2[1];
extern Uint8 LE_Event_Mask[3];

extern const Uint8 SIZE_HCI_COMMAND_LE_CONTROLLER_PARAMETERS[(30+2)];

extern const Uint8 TAB_HCI_PKT_LTH_TABLE[5];

extern const Uint8 TAB_SUPPORTED_COMMANDS_MASK[64];
extern const Uint8 TAB_LMP_FEATURES_MASK[8];

extern void initUart(void);
#line 653 "..\\..\\..\\include\\rf_include\\hci.h"

typedef void (*MHCCallBack)(BleCmdEvent event,
                            void *param);





#line 26 "..\\..\\..\\source\\LL.c"
#line 1 "..\\..\\..\\porting\\rf_porting\\porting_LLtimer.h"



#line 5 "..\\..\\..\\porting\\rf_porting\\porting_LLtimer.h"
#line 6 "..\\..\\..\\porting\\rf_porting\\porting_LLtimer.h"


extern void Tiny_Delay(uint32_t u32Usec);





#line 27 "..\\..\\..\\source\\LL.c"
#line 1 "..\\..\\..\\porting\\porting_spi.h"



#line 5 "..\\..\\..\\porting\\porting_spi.h"



 










 
void MCU_SpiInit(void);

extern uint8_t SPI_1BYT_SetRx(uint8_t regAddr);
extern uint8_t SPI_1BYT_SetRx_Isr(uint8_t regAddr);
extern void SPI_1BYT_SetTx(uint8_t regAddr, uint8_t u8SrcData);
extern void SPI_1BYT_SetTx_Isr(uint8_t regAddr, uint8_t u8SrcData);
extern void SPI_2BYT_SetTx_Isr(uint8_t regAddr, uint8_t *u8SrcAddr);

void SPI_PDMA_Init(void);
extern uint32_t SPI_PDMA_waitFinish(void);
extern void SPI_PDMA_SetRx_Isr(uint8_t regAddr, uint32_t u32DstAddr, uint32_t u32TransCount);
extern void SPI_PDMA_SetTx(uint8_t regAddr, uint32_t u32SrcAddr, uint32_t u32TransCount);




#line 28 "..\\..\\..\\source\\LL.c"




#line 1 "..\\..\\..\\include\\rf_include\\_rafael_phy.h"
 







 














#line 30 "..\\..\\..\\include\\rf_include\\_rafael_phy.h"








 
#line 47 "..\\..\\..\\include\\rf_include\\_rafael_phy.h"





 




#line 103 "..\\..\\..\\include\\rf_include\\_rafael_phy.h"




enum
{
    RFIP_REG_MEM_23 = 0U,
    RFIP_REG_MEM_36,
    RFIP_REG_MEM_40,
    RFIP_REG_MEM_41,
    RFIP_REG_MEM_42,
    RFIP_REG_MEM_43,
    RFIP_REG_MEM_57,
    RFIP_REG_MEM_61,
    RFIP_REG_MEM_107,
    RFIP_REG_MEM_119,
    RFIP_REG_MEM_120,
    RFIP_REG_MEM_121,
    RFIP_REG_MEM_166,
    SIZE_RFIP_REG
};




#line 144 "..\\..\\..\\include\\rf_include\\_rafael_phy.h"














#line 165 "..\\..\\..\\include\\rf_include\\_rafael_phy.h"











#line 183 "..\\..\\..\\include\\rf_include\\_rafael_phy.h"







 












 
enum
{
    RF_SLEEP_DISABLE = 0U,
    RF_SLEEP_ENABLE = 1U
};





 
extern volatile uint8_t RFIP_reg_MEM[SIZE_RFIP_REG];



 
extern void smpAES_in_key(uint8_t * key);
extern void smpAES_in_PlainText(uint8_t * plaintextData);
extern void smpAES_En(void);
extern void smpAES_inInv(uint8_t * key, uint8_t * plaintextData);
extern void smpAES_out(uint8_t * encDataOut);
extern void smpAES_outInv(uint8_t * encDataOut);



extern uint8_t CH_PLL_bank_Table[40];
extern void RF_PLL_VCO_Bank_set(uint8_t valueVCO);
extern uint8_t RF_CRCchk(void);
extern uint8_t RF_WTR_EnChk(void);

extern void rafael_reset_phy_fsm_Isr(void);
extern void setChannel_BLE(uint8_t ch);
extern void RF_IntReset(void);
extern void RF_WTR_intOff(void);
extern void RF_TxAutoAckOn(void);
extern void RF_TxAutoAckOff(void);
extern void RF_SymbolRate_set(uint8_t zSymbol_1M);
extern void RF_CRCInit(uint8_t *initParaAddr);
extern void RF_TxFIFO_ADVaddr_set(uint8_t *AdvAddr);
extern void RF_TxFIFO_ADVData_set(uint8_t *SrcAddr);
extern void RF_TxFIFO_LeData_set(uint8_t *SrcAddr, uint8_t length);
extern void RF_LE_HeaderStsLen_Tx(uint8_t *SrcAddr);
extern void RF_TxFIFO_LeData_wIdx_set(uint8_t *SrcAddr, uint8_t length);
extern void RF_RxLengthLimit(uint8_t maxPDU);

extern void RF_CCM_AES_Mode_set(uint8_t setting);
extern void RF_Tmr_Periodic_set_ISR(uint32_t period_tick);

extern void RF_PowerSaving_En_Isr(void);
extern uint8_t rssi_read_data[3];
extern int8_t RF_Get_LastRssi(void);
extern void RF_SymbolRate_Patch_1M_2M(uint8_t zSymbol_1M);
extern uint8_t RF_Set_TxPowerLevel_Isr(int8_t power);

extern void RF_WTR_intOn(void);
extern void RF_Header_Rdy_intOn(void);







#line 33 "..\\..\\..\\source\\LL.c"


LL_Adv          LL_Adv_Para;
LL_Adv          LL_Adv_Para_UpdBuf;
LL_Scan         LL_Scan_Para;
LL_Scan         LL_Scan_Para_UpdBuf;
LL_Conn         LL_Conn_Para[4+1];
MQUEUE          LL_ConnDataQ[4+1];
MQUEUE          LL_ConnCtrlQ[4+1];
MQUEUE          LL_ConnDataInQ[4+1];
MQUEUE          LL_ConnDataInTmp;
Uint8           LL_ConnDataTmpGate;
MBLK*           LL_ConnBuffPt[4+1];

LL_Init         LL_Init_Para;
LL_WhiteList    LL_WhiteList_Para[4];
LL_Para_Itrvl   LL_Para_Interval;
LL_Para_ItrvlR  LL_Para_IntervalR;
Uint8           LL_ConnID_Remaining;
BufProcess      LL_ConnBuffPrcs;
BufProcess      LL_ConnBuffPrcsR;
Adv_Data_Buf    *LL_Adv_Data_Buf;

Uint8 LL_Ref_ChMap[5];
LL_Para_Default_PHY LL_Ref_Default_PHY;


TBLK_LLx TmrBlk_LL[(2*(4+1+1))];
Uint8 TBlk_Free_LL;
Uint8 TBlk_InUse_LL;
TBLK_LLx *tblk_LL_pi;
TBLK_LLx *tblk_LL_pi2;

Uint8 LL_SMP_DataCh;
Uint8 status_LL_Tmr;
Uint8 anchor_LL_Tmr;
LL_Conn *LL_conn_pi;
Uint16 LL_DurRxPktAccu;
MBLK *mblk_LL_conn_Para[4+1];

Uint8 BD_Rand_AddrBuf[6];
Uint8 BD_Rand_Addr[6] = {0};
Uint32 LL_Slv_Win_Width;
Uint32 LL_Slv_Win_Width_Base;
uint32_t seedR16;
Uint8 RX_CRC_valid_flag = (1UL);


uint64_t Tmr37;
uint8_t ram_Tab[48];
Uint32 LL_Ref_Acs_Addr_ADVSCN;
Uint32 LL_Ref_crc_ini_ADVSCN;

uint8_t FT_WakeupFlag = 0;








const Uint8 LL_CHMAP_DEFAUT[5]= {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
const Uint8 LL_DEFAULT_PHY[]=
{
    (
        0x00
        
        
        
    ),  
    (
        0x01 |
        0x02 |
        
        0x00
    ),
    (
        0x01 |
        0x02 |
        
        0x00
    ),
    (
        0x00
        
        
    ),  
};

const Uint8 LE_LOCAL_VER_INFO[]=    
{
    0x09,
    0x01,
    0x00,
    0x09,
    0x64,
    0x08,
    0x00,
    0x00,
};

const Uint8 LL_VERSION[5]=
{
    0x09,
    0x64,
    0x08,
    0x00,
    0x00,
};

const Uint8 LL_FEATURE[] =                   
{
    (
        
        
        0x20 |
        
        0x08 |
        0x04 |
        
        0x01 |
        0x00
    ),
    (
        0x01 |
        
        
        
        
        
        
        
        0x00
    ),
    (
        
        
        
        
        
        
        
        
        0x00
    ),
    (
        
        
        
        
        0x00
    ),
    0x00, 0x00, 0x00, 0x00,
};


const Uint16 LL_LENGTH_DEFAULT[] =
{
    251,
    ((251+(10+4))*8),  
    251,
    ((251+(10+4))*8),  
};


const Uint16 LL_LENGTH[] =
{
    32,
    ((32+(10+4))*8),  
    32,
    ((32+(10+4))*8),  
};


Uint16 LL_Length_Prefer[4+1][(sizeof(LL_LENGTH)/sizeof(LL_LENGTH[0]))];   


const Uint32 LL_REF_ACS_ADDR[] =
{
    0x50829AAF,
    0x60859AAE,
    0x90A3655C,
    0xA093655A,
};








const Uint8 LL_REF_CRC_INI[][3] =
{
    {0x1A, 0xCB, 0xBF},
    {0xC4, 0x1A, 0x43},
    {0xE0, 0xC0, 0x69},
    {0xCB, 0xC3, 0x62},
};


const Uint8 LL_DUR_RSV[] =
{
    11+1+2,    
    8+1+2,     
    4+1+2,
    (11+1)+2,
    (11+1)+2,
    (8+1)+2,
    11+1+2,
};

const Uint16 LL_RF_PARA_TX_WIN_SIZE_MAP[]=
{
#line 263 "..\\..\\..\\source\\LL.c"
    0x003E, 0x0091, 0x00E5, 0x0138, 0x018B, 0x01DF, 0x0232,

};

const Uint32 LL_RF_PARA_WIN_WIDTH_CNVT[]=
{
#line 284 "..\\..\\..\\source\\LL.c"
    0x00020C49, 0x00018937, 0x000154C9, 0x00013A92, 0x00012D77, 0x0001205B, 0x000115DF, 0x000110A1,

};


#line 343 "..\\..\\..\\source\\LL.c"

const uint8_t LL_RF_DATA_CH[]=                                                       
{
    0,  1,  2,  3,  4,  5,  6,  7,
    8,  9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31,
    32, 33, 34, 35, 36,  0,  1,  2,
    3,  4,  5,  6,  7,  8,  9, 10,
    11, 12, 13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31, 32, 33, 34,
    35, 36,
};

const uint8_t CH_ADV_SEL_TABLE[] =
{
    0,  37,  38, 39
};

const Uint8 CH_ADV_CH_HOP_BY_MAP_TABLE[][4] =     
{
    {0, 0, 0, 0,},      
    {1, 0, 0, 0,},      
    {2, 0, 0, 0,},      
    {1, 2, 0, 0,},      
    {3, 0, 0, 0,},      
    {1, 3, 0, 0,},      
    {2, 0, 3, 0,},      
    {1, 2, 3, 0,},      
};
Uint8 Ch_ADV_Ch_Hop_Table[4];
Uint8 Ch_ADV_Ch_Hop_TableBuf[4];

const Uint8 PRIME_NUM[] =
{
    5,  7,  8,  9,
    11, 13, 17, 19,
    
};


const Uint8 RAND_INI32[]=
{
    0x8B, 0x14, 0x91, 0x7A,
    0x9C, 0x20, 0x0B, 0x09,
    0x0A, 0x05, 0xF7, 0x8C,
    0xE8, 0x93, 0xA4, 0x31,
    0xA6, 0xA9, 0xF2, 0x56,
    0x45, 0x8E, 0x1E, 0x36,
    0x8D, 0x81, 0xE4, 0xBE,
    0x35, 0xB7, 0xA5, 0x66,
};


const Uint8 TABLE_CONV_ADV_TYPE_HCI_LL[] =
{
    0x00,                    
    0x01,             
    0x06,               
    0x02,            
    0x01,             
};

const Uint8 LL_ACK_FLOW[] =
{
    (
        0x04 |
        0x40
    ),
    (
        (0x08|0x04) |
        (0x80|0x40)
    ),
    (
        0 |
        0x00
    ),
    (
        0x08 |
        0x80
    ),
    (
        0x04 |
        0x00
    ),
    (
        (0x08|0x04) |
        0x80
    ),
    (
        0 |
        0x40
    ),
    (
        0x08 |
        (0x80|0x40)
    ),
    (
        0x04 |
        (0x80|0x40)
    ),
    (
        (0x08|0x04) |
        0x40
    ),
    (
        0 |
        0x80
    ),
    (
        0x08 |
        0x00
    ),
    (
        0x04 |
        0x80
    ),
    (
        (0x08|0x04) |
        0x00
    ),
    (
        0 |
        (0x80|0x40)
    ),
    (
        0x08 |
        0x40
    ),
};


const Uint8 LL_ACK_FLOW_RE_TO[] =
{
    0x02, 0x03, 0x00, 0x01,
};


const Uint8 LL_PR_CONN_ID_PAIR[] =
{
    0,                                  
    0,                                  
    0x02,                   
    0x04,                         
    0x05,                   
    0x06,                   
    0x06,                   
    0,                                  
    0x09,                     
    0,                                  
    0x0B,                   
    0x0B,                   
    0x0C,                     
    0,                                  
    0x09,                     
    (0x40|0x10), 
    0,                                  
    0,                                  
    0x13,                        
    0,                                  
    0x15,                      
    0,                                  
    0x18,                  
    0x18,                  
    0,                                  
    0,                                  
};

const Uint8 LL_RX_CONN_ID_INIT[] =
{
    (0x80|0x00),    
    (0x80|0x01),     
    (0x80|0x02),        
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    (0x80|0x0C),     
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    0x04,                    
    (0x80|0x18),  
    0x04,                    
};

const Uint8 LL_TX_CONN_ID_INIT[] =
{
    (0x40|0x00),         
    (0x40|0x01),          
    (0x40|0x02),             
    (0x40|0x03),              
    0x04,                    
    (0x40|0x05),        
    (0x40|0x06),        
    0x04,                    
    (0x40|0x08),             
    0x04,                    
    (0x40|0x0A),        
    (0x40|0x0B),        
    (0x40|0x0C),          
    0x04,                    
    (0x40|0x0E),    
    (0x40|0x0F), 
    0x04,                    
    0x04,                    
    (0x40|0x12),             
    0x04,                    
    (0x40|0x14),           
    0x04,                    
    (0x40|0x16),              
    (0x40|0x17),              
    (0x40|0x18),       
    0x04,                    
};


const Uint8 LL_RX_CONN_ID_MASTER_SIDE_ONLY[] =
{
    (0x19+1),                     
    (0x19+1),                     
    0x02,                   
    (0x19+1),                     
    0x04,                         
    0x05,                   
    0x06,                   
    0x07,                     
    (0x19+1),                     
    0x09,                     
    (0x19+1),                     
    0x0B,                   
    0x0C,                     
    0x0D,                      
    0x0E,               
    0x0F,            
    0x10,            
    0x11,                  
    0x12,                        
    0x13,                        
    0x14,                      
    0x15,                      
    0x16,                         
    0x17,                         
    (0x19+1),                     
    0x19,           
};


const Uint8 LL_RX_CONN_ID_SLAVE_SIDE_ONLY[] =
{
    0x00,                 
    0x01,                 
    0x02,                   
    0x03,                         
    (0x19+1),                     
    (0x19+1),                     
    0x06,                   
    0x07,                     
    0x08,                     
    0x09,                     
    0x0A,                   
    0x0B,                   
    0x0C,                     
    0x0D,                      
    (0x19+1),                     
    0x0F,            
    (0x19+1),                     
    0x11,                  
    0x12,                        
    0x13,                        
    0x14,                      
    0x15,                      
    0x16,                         
    (0x19+1),                     
    0x18,                  
    (0x19+1),                     
};


const Uint8 LL_RX_CONN_ID_OP_LENGTH[] =
{
    11,                                 
    7,                                  
    1,                                  
    22,                                 
    12,                                 
    0,                                  
    0,                                  
    1,                                  
    8,                                  
    8,                                  
    0,                                  
    0,                                  
    5,                                  
    1,                                  
    8,                                  
    23,                                 
    23,                                 
    2,                                  
    0,                                  
    0,                                  
    8,                                  
    8,                                  
    2,                                  
    2,                                  
    4,                                  
    2,                                  
};


const Uint8 TAB_ZERO_128[] =
{
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
};  



const Uint8 LL_LEN_OFFSET_1M[] =    
{
    15,
    31,
    46,
    62,
    78,
    93,
    109,
    125,
    140,
    156,
    171,
    187,
    203,
    218,
    234,
    250,
    255,
};  


const Uint8 LL_LEN_OFFSET_2M[] =
{
    31,
    62,
    93,
    125,
    156,
    187,
    218,
    250,
    255,
};  
#line 754 "..\\..\\..\\source\\LL.c"

Uint8 LL_Msg_AdvScnConn;
Uint8 LL_Msg_AdvScnConnUpd;
Uint8 LL_Msg_AdvScnConnUpdSts;
#line 769 "..\\..\\..\\source\\LL.c"

MQUEUE MsgQueueEC;       

#line 779 "..\\..\\..\\source\\LL.c"

void smpAES_inInv(Uint8 * key, Uint8 * plaintextData);
void smpAES_out(Uint8 * encDataOut);
void smpAES_outInv(Uint8 * encDataOut);
void LL_smp_SKD_IV_genIn(Uint8 *DataOut);

extern void RF_Tmr16_IntvlSet(uint16_t param16);
uint8_t RF_Msg_RF0INT;
extern void rafael_reset_phy_fsm_Isr(void);



 




 



 
void SndMsgBlkEC(MBLK *pMsgBlk)
{

    __disable_irq();
    if (MsgQueueEC.QIn == (MBLK *)0)
    {
        MsgQueueEC.QOut = pMsgBlk;
        MsgQueueEC.QIn = pMsgBlk;
    }
    else
    {
        (MsgQueueEC.QIn)->Next = (MBLK *)pMsgBlk;
        MsgQueueEC.QIn = (MBLK *)pMsgBlk;
    }
    __enable_irq();
}


Uint8 LL_GetMasterFreeConnID(void)
{
    Uint8 i;

    for(i=0; i<4; i++)
    {
        if(LL_Conn_Para[i].LE_Conn_Para.LL_Conn_ID == 0x01)
        {
            break;
        }
    }
    return i;
}




Uint8 LL_GetSlaveFreeConnID(void)
{
    Uint8 i;

    for(i=4; i<4+1; i++)
    {
        if(LL_Conn_Para[i].LE_Conn_Para.LL_Conn_ID == 0x01)
        {
            break;
        }
    }
    return i;
}



void LL_ReleaseConnID(Uint8 LL_Conn_ID)
{
    MQUEUE *pqueue;
    extern MBLK *MBlk_Free;
    LL_Conn *pLL_conn;
    extern Uint8 MBlk_depth_Remaining;
    extern Uint8 LL_ConnID_Remaining;
    MBLK *mblk;
    Uint8 i;

    pLL_conn = &LL_Conn_Para[LL_Conn_ID];

    Knl_CodeCpy((Uint8 *)LL_Length_Prefer[LL_Conn_ID], (Uint8 *)LL_LENGTH_DEFAULT, sizeof(LL_LENGTH));
    Knl_CodeCpy(&pLL_conn->LE_Conn_Para.LL_Conn_ID, TAB_ZERO_128, (sizeof(LE_Conn_Para)));     
    __disable_irq();
    pLL_conn->LE_Conn_Para.LL_Conn_ID = 0x01;                                 
    pLL_conn->LE_Conn_Para.LL_Tx_PHYS = 0x01;                               
    pLL_conn->LE_Conn_Para.LL_Rx_PHYS = 0x01;                               
    pLL_conn->LE_Conn_Para.LL_Rx_length_1M = 27;                      
    pLL_conn->LE_Conn_Para.LL_Rx_length_2M = 27;                      
    pLL_conn->LE_Conn_Para.LL_Tx_length_1M = 27;                      
    pLL_conn->LE_Conn_Para.LL_Tx_length_2M = 27;                      
    

    if(LL_ConnBuffPt[LL_Conn_ID]!=(MBLK *)0)
    {
        mblk = LL_ConnBuffPt[LL_Conn_ID];
        while(1)
        {
            if(mblk != (MBLK *)0)
            {
                MBlk_depth_Remaining++;
                if(mblk->Next != (MBLK *)0)
                    mblk = mblk->Next;
                else
                    break;
            }
            else
                break;
        }
        if(mblk != (MBLK *)0)
        {
            mblk->Next = MBlk_Free;
            MBlk_Free = LL_ConnBuffPt[LL_Conn_ID];
            LL_ConnBuffPt[LL_Conn_ID] = (MBLK *)0;
        }
    }
    __enable_irq();
    for(i=0; i<3; i++)
    {
        switch(i)
        {
        case 0:
            pqueue = &LL_ConnDataQ[LL_Conn_ID];
            break;

        case 1:
            pqueue = &LL_ConnCtrlQ[LL_Conn_ID];
            break;

        case 2:
            pqueue = &LL_ConnDataInQ[LL_Conn_ID];
            break;

        default:
            break;
        }
        __disable_irq();
        if(pqueue->QIn != (MBLK *)0)
        {
            mblk = pqueue->QOut;
            while(1)
            {
                MBlk_depth_Remaining++;
                if(mblk->Next != (MBLK *)0)
                {
                    mblk = mblk->Next;
                }
                else
                {
                    break;
                }
            }
            (pqueue->QIn)->Next = MBlk_Free;
            MBlk_Free = pqueue->QOut;
            pqueue->QOut = (MBLK *)0;
            pqueue->QIn = (MBLK *)0;
        }
        __enable_irq();
        
    }
}


Uint8 LL_CheckConnExist(Uint8 LL_Conn_ID)
{

    if(LL_Conn_Para[LL_Conn_ID].LE_Conn_Para.LL_Conn_ID < 0x02)



    {
        return 0;
    }
    else
    {
        return (!0);
    }
}


Uint8 LL_Data2ConnTxFIFO(Uint8 connID, Uint8 *Data, Uint8 Length)
{
    MBLK *mblk;
    MBLK *pmblk;
    MQUEUE *pqueue;
    Uint8 DataIdx, len;

    if(LL_CheckConnExist(connID) == 0)
    {
        return (!0);
    }
    mblk = GetMsgBlk_L2_wSize(Length);
    if(mblk == (MBLK *)0)
    {
        return (!0);
    }


    if(Length<=7)
    {
        mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Length_CCM = 0;
    }


    pmblk = mblk;

    DataIdx = 0;
    while(Length)
    {
        if(Length >= ((6+31+7)-2-1))
        {
            len = ((6+31+7)-2-1);
            Length -= ((6+31+7)-2-1);
        }
        else
        {
            len = Length;
            Length = 0;
        }
        pmblk->Primitive = (0xB0+0x40);
        pmblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = len;
        pmblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = len;
        pmblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
        Knl_MemCpy(pmblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, (Data+DataIdx), len);

        if(pmblk->Next != (MBLK *)0)
        {
            pmblk = pmblk->Next;
            DataIdx += ((6+31+7)-2-1);
        }
    }
    mblk->Primitive = (0xB0+0x36);

    __disable_irq();
    if(LL_CheckConnExist(connID) == 0)
    {
        FreeMsgBlks(mblk);
        __enable_irq();
        return (!0);
    }
    else
    {
        pqueue = &LL_ConnDataQ[connID];
        if (pqueue->QIn == (MBLK *)0)
        {
            pqueue->QOut = mblk;
        }
        else
        {
            (pqueue->QIn)->Next = (MBLK *)mblk;
        }
        pqueue->QIn = pmblk;
        __enable_irq();
        return 0;
    }
}


Uint8 LL_Data2ConnTxFIFO_smp(Uint8 connID, Uint8 *Data, Uint8 Length)
{
    MBLK *mblk;
    MQUEUE *pqueue;

    if(LL_CheckConnExist(connID) == 0)
    {
        return (!0);
    }
    mblk = GetMsgBlk();
    if(mblk == (MBLK *)0)
    {
        return (!0);
    }

    if(Length<=7)
    {
        mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Length_CCM = 0;
    }


    mblk->Primitive = (0xB0+0x36);
    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = Length;
    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = Length;
    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
    Knl_MemCpy(mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, Data, Length);

    __disable_irq();
    if(LL_CheckConnExist(connID) == 0)
    {
        SndMsgBlk(mblk, 0);
        __enable_irq();
        return (!0);
    }
    else
    {
        pqueue = &LL_ConnDataQ[connID];
        if (pqueue->QIn == (MBLK *)0)
        {
            pqueue->QOut = mblk;
        }
        else
        {
            (pqueue->QIn)->Next = (MBLK *)mblk;
        }
        pqueue->QIn = mblk;
        __enable_irq();
        return 0;
    }
}


Uint8 LL_Data2ConnTxFIFO_smp_Isr(Uint8 connID, Uint8 *Data, Uint8 Length)
{
    MBLK *mblk;
    MQUEUE *pqueue;

    mblk = GetMsgBlk_Isr();
    if(mblk == (MBLK *)0)
    {
        return (!0);
    }

    if(Length<=7)
    {
        mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Length_CCM = 0;
    }


    mblk->Primitive = (0xB0+0x36);
    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = Length;
    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = Length;
    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
    Knl_MemCpy_Isr(mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, Data, Length);

    pqueue = &LL_ConnDataQ[connID];
    if (pqueue->QIn == (MBLK *)0)
    {
        pqueue->QOut = mblk;
    }
    else
    {
        (pqueue->QIn)->Next = (MBLK *)mblk;
    }
    pqueue->QIn = mblk;
    return 0;
}


void LL_Msg_CTRL_PKT(Uint8 *pParam, Uint8 OpcodeCtrl, Uint8 Len_Param, Uint8 ConnID)
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0xB0+0x38);
    mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_Conn_Hdl_L = ConnID;
    mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode = OpcodeCtrl;
    mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_DataPkt_LthL = Len_Param+1;

    if(Len_Param)
    {
        Knl_MemCpy(mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data, pParam, Len_Param);
    }
    SndMsgBlk(mblk, 0);
}


#line 1163 "..\\..\\..\\source\\LL.c"
void LL_Msg_Event_Status(Uint8 HCI_Status, Uint8 By_Primitive)
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x02);
    mblk->Para.MHC_Cmd_Status_Para.HCI_Status = HCI_Status;
    mblk->Para.MHC_Cmd_Status_Para.By_Primitive = By_Primitive;

    SndMsgBlk(mblk, 4);

}



void LL_Msg_Event_Encrypt_Change(Uint8 HCI_Status, Uint8 ConnID, Uint8 HCI_Encrpt_En)
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x06);
    mblk->Para.MHC_Encrypt_Change_Para.HCI_Status = HCI_Status;
    mblk->Para.MHC_Encrypt_Change_Para.HCI_Conn_Hdl_L = ConnID;
    mblk->Para.MHC_Encrypt_Change_Para.HCI_Encrpt_En = HCI_Encrpt_En;

    SndMsgBlk(mblk, 4);

}


void LL_Msg_Event_Le_PHY_Update_Complete(Uint8 HCI_Status, Uint8 ConnID, Uint8 LL_Phys)
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x14);
    mblk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_Status = HCI_Status;
    mblk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_Conn_Hdl_L = ConnID;
    mblk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_TX_PHY = LL_Phys;
    mblk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_RX_PHY = LL_Phys;

    SndMsgBlk(mblk, 4);

}

#line 1225 "..\\..\\..\\source\\LL.c"
void LL_Msg_Event_Le_Data_Length_Change(Uint8 ConnID, Uint8 Tx_length, Uint8 Rx_length)
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x13);
    mblk->Para.MHC_Le_Data_Length_Change_Para.HCI_Conn_Hdl_L = ConnID;
    mblk->Para.MHC_Le_Data_Length_Change_Para.HCI_MaxTxOctets = Tx_length;
    mblk->Para.MHC_Le_Data_Length_Change_Para.HCI_MaxRxOctets = Rx_length;

    SndMsgBlk(mblk, 4);

}



void MLL_HCI_Null(MBLK *pMBlk)
{
    __nop();
}


void HCLL_Disconnect(MBLK *pMBlk)                         
{
    Uint8 i;

    i = pMBlk->Para.HCLL_Disconnect_Para.HCI_Conn_Hdl_L;
    if(i >= 4+1)
    {
        return;
    }
    if(LL_Conn_Para[i].LE_Conn_Para.LL_Conn_ID != 0x01)
    {
        switch(pMBlk->Para.HCLL_Disconnect_Para.HCI_Reason)                 
        {
        case 0x13:
            LL_Msg_CTRL_PKT(&pMBlk->Para.HCLL_Disconnect_Para.HCI_Reason, 0x02, 1, i);



            LL_Msg_Event_Status(0x00, (0xB0+0x01));

            break;

        case 0x05:
        case 0x14:
        case 0x15:
        case 0x1A:
        case 0x29:
        case 0x3B:
            break;

        default:
            break;
        }
    }
    __nop();
}


void HCLL_Read_Remote_Ver_Info(MBLK *pMBlk)               
{
    Uint8 i;
    Uint8 *pCtrlPkt_Data;

    i = pMBlk->Para.HCLL_Read_Remote_Ver_Info_Para.HCI_Conn_Hdl_L;
    if(i<4+1)
    {



        LL_Msg_Event_Status(0x00, (0xB0+0x02));

        pCtrlPkt_Data = pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data;
        Knl_CodeCpy(pCtrlPkt_Data, LL_VERSION, 5);
        LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x0C, 5, i);
    }
}


void HCLL_Set_Event_Mask(MBLK *pMBlk)                     
{
    MBLK *mblk;

    
    Event_Mask[0] = pMBlk->Para.HCLL_Set_Event_Mask_Para.HCI_Event_Mask[0];     
    Event_Mask[1] = pMBlk->Para.HCLL_Set_Event_Mask_Para.HCI_Event_Mask[1];     
    Event_Mask[2] = pMBlk->Para.HCLL_Set_Event_Mask_Para.HCI_Event_Mask[3];     
    Event_Mask[3] = pMBlk->Para.HCLL_Set_Event_Mask_Para.HCI_Event_Mask[5];     
    Event_Mask[4] = pMBlk->Para.HCLL_Set_Event_Mask_Para.HCI_Event_Mask[7];     

    
    mblk = GetMsgBlk_L2_wSize(sizeof(mblk->Para.HCLL_Set_Event_Mask_Para));
    if(mblk == (MBLK *)0)
    {
        return;
    }
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x03);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);

}


void HCLL_Reset(MBLK *pMBlk)                              
{

    extern void MCU_WDTmr_En(void);

    MCU_WDTmr_En();
#line 1361 "..\\..\\..\\source\\LL.c"
}


void HCLL_Read_Transmit_Pwr_Level(MBLK *pMBlk)            
{
    pMBlk = pMBlk;
}


void HCLL_Set_Ctrler_to_Host_Flow_Ctrl(MBLK *pMBlk)       
{
    pMBlk = pMBlk;
}


void HCLL_Host_Buffer_Size(MBLK *pMBlk)                   
{
    pMBlk = pMBlk;
}


void HCLL_Host_Num_of_Completed_Packets(MBLK *pMBlk)      
{
    pMBlk = pMBlk;
}


void HCLL_Set_Event_Mask_Page_2(MBLK *pMBlk)              
{
    MBLK *mblk;

    
    Event_Mask2[0] = pMBlk->Para.HCLL_Set_Event_Mask_Page_2_Para.HCI_Event_Mask_Page2[2];
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x09);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);

}


void HCLL_Read_Le_Host_Support(MBLK *pMBlk)               
{
    pMBlk = pMBlk;
}


void HCLL_Write_Le_Host_Support(MBLK *pMBlk)              
{
    pMBlk = pMBlk;
}


void HCLL_Read_authen_payload_timeout(MBLK *pMBlk)        
{
    pMBlk = pMBlk;
}


void HCLL_Write_authen_payload_timeout(MBLK *pMBlk)       
{
    pMBlk = pMBlk;
}


void HCLL_Read_Local_Ver_Info(MBLK *pMBlk)                
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    Knl_CodeCpy(&mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1], LE_LOCAL_VER_INFO, sizeof(LE_LOCAL_VER_INFO));
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x0E);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);

}


void HCLL_Read_Local_Supported_Cmd(MBLK *pMBlk)           
{
    MBLK *mblk;

    mblk = GetMsgBlk();



    Knl_CodeCpy(&mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1], TAB_SUPPORTED_COMMANDS_MASK, ((6+31+7)-6-1));

    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x0F);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);



    mblk = GetMsgBlk();
    Knl_CodeCpy(mblk->Para.MHC_Event_HCI_Raw_Para.Data, &TAB_SUPPORTED_COMMANDS_MASK[((6+31+7)-6-1)], (64-((6+31+7)-6-1)));
    mblk->Primitive = (0x60+0x12);
    mblk->Para.MHC_Event_HCI_Raw_Para.HCI_Para_Length = (64-((6+31+7)-6-1));

    SndMsgBlk(mblk, 4);


}


void HCLL_Read_Local_Supported_Feat(MBLK *pMBlk)          
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    Knl_CodeCpy(&mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1], TAB_LMP_FEATURES_MASK, sizeof(TAB_LMP_FEATURES_MASK));
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x10);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);

}


void HCLL_Read_buffer_size(MBLK *pMBlk)                   
{
    pMBlk = pMBlk;
}


void HCLL_Read_BD_Addr(MBLK *pMBlk)                       
{
    MBLK *mblk;

    mblk = GetMsgBlk();

    Knl_MemCpy(&mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1], ble_device_param.ble_deviceAddr_param.addr, 6);

    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x12);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;


    SndMsgBlk(mblk, 4);

}


void HCLL_Read_rssi(MBLK *pMBlk)                          
{
    MBLK *mblk;
    Uint8 i;

    i = pMBlk->Para.HCLL_Read_Rssi_Para.HCI_Conn_Hdl_L;
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x13);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] = i;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2] = 0x01;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[3] = LL_Conn_Para[i].LE_Conn_Para.RF_Rssi;


    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Set_Event_Mask(MBLK *pMBlk)                  
{
    MBLK *mblk;

    LE_Event_Mask[0] = pMBlk->Para.HCLL_LE_Set_Event_Mask_Para.HCI_LE_Event_Mask[0];
    LE_Event_Mask[1] = pMBlk->Para.HCLL_LE_Set_Event_Mask_Para.HCI_LE_Event_Mask[1];
    LE_Event_Mask[2] = pMBlk->Para.HCLL_LE_Set_Event_Mask_Para.HCI_LE_Event_Mask[2];
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x14);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Read_Buffer_Size(MBLK *pMBlk)                
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x11);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] = (6+31+7);
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[3] = 0x01;

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Read_Local_Supported_Feat(MBLK *pMBlk)       
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x16);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    Knl_CodeCpy(&mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1], LL_FEATURE, sizeof(LL_FEATURE));
    if((ble_device_param.ble_deviceChipId&0x70) == 0x70)
    {
        mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] &= (~0x01);
    }

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Set_Random_Address(MBLK *pMBlk)              
{
    MBLK    *mblk;
    Uint8   eventRsp;

    if (LL_Msg_AdvScnConnUpd & (0x01 |
                                0x02 |
                                0x04))
    {
        
        eventRsp = 0x0C;
    }
    else
    {
        __disable_irq();
        Knl_MemCpy(BD_Rand_AddrBuf, pMBlk->Para.HCLL_LE_Set_Random_Addr_Para.HCI_Random_Addr, 6);
        __disable_irq();
        if((LL_Msg_AdvScnConnUpdSts & 0x20))
        {
            LL_Msg_AdvScnConnUpd &= (~0x20);
        }
        else
        {
            LL_Msg_AdvScnConnUpd |= 0x20;
        }
        __enable_irq();
        eventRsp = 0x00;
    }

    
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);
#line 1676 "..\\..\\..\\source\\LL.c"
    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x17);
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = eventRsp;


    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Set_Adv_Param(MBLK *pMBlk)                   
{
    MBLK    *mblk;
    Uint8   i;
    Uint8   eventRsp;

    if (LL_Msg_AdvScnConnUpd & 0x01)
    {
        
        eventRsp = 0x0C;
    }
#line 1710 "..\\..\\..\\source\\LL.c"
    else
    {
        Knl_MemCpy((uint8_t *)&LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Interval_MinI, (uint8_t *)&(pMBlk->Para.HCLL_LE_Set_Adv_Param_Para.HCI_Adv_Interval_Min), (2*2+1*5+1*6));

        i = LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Type;


        if(i != 0x01)
        {
            LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Interval_Min = (LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Interval_MinI*5);
        }
        else
        {
            LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Interval_Min = (17*3);
        }
#line 1739 "..\\..\\..\\source\\LL.c"

        i = TABLE_CONV_ADV_TYPE_HCI_LL[i];
        if(i == 0x01)
        {
            if(LL_Adv_Para_UpdBuf.Adv_Para.LL_DirectAddr_Type != 0x00)
            {
                i |= 0x80;
            }
        }
        if(LL_Adv_Para_UpdBuf.Adv_Para.LL_Own_Addr_Type != 0x00)
        {
            i |= 0x40;
        }
        LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Type = i;

        Knl_CodeCpy(Ch_ADV_Ch_Hop_TableBuf, CH_ADV_CH_HOP_BY_MAP_TABLE[LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Channel_Map], 4);

        __disable_irq();
        if((LL_Msg_AdvScnConnUpdSts & 0x20))
        {
            LL_Msg_AdvScnConnUpd &= (~0x20);
        }
        else
        {
            LL_Msg_AdvScnConnUpd |= 0x20;
        }
        __enable_irq();
        eventRsp = 0x00;
    }

    
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);
#line 1778 "..\\..\\..\\source\\LL.c"
    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x18);
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = eventRsp;


    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Read_Adv_Ch_Tx_Pwr(MBLK *pMBlk)              
{
    __nop();
}


void HCLL_LE_Set_Adv_Data(MBLK *pMBlk)                    
{
    MBLK *mblk;
    Uint8 i;

    if(LL_Msg_AdvScnConnUpdSts&0x08)
    {
        LL_Adv_Data_Buf = (Adv_Data_Buf *)&LL_Adv_Para.Adv_Para.LL_Adv_Data;
    }
    else
    {
        LL_Adv_Data_Buf = (Adv_Data_Buf *)&LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Data;
    }
    i = pMBlk->Para.HCLL_LE_Set_Adv_Data_Para.HCI_Adv_Data_Length;
    LL_Adv_Data_Buf->LL_Adv_Data_Length = i+6;
    Knl_MemCpy(LL_Adv_Data_Buf->LL_Adv_Data, pMBlk->Para.HCLL_LE_Set_Adv_Data_Para.HCI_Adv_Data, i);

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x1A);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);

    __disable_irq();
    if((LL_Msg_AdvScnConnUpdSts & 0x08))
    {
        LL_Msg_AdvScnConnUpd &= (~0x08);
    }
    else
    {
        LL_Msg_AdvScnConnUpd |= 0x08;
    }
    __enable_irq();
}


void HCLL_LE_Set_Scan_Response_Data(MBLK *pMBlk)          
{
    MBLK *mblk;
    Uint8 i;

    if(LL_Msg_AdvScnConnUpdSts&0x10)
    {
        LL_Adv_Data_Buf = (Adv_Data_Buf *)&LL_Adv_Para.Adv_Para.LL_Adv_Data;
    }
    else
    {
        LL_Adv_Data_Buf = (Adv_Data_Buf *)&LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Data;
    }
    i = pMBlk->Para.HCLL_LE_Set_Scan_Response_Data_Para.HCI_Scan_Rsp_Length;
    LL_Adv_Data_Buf->LL_ScanRsp_Data_Length = i;
    Knl_MemCpy(LL_Adv_Data_Buf->LL_ScanRsp_Data, pMBlk->Para.HCLL_LE_Set_Scan_Response_Data_Para.HCI_Scan_Rsp, i);

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x1B);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);

    __disable_irq();
    if((LL_Msg_AdvScnConnUpdSts & 0x10))
    {
        LL_Msg_AdvScnConnUpd &= (~0x10);
    }
    else
    {
        LL_Msg_AdvScnConnUpd |= 0x10;
    }
    __enable_irq();
}


void HCLL_LE_Set_Advertise_Enable(MBLK *pMBlk)            
{
    Uint8 i, id, advType;
    MBLK *mblk;

    if((pMBlk->Para.HCLL_LE_Set_Advertise_Enable_Para.HCI_Adv_En & 0x01))     
    {
        i = 0x00;
        __disable_irq();
        if((LL_Msg_AdvScnConnUpdSts ^ 0x01)==0)
        {
            __enable_irq();
        }
        else
        {
            __enable_irq();
            advType = (LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Type&0x0F);   
            __disable_irq();
            id = LL_GetSlaveFreeConnID();                                  
            __enable_irq();
            if(id==4+1)      
            {
                switch(advType)
                {
                case 0x01:
                case 0x00:
                    i = 0x0D;
                    break;

                default:
                    break;
                }
            }
            if(i== 0x00)
            {
#line 1932 "..\\..\\..\\source\\LL.c"
                

                {
                    LL_Adv_Para_UpdBuf.Adv_Para.LL_AdvMap_ID = Ch_ADV_Ch_Hop_TableBuf[0];
                    LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Interval_Max = 0;
                    if(mblk_LL_conn_Para[id]==(MBLK *)0)
                    {
                        mblk = GetMsgBlk();
                    }
                    switch(advType)
                    {
                    case 0x01:
                        if(LL_Adv_Para_UpdBuf.Adv_Para.LL_DirectAddr_Type)
                        {
                            i = 0x01;
                        }
                        else
                        {
                            i = 0x00;
                        }
                        mblk->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddrType = i;
                        mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Status = 0x3C;
                        Knl_MemCpy(mblk->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddr, LL_Adv_Para_UpdBuf.Adv_Para.LL_DirectAddr, 6);
                        Knl_CodeCpy(&mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Conn_IntervalL, TAB_ZERO_128, 6);
                    case 0x00:
                        mblk->Primitive = (0x60+0x0A);
                        mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Conn_Hdl_L = id;
                        mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Role = 0x01;
                        mblk_LL_conn_Para[id] = mblk;
                        LL_Adv_Para_UpdBuf.Adv_Para.LL_AdvConn_ID = id;
                        break;

                    default:
                        mblk->Primitive = (0xB0+0x00);
                        SndMsgBlk(mblk, 0);
                        break;
                    }
                    i = 0x00;
                }
            }
        }
        __disable_irq();

        if(i==0x00)
        {
            LL_Msg_AdvScnConnUpd |= 0x01;
        }
        __enable_irq();
    }
    else     
    {
        __disable_irq();
        LL_Msg_AdvScnConnUpd &= (~0x01);
        i = LL_Msg_AdvScnConnUpdSts;
        __enable_irq();
        if((i&0x01)==0)
        {
            id = LL_Adv_Para_UpdBuf.Adv_Para.LL_AdvConn_ID;
            if(LL_Conn_Para[id].LE_Conn_Para.LL_Conn_ID != 0x02)
            {
                if(id < 4+1)
                {
                    LL_Conn_Para[id].LE_Conn_Para.LL_Conn_ID = 0x01;

                    mblk = mblk_LL_conn_Para[id];
                    if(mblk!=(MBLK *)0)
                    {
                        mblk->Primitive = (0xB0+0x00);
                        mblk_LL_conn_Para[id] = (MBLK *)0;
                        SndMsgBlk(mblk, 0);
                    }
                }
            }
        }
        i = 0x00;
    }
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x1C);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = i;

    SndMsgBlk(mblk, 4);
#line 2031 "..\\..\\..\\source\\LL.c"
}


void HCLL_LE_Set_scan_param(MBLK *pMBlk)                  
{
    MBLK    *mblk;
    Uint8   eventRsp;

    __disable_irq();
    if (LL_Msg_AdvScnConnUpd & 0x02)
    {
        __enable_irq();
        
        eventRsp = 0x0C;
    }
    else
    {
        __enable_irq();
        eventRsp = 0x00;
        Knl_MemCpy((Uint8 *)&LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Interval, (Uint8 *)&pMBlk->Para.HCLL_LE_Set_Scan_Param_Para.HCI_LE_Scan_Interval, 4);
        LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_ScanType = pMBlk->Para.HCLL_LE_Set_Scan_Param_Para.HCI_LEScanType;
        LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_Own_Addr_Type = pMBlk->Para.HCLL_LE_Set_Scan_Param_Para.HCI_Own_Addr_Type;
        LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_ScanFilterPolicy = pMBlk->Para.HCLL_LE_Set_Scan_Param_Para.HCI_ScannFilterPolicy;


        if(LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Interval>0x3333)
        {
            LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Interval = 0x3333;
        }
        LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Interval = (LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Interval*5);
        if(LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Window>0x3333)
        {
            LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Window = 0x3333;
        }
        LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Window = (LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Window*5);
#line 2078 "..\\..\\..\\source\\LL.c"

        LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Interval = LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Interval - LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Window;
        if((LL_Msg_AdvScnConnUpdSts & 0x40))
        {
            LL_Msg_AdvScnConnUpd &= (~0x40);
        }
        else
        {
            LL_Msg_AdvScnConnUpd |= 0x40;
        }
    }

    
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);
#line 2099 "..\\..\\..\\source\\LL.c"
    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x1D);
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = eventRsp;


    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Set_scan_enable(MBLK *pMBlk)                 
{
    MBLK    *mblk;




    if((pMBlk->Para.HCLL_LE_Set_Scan_Enable_Para.HCI_LE_Scan_En & 0x01) != 0)
    {
#line 2144 "..\\..\\..\\source\\LL.c"
        {
            __disable_irq();
            if((LL_Msg_AdvScnConnUpd & 0x02)==0)
            {
                LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_AdvMap_ID = CH_ADV_CH_HOP_BY_MAP_TABLE[0x07][0];               
                LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_ScanAccl = LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_LE_Scan_Window; 
                LL_Msg_AdvScnConnUpd |= 0x02;
            }
            __enable_irq();
        }
    }
    else
    {
        __disable_irq();
        LL_Msg_AdvScnConnUpd &= (~0x02);
        __enable_irq();
    }

    
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);
#line 2171 "..\\..\\..\\source\\LL.c"
    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x1E);
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;


    SndMsgBlk(mblk, 4);


}



void HCLL_LE_Create_Conn(MBLK *pMBlk)                     
{

    Uint8 i;
    LL_Conn *pLL_conn;
    MBLK *mblk;
    Uint16 i16, j16;

    __disable_irq();
    if((LL_Msg_AdvScnConnUpd & 0x04))
    {
        __enable_irq();
        mblk = GetMsgBlk();
        mblk->Primitive = (0x60+0x01);





        mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x1F);

        mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x0C;

        SndMsgBlk(mblk, 4);

        return;
    }
    else
    {
        __enable_irq();



        LL_Msg_Event_Status(0x00, (0xB0+0x1F));

    }
    i = LL_GetMasterFreeConnID();
    if(i==4)
    {
        return;
    }
    Knl_MemCpy((uint8_t *)&LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval, (uint8_t *)&pMBlk->Para.HCLL_LE_Create_Conn_Para.HCI_LE_Scan_Interval, (1*3+6+2*2));

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x0A);
    mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Status = 0x00;
    mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Role = 0x00;
    mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Master_Clk_Accuracy = 0;
    Knl_MemCpy(&mblk->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddrType, &pMBlk->Para.HCLL_LE_Create_Conn_Para.HCI_PeerAddrType, (6+1));
    mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Conn_Hdl_L = i;
    mblk_LL_conn_Para[i] = mblk;
    LL_Init_Para.LE_Init_Para.LL_Conn_ID = i;
    LL_Init_Para.LE_Init_Para.LL_AdvMap_ID = CH_ADV_CH_HOP_BY_MAP_TABLE[0x07][0];
    pLL_conn = &LL_Conn_Para[i];        
    pLL_conn->LE_Conn_Para.LL_Conn_ID = 0x03;
    pLL_conn->LE_Conn_Para.LL_AccessAddr = LL_REF_ACS_ADDR[(ble_device_param.ble_deviceAddr_param.addr[0]&0x03)];       

    Knl_CodeCpy(pLL_conn->LE_Conn_Para.LL_CRC_Init, LL_REF_CRC_INI[i], 3);
    Knl_CodeCpy(pLL_conn->LE_Conn_Para.LL_RF_Data_Ch_ReM, LL_RF_DATA_CH, 37);
    Knl_MemCpy(pLL_conn->LE_Conn_Para.LL_ChMapReM, LL_Ref_ChMap, 5);
    i = PRIME_NUM[(i&0x07)];
    pLL_conn->LE_Conn_Para.LL_HopIncrement = i;
    pLL_conn->LE_Conn_Para.LL_CurrentCH = i;

    i16 = pMBlk->Para.HCLL_LE_Create_Conn_Para.HCI_ConnIntervalMin;
    j16 = pMBlk->Para.HCLL_LE_Create_Conn_Para.HCI_ConnIntervalMax;
    if(i16 == j16)
    {
        LL_Init_Para.LE_Init_Para.LL_ConnInterval = i16;
    }
    else
    {
        i16 = ((i16+j16)>>1)|0x01;
        LL_Init_Para.LE_Init_Para.LL_ConnInterval = i16;
    }
    pLL_conn->LE_Conn_Para.LL_ConnIntervalOrg = i16;   
    pLL_conn->LE_Conn_Para.LL_SvToutAccu = i16<<2;     
    i16 = i16*10;
    pLL_conn->LE_Conn_Para.LL_ConnInterval = i16;      
    pLL_conn->LE_Conn_Para.LL_SvisionTimeout = i16;    
    for(i=0; i<7; i++)
    {
        if(LL_RF_PARA_TX_WIN_SIZE_MAP[i] > i16)
        {
            break;
        }
    }
    pLL_conn->LE_Conn_Para.WinSize_DataHdr = i;
    LL_Init_Para.LE_Init_Para.LL_ConnLatency = pMBlk->Para.HCLL_LE_Create_Conn_Para.HCI_ConnLatency;
    pLL_conn->LE_Conn_Para.LL_ConnLatency = pMBlk->Para.HCLL_LE_Create_Conn_Para.HCI_ConnLatency;
    
    LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval = 4;
    LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window = 4;
    
    i16 = pMBlk->Para.HCLL_LE_Create_Conn_Para.HCI_SvisionTimeout;
    LL_Init_Para.LE_Init_Para.LL_SvisionTimeout = i16;            
    pLL_conn->LE_Conn_Para.LL_SvisionTimeoutUpd = (i16<<3);       
    Knl_MemCpy(&mblk->Para.MHC_Le_Conn_Complete_Para.HCI_Conn_IntervalL, (Uint8 *)&LL_Init_Para.LE_Init_Para.LL_ConnInterval, 6);
    LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval = (LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval*5);
    if(LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval==0)
    {
        LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval = 0xFFFF;
    }
    LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window = (LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window*5);
    if(LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window==0)
    {
        LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window = 0xFFFF;
    }
    __disable_irq();
    LL_Msg_AdvScnConnUpd |= 0x04;
    __enable_irq();

}
#line 2423 "..\\..\\..\\source\\LL.c"


void HCLL_LE_Create_conn_cancel(MBLK *pMBlk)              
{

    Uint8 i;
    MBLK *mblk;

    if((LL_Msg_AdvScnConnUpd & 0x04))
    {
        i = LL_Init_Para.LE_Init_Para.LL_Conn_ID;
        LL_Init_Para.LE_Init_Para.LL_Conn_ID = 0xFF;
        LL_ReleaseConnID(i);
        mblk = mblk_LL_conn_Para[i];
        if(mblk!=(MBLK *)0)
        {
            mblk_LL_conn_Para[i] = (MBLK *)0;
        }
        else
        {
            mblk = GetMsgBlk();
        }
        mblk->Primitive = (0x60+0x01);





        mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x20);

        mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

        SndMsgBlk(mblk, 4);

        __disable_irq();
        LL_Msg_AdvScnConnUpd &= (~0x04);
        __enable_irq();
    }

}


void HCLL_LE_Read_White_List_Size(MBLK *pMBlk)            
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x21);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] = 4;

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Clear_White_List(MBLK *pMBlk)                
{
    MBLK *mblk;
    Uint8 status, i;

    __disable_irq();
    i = LL_Msg_AdvScnConn;
    __enable_irq();
    if((i & (0x10|0x20)) == 0)
    {
        for(i=0; i<4; i++)
        {
            LL_WhiteList_Para[i].LE_WhiteList_Para.AddrType = 0xFC;
        }
        status = 0x00;
    }
    else
    {
        status = 0x0C;
    }
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x22);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = status;

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Add_Device_to_White_List(MBLK *pMBlk)        
{
    MBLK *mblk;
    Uint8 status, i;

    __disable_irq();
    i = LL_Msg_AdvScnConn;
    __enable_irq();
    if((i & (0x10|0x20)) == 0)
    {
        for(status=0; status<4; status++)
        {
            if(LL_WhiteList_Para[status].LE_WhiteList_Para.AddrType != 0xFC)
            {
                if(Knl_MemComp(&LL_WhiteList_Para[status].LE_WhiteList_Para.AddrType, &pMBlk->Para.HCLL_LE_Add_Device_To_White_List_Para.HCI_Addr_Type, sizeof(LE_WhiteList_Para)) == 0)
                {
                    break;
                }
            }
        }

        if(status == 4)
        {
            for(status=0; status<4; status++)
            {
                if(LL_WhiteList_Para[status].LE_WhiteList_Para.AddrType == 0xFC)
                {
                    Knl_MemCpy(&LL_WhiteList_Para[status].LE_WhiteList_Para.AddrType, &pMBlk->Para.HCLL_LE_Add_Device_To_White_List_Para.HCI_Addr_Type, sizeof(LE_WhiteList_Para));
                    break;
                }
            }
        }
        if(status == 4)
        {
            status = 0x07;
        }
        else
        {
            status = 0x00;
        }
    }
    else
    {
        status = 0x0C;
    }
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x23);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = status;

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Rmv_Device_from_White_List(MBLK *pMBlk)      
{
    MBLK *mblk;
    Uint8 status, i;

    __disable_irq();
    i = LL_Msg_AdvScnConn;
    __enable_irq();
    if((i & (0x10|0x20)) == 0)
    {
        for(status=0; status<4; status++)
        {
            if(LL_WhiteList_Para[status].LE_WhiteList_Para.AddrType != 0xFC)
            {
                if(Knl_MemComp(&LL_WhiteList_Para[status].LE_WhiteList_Para.AddrType, &pMBlk->Para.HCLL_LE_Add_Device_To_White_List_Para.HCI_Addr_Type, sizeof(LE_WhiteList_Para)) == 0)
                {
                    LL_WhiteList_Para[status].LE_WhiteList_Para.AddrType = 0xFC;
                    break;
                }
            }
        }
        if(status == 4)
        {
            status = 0x12;
        }
        else
        {
            status = 0x00;
        }
    }
    else
    {
        status = 0x0C;
    }
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x24);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = status;

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Conn_update(MBLK *pMBlk)                     
{
    Uint8 i, j;
    Uint16 i16, j16;
    LL_Conn *pLL_Conn;
    MBLK *mblk;


    i16 = pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMin;
    j16 = pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMax;
    if(i16 != j16)
    {
        i16 = ((i16+j16)>>1)|0x01;
        pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMin = i16;
        pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMax = i16;
    }
    mblk = GetMsgBlk();
    Knl_MemCpy((Uint8 *)&mblk->Para.HCLL_LE_Ctrl_Pkt_Conn_Update_Para.HCI_CtrlPkt_Interval, (Uint8 *)&pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMax, 6);


    pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMax = (pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMax*10);   
    pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_SvisionTimeout = (pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_SvisionTimeout<<3);     




    i = pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_Conn_Hdl_L;
    pLL_Conn = &LL_Conn_Para[i];      



    LL_Msg_Event_Status(0x00, (0xB0+0x25));

    if(Knl_MemComp((Uint8 *)&pLL_Conn->LE_Conn_Para.LL_ConnIntervalUpd, (Uint8 *)&pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMax, 6) == 0)     
    {
        mblk->Primitive = (0xB0+0x00);
        SndMsgBlk(mblk, 0);
        return;
    }

    for(j=0; j<7; j++)
    {
        if(LL_RF_PARA_TX_WIN_SIZE_MAP[j] > i16)
        {
            break;
        }
    }
    mblk->Para.HCLL_LE_Ctrl_Pkt_Conn_Update_Para.HCI_CtrlPkt_WinSize = j+1;

    mblk->Para.HCLL_LE_Ctrl_Pkt_Conn_Update_Para.HCI_CtrlPkt_Instant = (pLL_Conn->LE_Conn_Para.LL_EventCounter+(4+1+3));

    LL_Msg_CTRL_PKT(&mblk->Para.HCLL_LE_Ctrl_Pkt_Conn_Update_Para.HCI_CtrlPkt_WinSize, 0x00, 11, i);
    mblk->Primitive = (0x60+0x0C);
    Knl_MemCpy_Fwd(&mblk->Para.MHC_Le_Conn_Update_Complete_Para.HCI_ConnIntervalL, (Uint8 *)&mblk->Para.HCLL_LE_Ctrl_Pkt_Conn_Update_Para.HCI_CtrlPkt_Interval, 6);
    mblk->Para.MHC_Le_Conn_Update_Complete_Para.HCI_Conn_Hdl_L = i;
    mblk_LL_conn_Para[i] = mblk;
    pLL_Conn->LE_Conn_Para.LL_EventCounterUpd = (pLL_Conn->LE_Conn_Para.LL_EventCounter+(4+1+3));
    pLL_Conn->LE_Conn_Para.LL_ConnIntervalOrgUpd = i16;
    Knl_MemCpy((Uint8 *)&pLL_Conn->LE_Conn_Para.LL_ConnIntervalUpd, (Uint8 *)&pMBlk->Para.HCLL_LE_Conn_Update_Para.HCI_ConnIntervalMax, 6);
}


void HCLL_LE_Set_Host_Ch_Classification(MBLK *pMBlk)      
{

    MBLK *mblk;
    Uint8 i;
    Uint16 i16;

    if(Knl_MemComp(LL_Ref_ChMap, pMBlk->Para.HCLL_LE_Set_Host_Ch_Classification_Para.HCI_Channel_Map, 5) == 0)
    {
        return;
    }
    Knl_MemCpy(LL_Ref_ChMap, pMBlk->Para.HCLL_LE_Set_Host_Ch_Classification_Para.HCI_Channel_Map, 5);
    for(i=0; i<4; i++)
    {
        if(LL_Conn_Para[i].LE_Conn_Para.LL_Conn_ID != 0x01)
        {
            i16 = LL_Conn_Para[i].LE_Conn_Para.LL_EventCounter+8;
            LL_Conn_Para[i].LE_Conn_Para.LL_EventCounterUpd = i16;
            pMBlk->Para.HCLL_LE_Set_Host_Ch_Classification_Para.LL_EventCounterUpd = i16;

            Knl_MemCpy(LL_Conn_Para[i].LE_Conn_Para.LL_ChMapReM, LL_Ref_ChMap, 5);
            LL_Msg_CTRL_PKT(pMBlk->Para.HCLL_LE_Set_Host_Ch_Classification_Para.HCI_Channel_Map, 0x01, (5+2), i);
        }
    }
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x26);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    SndMsgBlk(mblk, 4);


}


void HCLL_LE_Read_ch_map(MBLK *pMBlk)                     
{
    MBLK *mblk;
    Uint8 i;

    i = pMBlk->Para.HCLL_LE_Read_Ch_map_Para.HCI_Conn_Hdl_L;
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x27);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] = i;




    Knl_MemCpy(&mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2], LL_Conn_Para[i].LE_Conn_Para.LL_ChMapReM, 5);



    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Read_Remote_Used_Feat(MBLK *pMBlk)           
{
    Uint8 *pCtrlPkt_Data;

    pCtrlPkt_Data = pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data;
    Knl_CodeCpy(pCtrlPkt_Data, LL_FEATURE, sizeof(LL_FEATURE));
    if((ble_device_param.ble_deviceChipId&0x70) == 0x70)
    {
        *(pCtrlPkt_Data+1) &= (~0x01);
    }

    if(pMBlk->Para.HCLL_LE_Read_Remote_Used_Feat_Para.HCI_Conn_Hdl_L < 4)
    {
        LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x08, sizeof(LL_FEATURE), pMBlk->Para.HCLL_LE_Read_Remote_Used_Feat_Para.HCI_Conn_Hdl_L);
    }
    else

    {

        LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x0E, sizeof(LL_FEATURE), pMBlk->Para.HCLL_LE_Read_Remote_Used_Feat_Para.HCI_Conn_Hdl_L);

    }



    LL_Msg_Event_Status(0x00, (0xB0+0x28));

}


void HCLL_LE_Encrypt(MBLK *pMBlk)                         
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x50+0x01);
    Knl_MemCpy(mblk->Para.Data, pMBlk->Para.Data, (16+16+0));
    SndMsgBlkEC(mblk);
}


void HCLL_LE_Rand(MBLK *pMBlk)                            
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x50+0x02);
    Knl_MemCpy(mblk->Para.Data, pMBlk->Para.Data, 0);
    SndMsgBlkEC(mblk);
}


void HCLL_LE_Start_encryption(MBLK *pMBlk)                
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x50+0x03);
    Knl_MemCpy(mblk->Para.Data, pMBlk->Para.Data, (0+1+8+2+16));
    SndMsgBlkEC(mblk);
}


void HCLL_LE_Long_term_key_req_reply(MBLK *pMBlk)         
{
    MBLK *mblk;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x50+0x04);
    Knl_MemCpy(mblk->Para.Data, pMBlk->Para.Data, (0+1+16));
    SndMsgBlkEC(mblk);
}


void HCLL_LE_Long_term_key_req_neg_reply(MBLK *pMBlk)     
{
    Uint8 i;
    Uint8 *p;
    MBLK *mblk;
    LL_Conn *pLL_Conn;

    i = pMBlk->Para.HCLL_LE_Long_Term_key_Req_Neg_Reply_Para.HCI_Conn_Hdl_L;
    pLL_Conn = &LL_Conn_Para[i];
    pLL_Conn->LE_Conn_Para.LL_SMP_Gate = 0;
    pLL_Conn->LE_Conn_Para.LL_SMP_DataCh = 0;

    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x2D);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] = i;




    p = &mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2];

    if((pLL_Conn->LE_Conn_Para.LL_Feature&0x04))
    {
        *p = 0x03;
        *(p+1) = 0x06;
        LL_Msg_CTRL_PKT(p, 0x11, 2, i);
    }
    else
    {
        *p = 0x06;
        LL_Msg_CTRL_PKT(p, 0x0D, 1, i);
    }

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Read_supported_states(MBLK *pMBlk)           
{
    pMBlk = pMBlk;
}


void HCLL_LE_Receiver_test(MBLK *pMBlk)                   
{
    pMBlk = pMBlk;
}


void HCLL_LE_Transmitter_test(MBLK *pMBlk)                
{
    pMBlk = pMBlk;
}


void HCLL_LE_Test_end(MBLK *pMBlk)                        
{
    pMBlk = pMBlk;
}


void HCLL_LE_Remote_conn_param_req_reply(MBLK *pMBlk)     
{
    
    
    __nop();
}


void HCLL_LE_Remote_conn_param_req_neg_reply(MBLK *pMBlk) 
{
    
    
    __nop();
}

void HCLL_LE_Read_Phy(MBLK *pMBlk)     
{
    Uint8 i;
    MBLK *mblk;

    mblk = GetMsgBlk();
    i = pMBlk->Para.HCLL_Le_Read_Phy_Para.HCI_Conn_Hdl_L;
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x3B);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] = i;
#line 2955 "..\\..\\..\\source\\LL.c"
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2] = LL_Conn_Para[i].LE_Conn_Para.LL_Tx_PHYS;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[3] = LL_Conn_Para[i].LE_Conn_Para.LL_Rx_PHYS;
    if(mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2] == 0x04)
    {
        mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2] = 0x03;
    }
    if(mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[3] == 0x04)
    {
        mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[3] = 0x03;
    }


    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Set_Default_Phy(MBLK *pMBlk)     
{
    MBLK *mblk;

     
     
    mblk = GetMsgBlk();

    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x3C);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;

    if((pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_TxPhys&(0|0x04|0xF8))||(pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_RxPhys&(0|0x04|0xF8)))
    {
        mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x11;
    }
    else
    {
        switch(pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_AllPhys)
        {
        case 0x00:
            if(pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_TxPhys != pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_RxPhys)
            {
                mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x11;
            }
            else
            {
                LL_Ref_Default_PHY.Tx_PHYs = pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_TxPhys;
                LL_Ref_Default_PHY.Rx_PHYs = LL_Ref_Default_PHY.Tx_PHYs;            
            }
            break;

        case 0x02:
            if(pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_TxPhys)
            {
                LL_Ref_Default_PHY.Tx_PHYs = pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_TxPhys;
                LL_Ref_Default_PHY.Rx_PHYs = LL_Ref_Default_PHY.Tx_PHYs;            
            }
            break;

        case 0x01:
            if(pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_RxPhys)
            {
                LL_Ref_Default_PHY.Tx_PHYs = pMBlk->Para.HCLL_Le_Set_Default_Phy_Para.HCI_RxPhys;
                LL_Ref_Default_PHY.Rx_PHYs = LL_Ref_Default_PHY.Tx_PHYs;            
            }
            break;

        default:
            break;
        }
    }

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_Set_Phy(MBLK *pMBlk)     
{
    Uint8 i, j;
    LL_Conn *pLL_conn;

    i = pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_Conn_Hdl_L;
    pLL_conn = &LL_Conn_Para[i];
    if(i<4+1)
    {
        j = 0x00;
        if((pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_TxPhys&(0|0x04|0xF8))||(pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_RxPhys&(0|0x04|0xF8)))
        {
            j = 0x11;
        }
        else
        {
            switch(pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_AllPhys)
            {
            case 0x00:
                if(pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_TxPhys != pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_RxPhys)
                {
                    j = 0x11;
                }
                else
                {
                    pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd = pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_TxPhys;
                    pLL_conn->LE_Conn_Para.LL_Rx_PHYsUpd = pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd;    
                }
                break;

            case 0x02:
                if(pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_TxPhys)
                {
                    pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd = pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_TxPhys;
                    pLL_conn->LE_Conn_Para.LL_Rx_PHYsUpd = pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd;    
                }
                break;

            case 0x01:
                if(pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_RxPhys)
                {
                    pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd = pMBlk->Para.HCLL_Le_Set_Phy_Para.HCI_RxPhys;
                    pLL_conn->LE_Conn_Para.LL_Rx_PHYsUpd = pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd;    
                }
                break;

            default:
                pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd = pLL_conn->LE_Conn_Para.LL_Tx_PHYS;
                pLL_conn->LE_Conn_Para.LL_Rx_PHYsUpd = pLL_conn->LE_Conn_Para.LL_Rx_PHYS;
                break;
            }
        }




        LL_Msg_Event_Status(j, (0xB0+0x3D));


        if(j==0x00)
        {
            if(pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd == pLL_conn->LE_Conn_Para.LL_Tx_PHYS)
            {
                pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd = 0;
                pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd = 0;
                LL_Msg_Event_Le_PHY_Update_Complete(0x00, i, pLL_conn->LE_Conn_Para.LL_Tx_PHYS);
            }
            else
            {
                LL_Msg_CTRL_PKT(&pLL_conn->LE_Conn_Para.LL_Tx_PHYsUpd, 0x16, 2, i);
            }
        }
    }
}


void HCLL_LE_Set_Data_Length(MBLK *pMBlk)
{
    Uint8 i;
    MBLK *mblk;

    mblk = GetMsgBlk();
    i = pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_Conn_Hdl_L;
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x3E);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] = i;




    SndMsgBlk(mblk, 4);

    if(pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_RxOctets0 > 251)
    {
        pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_RxOctets0 = 251;
    }
    if(pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_RxTime > ((251+(10+4))*8))
    {
        pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_RxTime = ((251+(10+4))*8);
    }
    if(pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_TxOctets0 > 251)
    {
        
        pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_TxOctets0 = LL_Conn_Para[i].LE_Conn_Para.LL_Tx_length_2M;
    }
    if(pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_TxTime > ((251+(10+4))*8))
    {
        pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_TxTime = ((251+(10+4))*8);
    }

    Knl_MemCpy((Uint8 *)LL_Length_Prefer[i], (Uint8 *)&pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_RxOctets0, sizeof(LL_LENGTH));
    LL_Msg_CTRL_PKT((Uint8 *)&pMBlk->Para.HCLL_LE_Set_Data_Length_Para.HCI_RxOctets0, 0x14, sizeof(LL_LENGTH), i);
}

void LLHC_ERR_CODE_Unknown_Hci_Command(MBLK *pMBlk)       
{
    MBLK *mblk;

    mblk = GetMsgBlk();

    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x34);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x01;

    SndMsgBlk(mblk, 4);

}


void LLHC_ERR_CODE_Invalid_Lmp_Parameters(MBLK *pMBlk)    
{
    MBLK *mblk;

    mblk = GetMsgBlk();

    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x35);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x1E;

    SndMsgBlk(mblk, 4);

}


void HCLL_LE_ACL_Data_Pkt(MBLK *pMBlk)
{
#line 3228 "..\\..\\..\\source\\LL.c"
}


void HCLL_LE_ACL_Data_Pkt_Extend(MBLK *pMBlk)
{
    __nop();
}

void MLL_LE_Ctrl_Pkt(MBLK *pMBlk)
{
    __nop();
}


void LLHC_LE_Ctrl_Pkt(MBLK *pMBlk)
{
    MBLK *mblk;
    Uint8 i;
    Uint8 mskPhy;
    LL_Conn *pLL_Conn;
    Uint8 *pCtrlPkt_Data;
    Uint16 *pCtrlPkt_Data16;
    Uint16 Length16;

    pCtrlPkt_Data = pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data;
    pCtrlPkt_Data16 = (Uint16 *)pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data;
    i = pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_Conn_Hdl_L;
    pLL_Conn = &LL_Conn_Para[i];
    switch(pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode)
    {
    case 0x00:
        if(pLL_Conn->LE_Conn_Para.LL_Conn_ID == (0x40|0x16))          
        {
            pLL_Conn->LE_Conn_Para.LL_Conn_ID = (0x80|0x00);
            LL_Msg_Event_Le_PHY_Update_Complete(0x2A, i, pLL_Conn->LE_Conn_Para.LL_Tx_PHYS);
        }

#line 3279 "..\\..\\..\\source\\LL.c"

        if(Knl_MemComp((Uint8 *)&pLL_Conn->LE_Conn_Para.LL_ConnInterval, (Uint8 *)&pLL_Conn->LE_Conn_Para.LL_ConnIntervalUpd, 6))    
        {
            mblk = GetMsgBlk();
            mblk->Primitive = (0x60+0x0C);
            mblk->Para.MHC_Le_Conn_Update_Complete_Para.HCI_Conn_Hdl_L = i;
            Knl_MemCpy(&mblk->Para.MHC_Le_Conn_Update_Complete_Para.HCI_ConnIntervalL, (pCtrlPkt_Data+3), 6);
            SndMsgBlk(mblk, 4);
        }
        break;

    case 0x01:
        if(pLL_Conn->LE_Conn_Para.LL_Conn_ID == (0x40|0x16))          
        {
            pLL_Conn->LE_Conn_Para.LL_Conn_ID = (0x80|0x01);
            LL_Msg_Event_Le_PHY_Update_Complete(0x2A, i, pLL_Conn->LE_Conn_Para.LL_Tx_PHYS);
        }
        break;

    case 0x02:
        break;

    case 0x03:
    case 0x04:
        mblk = GetMsgBlk();
        mblk->Primitive = (0x50+0x05);
        Knl_MemCpy(mblk->Para.Data, pMBlk->Para.Data, (2+1+1+8+2+8+4));
        SndMsgBlkEC(mblk);
        break;

    case 0x05:
        LL_Msg_CTRL_PKT(0, 0x06, 0, i);
        break;

    case 0x06:


        if(i >= 4)

        {
            LL_Msg_CTRL_PKT(0, 0x06, 0, i);
        }

        if(pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[0] == 2)
        {
            mblk = GetMsgBlk();
            mblk->Primitive = (0x60+0x07);
            mblk->Para.MHC_Encrypt_Key_Refresh_Complete_Para.HCI_Conn_Hdl_L = i;

            SndMsgBlk(mblk, 4);

        }
        else
        {
            LL_Msg_Event_Encrypt_Change(0x00, i, 1);
        }
        break;

    case 0x07:
        switch(pLL_Conn->LE_Conn_Para.LL_Conn_ID)
        {
        case (0x40|0x0E):
            pLL_Conn->LE_Conn_Para.LL_Conn_ID = 0x04;
            mblk = GetMsgBlk();
            mblk->Primitive = (0x60+0x0D);
            mblk->Para.MHC_Le_Read_Remote_Used_Feat_Complete_Para.HCI_Status = 0x1A;
            mblk->Para.MHC_Le_Read_Remote_Used_Feat_Complete_Para.HCI_Conn_Hdl_L = i;
            SndMsgBlk(mblk, 4);
            break;

        case (0x40|0x16):
            pLL_Conn->LE_Conn_Para.LL_Conn_ID = 0x04;
            LL_Msg_Event_Le_PHY_Update_Complete(0x1A, i, pLL_Conn->LE_Conn_Para.LL_Tx_PHYS);
            pLL_Conn->LE_Conn_Para.LL_Tx_PHYsUpd = 0;
            pLL_Conn->LE_Conn_Para.LL_Rx_PHYsUpd = 0;
            break;

        default:
            if(*(pCtrlPkt_Data) == 0x16)
            {
                LL_Msg_Event_Le_PHY_Update_Complete(0x1A, i, pLL_Conn->LE_Conn_Para.LL_Tx_PHYS);
                pLL_Conn->LE_Conn_Para.LL_Tx_PHYsUpd = 0;
                pLL_Conn->LE_Conn_Para.LL_Rx_PHYsUpd = 0;
            }
            break;
        }

        break;

    case 0x08:
    case 0x0E:
        Knl_CodeCpy(pCtrlPkt_Data, LL_FEATURE, sizeof(LL_FEATURE));
        if((ble_device_param.ble_deviceChipId&0x70) == 0x70)
        {
            *(pCtrlPkt_Data+1) &= (~0x01);
        }
        LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x09, sizeof(LL_FEATURE), i);
        break;

    case 0x09:
        mblk = GetMsgBlk();
        mblk->Primitive = (0x60+0x0D);
        mblk->Para.MHC_Le_Read_Remote_Used_Feat_Complete_Para.HCI_Status = 0x00;
        mblk->Para.MHC_Le_Read_Remote_Used_Feat_Complete_Para.HCI_Conn_Hdl_L = i;
        Knl_MemCpy(mblk->Para.MHC_Le_Read_Remote_Used_Feat_Complete_Para.HCI_LE_Feat, pCtrlPkt_Data, sizeof(LL_FEATURE));
        SndMsgBlk(mblk, 4);
        break;

    case 0x0A:
        LL_Msg_CTRL_PKT(0, 0x0B, 0, i);
        break;

    case 0x0B:

        if(i<4)
        {
            LL_Msg_CTRL_PKT(0, 0x0B, 0, i);
        }

        break;

    case 0x0C:
        switch(pLL_Conn->LE_Conn_Para.LL_Conn_ID)
        {
        case (0x40|0x03):
        case (0x80|0x0C):
            Knl_CodeCpy(pCtrlPkt_Data, LL_VERSION, 5);
            LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x0C, 5, i);
            break;

        default:
            if((Event_Mask[1] & 0x08) != 0)
            {
                mblk = GetMsgBlk();
                mblk->Primitive = (0x60+0x05);
                mblk->Para.MHC_Read_Remote_Ver_Info_Complete_Para.HCI_Conn_Hdl_L = i;
                Knl_MemCpy(&mblk->Para.MHC_Read_Remote_Ver_Info_Complete_Para.HCI_Ver, pCtrlPkt_Data, 5);

                SndMsgBlk(mblk, 4);

            }
            break;
        }
        break;

    case 0x11:
        pCtrlPkt_Data += 1;
    case 0x0D:
        switch(pLL_Conn->LE_Conn_Para.LL_Conn_ID)
        {
        case (0x40|0x03):
        case (0x40|0x04):
            pLL_Conn->LE_Conn_Para.LL_Conn_ID = 0x04;
            LL_Msg_Event_Encrypt_Change(*pCtrlPkt_Data, i, 0);
            break;
        case (0x40|0x16):
            pLL_Conn->LE_Conn_Para.LL_Conn_ID = 0x04;
            LL_Msg_Event_Le_PHY_Update_Complete(*pCtrlPkt_Data, i, pLL_Conn->LE_Conn_Para.LL_Tx_PHYS);
            break;

        
        
        default:

            if(*(pCtrlPkt_Data-1) == 0x16)
            {
                LL_Msg_Event_Le_PHY_Update_Complete(*pCtrlPkt_Data, i, pLL_Conn->LE_Conn_Para.LL_Tx_PHYS);
            }

            break;
        }
        break;

    case 0x16:

        if(i >= 4)

        {

            mskPhy = ((pCtrlPkt_Data[0]&pCtrlPkt_Data[1])&(~(0|0x04|0xF8)));     
            if(mskPhy)
            {
                mskPhy &= ~(pLL_Conn->LE_Conn_Para.LL_Tx_PHYS);
                mskPhy &= LL_Ref_Default_PHY.Tx_PHYs;
            }
            if(mskPhy==0)
            {
                mskPhy = pLL_Conn->LE_Conn_Para.LL_Tx_PHYS;
            }
            
            *pCtrlPkt_Data = mskPhy;
            *(pCtrlPkt_Data+1) = mskPhy;                                    
            LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x17, 2, i);
            break;

        }

        else
        {
            switch(pLL_Conn->LE_Conn_Para.LL_Conn_ID)
            {
            case (0x40|0x00):
            case (0x40|0x01):
                *pCtrlPkt_Data = 0x16;
                *(pCtrlPkt_Data+1) = 0x2A;
                LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x11, 2, i);
                i = 0xFF;
                break;

            case (0x40|0x16):
                *pCtrlPkt_Data = 0x16;
                *(pCtrlPkt_Data+1) = 0x23;
                LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x11, 2, i);
                i = 0xFF;
                break;

            default:
                break;
            }
            if(i == 0xFF)
            {
                break;
            }
        }

    case 0x17:
        
        mskPhy = ((pCtrlPkt_Data[0]&pCtrlPkt_Data[1])&(~(0|0x04|0xF8)));     
        if(mskPhy)
        {
            if(pLL_Conn->LE_Conn_Para.LL_Tx_PHYsUpd)                        
            {
                mskPhy &= pLL_Conn->LE_Conn_Para.LL_Tx_PHYsUpd;
            }
            mskPhy &= LL_Ref_Default_PHY.Tx_PHYs;
            mskPhy &= ~pLL_Conn->LE_Conn_Para.LL_Tx_PHYS;
            if(mskPhy)
            {
                if(mskPhy!=pLL_Conn->LE_Conn_Para.LL_Tx_PHYS)
                {
                    if(mskPhy&0x02)
                    {
                        mskPhy = 0x02;
                    }
                    else if(mskPhy&0x01)
                    {
                        mskPhy = 0x01;
                    }
                    else
                    {
                        mskPhy = 0;     
                    }
                }
                else
                {
                    mskPhy = 0;
                }
            }
        }
        if(mskPhy)
        {
            *pCtrlPkt_Data = mskPhy;
            *(pCtrlPkt_Data+1) = mskPhy;                                    
            pLL_Conn->LE_Conn_Para.LL_EventCounterUpd = pLL_Conn->LE_Conn_Para.LL_EventCounter+8;
            Knl_MemCpy((pCtrlPkt_Data+2), (Uint8 *)&pLL_Conn->LE_Conn_Para.LL_EventCounterUpd, 2);
        }
        else
        {
            Knl_MemCpy((pCtrlPkt_Data), (Uint8 *)TAB_ZERO_128, 4);
        }
        LL_Msg_CTRL_PKT(pCtrlPkt_Data, 0x18, 4, i);
        pLL_Conn->LE_Conn_Para.LL_Tx_PHYsUpd = mskPhy;
        pLL_Conn->LE_Conn_Para.LL_Rx_PHYsUpd = mskPhy;
        break;

    case 0x18:
#line 3577 "..\\..\\..\\source\\LL.c"
        break;

    case 0x14:
        
        LL_Msg_CTRL_PKT((Uint8 *)LL_Length_Prefer[i], 0x15, sizeof(LL_LENGTH), i);
    case 0x15:
        for(mskPhy=0; mskPhy<(sizeof(LL_LENGTH)/sizeof(LL_LENGTH[0])); mskPhy++)       
        {
            if(*(pCtrlPkt_Data16+mskPhy) > (LL_LENGTH_DEFAULT[mskPhy]))
            {
                *(pCtrlPkt_Data16+mskPhy) = (LL_LENGTH_DEFAULT[mskPhy]);
            }
            if(*(pCtrlPkt_Data16+mskPhy) > LL_Length_Prefer[i][mskPhy])
            {
                *(pCtrlPkt_Data16+mskPhy) = LL_Length_Prefer[i][mskPhy];
            }
        }
        Length16 = *(pCtrlPkt_Data16+1);
        Length16 = (Length16>>2)-(10+4);        
        if(Length16 < *(pCtrlPkt_Data16+0))
        {
            pLL_Conn->LE_Conn_Para.LL_Rx_length_2M = Length16;
        }
        else
        {
            pLL_Conn->LE_Conn_Para.LL_Rx_length_2M = *(pCtrlPkt_Data16+0);
        }

        Length16 = (Length16>>1)-((10+4)/2);    
        if(Length16 < *(pCtrlPkt_Data16+0))
        {
            pLL_Conn->LE_Conn_Para.LL_Rx_length_1M = Length16;
        }
        else
        {
            pLL_Conn->LE_Conn_Para.LL_Rx_length_1M = *(pCtrlPkt_Data16+0);
        }

        Length16 = *(pCtrlPkt_Data16+3);
        Length16 = (Length16>>2)-(10+4);        
        if(Length16 < *(pCtrlPkt_Data16+2))
        {
            pLL_Conn->LE_Conn_Para.LL_Tx_length_2M = Length16;
        }
        else
        {
            pLL_Conn->LE_Conn_Para.LL_Tx_length_2M = *(pCtrlPkt_Data16+2);
        }

        Length16 = (Length16>>1)-((10+4)/2);    
        if(Length16 < *(pCtrlPkt_Data16+2))
        {
            pLL_Conn->LE_Conn_Para.LL_Tx_length_1M = Length16;
        }
        else
        {
            pLL_Conn->LE_Conn_Para.LL_Tx_length_1M = *(pCtrlPkt_Data16+2);
        }
        LL_Msg_Event_Le_Data_Length_Change(i, pLL_Conn->LE_Conn_Para.LL_Tx_length_1M, pLL_Conn->LE_Conn_Para.LL_Rx_length_1M);
        break;

    default:
        LL_Msg_CTRL_PKT(&pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode, 0x07, 1, i);
        break;
    }
}


void HCLL_LE_Ctrl_Pkt(MBLK *pMBlk)
{
    MBLK *mblk;
    MQUEUE *pqueue;
    LL_Conn *pLL_Conn;
    Uint8 i, connID;
    extern MBLK *MBlk_Free;

    mblk = GetMsgBlk();
    Knl_MemCpy(mblk->Para.Data, pMBlk->Para.Data, (pMBlk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_DataPkt_LthL+2+1));
    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = pMBlk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_DataPkt_LthL;


    if(pMBlk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_DataPkt_LthL<=7)
    {
        mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Length_CCM = 0;
    }

    mblk->Primitive = (0xB0+0x3F);
    i = pMBlk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode;
    connID = pMBlk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_Conn_Hdl_L;
    pLL_Conn = &LL_Conn_Para[connID];
    if(i == 0x02)
    {
        pLL_Conn->LE_Conn_Para.LL_Conn_ID = (0x40|0x02);
        pLL_Conn->LE_Conn_Para.LL_PrToutAccu = 0;
    }
    if(LL_CheckConnExist(connID))
    {
        pqueue = &LL_ConnCtrlQ[connID];
        __disable_irq();
        if (pqueue->QIn == (MBLK *)0)
        {
            pqueue->QOut = mblk;
            pqueue->QIn = mblk;
            __enable_irq();
        }
        else
        {
            
            switch(i)
            {
            case 0x02:
                
                mblk->Next = pqueue->QOut;
                pqueue->QOut = mblk;
                __enable_irq();
                break;

            case 0x03:
            case 0x04:
            case 0x05:
            case 0x06:
            case 0x0A:
            case 0x0B:
            case 0x0D:
            case 0x11:
                if(pqueue->QOut->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode >= mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode)
                {
                    
                    mblk->Next = pqueue->QOut;
                    pqueue->QOut = mblk;
                }
                else
                {
                    
                    if (pqueue->QOut == pqueue->QIn)
                    {
                        pqueue->QIn = mblk;
                    }
                    mblk->Next = (pqueue->QOut)->Next;
                    (pqueue->QOut)->Next = mblk;
                }
                __enable_irq();
                break;

            default:
                
                (pqueue->QIn)->Next = mblk;
                pqueue->QIn = mblk;
                __enable_irq();
                break;
            }
        }
    }
}


void MLL_LL_CHN_Map_Upd(MBLK *pMBlk)
{
    extern void LL_RfChRemap(LL_Conn *pLL_Conn);

    LL_RfChRemap(&LL_Conn_Para[pMBlk->Para.MLL_LL_CHN_Map_Upd_Para.LL_Conn_No]);
}


void MLL_LL_Enc_Req(MBLK *pMBlk)
{
    Uint8 i;

    i = pMBlk->Para.MLL_LL_Enc_Req_Para.LL_Conn_No;
    LL_Msg_CTRL_PKT(mblk_LL_conn_Para[i]->Para.MLL_LE_Conn_Para_SMP.LL_Random_Num, 0x03, (8+2+8+4), i);
}


void (* const LL_fsm[])(MBLK *) =
{
    MLL_HCI_Null,
      HCLL_Disconnect,
    HCLL_Read_Remote_Ver_Info,
    HCLL_Set_Event_Mask,
    HCLL_Reset,
    HCLL_Read_Transmit_Pwr_Level,
    HCLL_Set_Ctrler_to_Host_Flow_Ctrl,
    HCLL_Host_Buffer_Size,
    HCLL_Host_Num_of_Completed_Packets,
    HCLL_Set_Event_Mask_Page_2,
    HCLL_Read_Le_Host_Support,
    HCLL_Write_Le_Host_Support,
    HCLL_Read_authen_payload_timeout,
    HCLL_Write_authen_payload_timeout,
    HCLL_Read_Local_Ver_Info,
    HCLL_Read_Local_Supported_Cmd,
      HCLL_Read_Local_Supported_Feat,
    HCLL_Read_buffer_size,
    HCLL_Read_BD_Addr,
    HCLL_Read_rssi,
    HCLL_LE_Set_Event_Mask,
    HCLL_LE_Read_Buffer_Size,
    HCLL_LE_Read_Local_Supported_Feat,
    HCLL_LE_Set_Random_Address,
    HCLL_LE_Set_Adv_Param,
    HCLL_LE_Read_Adv_Ch_Tx_Pwr,
    HCLL_LE_Set_Adv_Data,
    HCLL_LE_Set_Scan_Response_Data,
    HCLL_LE_Set_Advertise_Enable,
    HCLL_LE_Set_scan_param,
    HCLL_LE_Set_scan_enable,
    HCLL_LE_Create_Conn,
      HCLL_LE_Create_conn_cancel,
    HCLL_LE_Read_White_List_Size,
    HCLL_LE_Clear_White_List,
    HCLL_LE_Add_Device_to_White_List,
    HCLL_LE_Rmv_Device_from_White_List,
    HCLL_LE_Conn_update,
    HCLL_LE_Set_Host_Ch_Classification,
    HCLL_LE_Read_ch_map,
    HCLL_LE_Read_Remote_Used_Feat,
    HCLL_LE_Encrypt,
    HCLL_LE_Rand,
    HCLL_LE_Start_encryption,
    HCLL_LE_Long_term_key_req_reply,
    HCLL_LE_Long_term_key_req_neg_reply,
    HCLL_LE_Read_supported_states,
    HCLL_LE_Receiver_test,
      HCLL_LE_Transmitter_test,
    HCLL_LE_Test_end,
    HCLL_LE_Remote_conn_param_req_reply,
    HCLL_LE_Remote_conn_param_req_neg_reply,
    LLHC_ERR_CODE_Unknown_Hci_Command,
    LLHC_ERR_CODE_Invalid_Lmp_Parameters,
    HCLL_LE_ACL_Data_Pkt,
    LLHC_LE_Ctrl_Pkt,
    HCLL_LE_Ctrl_Pkt,
    MLL_LL_CHN_Map_Upd,
    MLL_LL_Enc_Req,
      HCLL_LE_Read_Phy,
      HCLL_LE_Set_Default_Phy,
      HCLL_LE_Set_Phy,
    HCLL_LE_Set_Data_Length,
    MLL_LE_Ctrl_Pkt,
    HCLL_LE_ACL_Data_Pkt_Extend,
};


void LL_Root(Uint8 QueueId)
{
    Uint8 event;
    MBLK *pmblk;

    pmblk = RcvMsgBlk(QueueId);

    event = pmblk->Primitive - 0xB0;
    (*(LL_fsm[event]))(pmblk);
    FreeMsgBlk(pmblk);
}


#pragma Otime
void LL_Msg_CTRL_PKT_Isr(Uint8 *pParam, Uint8 OpcodeCtrl, Uint8 Len_Param, Uint8 ConnID)
{
    MBLK *mblk;

    mblk = GetMsgBlk_Isr();
    mblk->Primitive = (0xB0+0x38);
    mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_Conn_Hdl_L = ConnID;
    mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode = OpcodeCtrl;
    mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_DataPkt_LthL = Len_Param+1;

    if(Len_Param)
    {
        Knl_MemCpy_Isr(mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data, pParam, Len_Param);
    }
    SndMsgBlk_Isr(mblk, 0);
}


void MEC_HCI_Null(MBLK *pMBlk)
{
    __nop();
}


void LLEC_LE_Encrypt(MBLK *pMBlk)                         
{
    MBLK *mblk;

    smpAES_inInv(pMBlk->Para.HCLL_LE_Encrypt_Para.HCI_Key, pMBlk->Para.HCLL_LE_Encrypt_Para.HCI_Plain_Data);
    mblk = GetMsgBlk_Isr();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x29);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    smpAES_outInv(&(mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1]));

    SndMsgBlk_Isr(mblk, 4);

}


void LLEC_LE_Rand(MBLK *pMBlk)                            
{
    MBLK *mblk;

    Knl_CodeCpy_Isr(pMBlk->Para.HCLL_LE_Rand_Para.HCI_ResvMem, &RAND_INI32[(((Uint8 *) &seedR16)[0]&0x0F)], 16);
    smpAES_inInv((Uint8 *)&seedR16, pMBlk->Para.HCLL_LE_Rand_Para.HCI_ResvMem);

    mblk = GetMsgBlk_Isr();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x2A);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    smpAES_out(&mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1]);

    SndMsgBlk_Isr(mblk, 4);

}


void LLEC_LE_Start_encryption(MBLK *pMBlk)                
{
    Uint8 i;
    Uint8 *pi;
    MBLK *mblk;

    pi = pMBlk->Para.HCLL_LE_Start_Encryption_Para.HCI_LongTermKey;
    mblk = GetMsgBlk_Isr();
    Knl_MemCpy_Isr(mblk->Para.MLL_LE_Conn_Para_SMP.LL_SMP_Key, pi, 16);
    LL_smp_SKD_IV_genIn(pi);
    i = pMBlk->Para.HCLL_LE_Start_Encryption_Para.HCI_Conn_Hdl_L;
    mblk_LL_conn_Para[i] = mblk;
    mblk = GetMsgBlk_Isr();
    mblk->Primitive = (0x60+0x02);
    mblk->Para.MHC_Cmd_Status_Para.HCI_Status = 0x00;




    mblk->Para.MHC_Cmd_Status_Para.By_Primitive = (0xB0+0x2B);


    SndMsgBlk_Isr(mblk, 4);


    smpAES_out(pi);
    Knl_MemCpy_Isr(mblk_LL_conn_Para[i]->Para.MLL_LE_Conn_Para_SMP.LL_Random_Num, pMBlk->Para.HCLL_LE_Start_Encryption_Para.HCI_Random_Num, (8+2+8+4));

    if((LL_Conn_Para[i].LE_Conn_Para.LL_SMP_DataCh&(0x01|0x02)) == 0)
    {
        LL_Msg_CTRL_PKT_Isr(pMBlk->Para.HCLL_LE_Start_Encryption_Para.HCI_Random_Num, 0x03, (8+2+8+4), i);
    }
    else
    {
        LL_Msg_CTRL_PKT_Isr(0, 0x0A, 0, i);
    }
}


void LLEC_LE_Long_term_key_req_reply(MBLK *pMBlk)         
{
    MBLK *mblk;
    Uint8 i;
    LL_Conn *pLL_Conn;

    i = pMBlk->Para.HCLL_LE_Long_Term_key_Req_Reply_Para.HCI_Conn_Hdl_L;
    pLL_Conn = &LL_Conn_Para[i];
    smpAES_inInv(pMBlk->Para.HCLL_LE_Long_Term_key_Req_Reply_Para.HCI_LongTermKey, pLL_Conn->LE_Conn_Para.LL_SMP_Key);

    mblk = GetMsgBlk_Isr();
    mblk->Primitive = (0x60+0x01);





    mblk->Para.MHC_Cmd_Complete_Para.By_Primitive = (0xB0+0x2C);

    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0] = 0x00;
    mblk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1] = i;




    SndMsgBlk_Isr(mblk, 4);

    LL_Msg_CTRL_PKT_Isr(0, 0x05, 0, i);

    smpAES_out(pLL_Conn->LE_Conn_Para.LL_SMP_Key);

    LL_SMP_DataCh = 0x02;
    pLL_Conn->LE_Conn_Para.LL_SMP_DataCh = LL_SMP_DataCh;

    pLL_Conn->LE_Conn_Para.LL_SMP_packetCounterT = 0;
    pLL_Conn->LE_Conn_Para.LL_SMP_packetCounterR = 0;
    pLL_Conn->LE_Conn_Para.LL_SMP_packetCounterTd = 0;
    pLL_Conn->LE_Conn_Para.LL_SMP_packetCounterRd = 0;
}


void ECHC_LE_Ctrl_Pkt(MBLK *pMBlk)
{
    MBLK *mblk;
    Uint8 i;
    LL_Conn *pLL_Conn;
    Uint8 *pCtrlPkt_Data;

    pCtrlPkt_Data = pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data;
    i = pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_Conn_Hdl_L;
    pLL_Conn = &LL_Conn_Para[i];
    switch(pMBlk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode)
    {
    case 0x03:
        LL_smp_SKD_IV_genIn((pCtrlPkt_Data+(8+2+8+4)));

        Knl_MemCpy_Isr(pLL_Conn->LE_Conn_Para.LL_SMP_Key, (pCtrlPkt_Data+(8+2)), 8);
        Knl_MemCpy_Isr(pLL_Conn->LE_Conn_Para.LL_SMP_IV, (pCtrlPkt_Data+(8+2+8)), 4);

        mblk = GetMsgBlk_Isr();
        mblk->Primitive = (0x60+0x0E);
        mblk->Para.MHC_Le_Long_Term_Key_Req_Para.HCI_Conn_Hdl_L = i;
        Knl_MemCpy_Isr(mblk->Para.MHC_Le_Long_Term_Key_Req_Para.HCI_Random_Num, pCtrlPkt_Data, (2+8));
        SndMsgBlk_Isr(mblk, 4);

        smpAES_out(pCtrlPkt_Data);
        Knl_MemCpy_Isr(&pLL_Conn->LE_Conn_Para.LL_SMP_Key[8], pCtrlPkt_Data, 8);
        Knl_MemCpy_Isr(&pLL_Conn->LE_Conn_Para.LL_SMP_IV[4], (pCtrlPkt_Data+8), 4);

        LL_Msg_CTRL_PKT_Isr(pCtrlPkt_Data, 0x04, (8+4), i);
        break;

    case 0x04:
        mblk = mblk_LL_conn_Para[i];

        Knl_MemCpy_Isr((mblk->Para.MLL_LE_Conn_Para_SMP.LL_SMP_IV+4), (pCtrlPkt_Data+8), 4);
        Knl_MemCpy_Isr(pLL_Conn->LE_Conn_Para.LL_SMP_IV, mblk->Para.MLL_LE_Conn_Para_SMP.LL_SMP_IV, (4+4));

        Knl_MemCpy_Isr(mblk->Para.MLL_LE_Conn_Para_SMP.LL_SMP_IV, pCtrlPkt_Data, 8);
        smpAES_inInv(mblk->Para.MLL_LE_Conn_Para_SMP.LL_SMP_Key, mblk->Para.MLL_LE_Conn_Para_SMP.LL_SMP_SKD);

        mblk->Primitive = (0xB0+0x00);
        mblk_LL_conn_Para[i] = (MBLK *)0;
        SndMsgBlk_Isr(mblk, 0);

        smpAES_out(pLL_Conn->LE_Conn_Para.LL_SMP_Key);
        break;

    default:
        break;
    }
}



void LLEC_LE_CCM_Manual(MBLK *pMBlk)
{
    MBLK *mblk;

    Uint8 i;
    Uint8 length;
    Uint8 b[16];
    Uint8 y[16];
    Uint8 ctr[16];

    mblk = (MBLK *)pMBlk->Para.MLL_LE_CCM_Manual_Para.mblk_LL_ConnDataQ;
    i = pMBlk->Para.MLL_LE_CCM_Manual_Para.HCI_Conn_Hdl_L;
    smpAES_in_key(LL_Conn_Para[i].LE_Conn_Para.LL_SMP_Key);

    Knl_MemCpy_Isr((Uint8 *)&mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterT, (Uint8 *)&LL_Conn_Para[i].LE_Conn_Para.LL_SMP_packetCounterT,5);
    Knl_MemCpy_Isr(mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_IV, LL_Conn_Para[i].LE_Conn_Para.LL_SMP_IV,8);

    if(i<4)         
    {
        mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterTd |= 0x80;
    }

    length = mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_LthL;

    
    b[0] = 0x49;                        
    b[14] = 0;                          
    b[15] = length;                     
    Knl_MemCpy_Isr(&b[1], (Uint8 *)&mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterT, 13);

     
    for(i=0; i<16; i++)
    {
        y[i] = TAB_ZERO_128[i] ^ b[i];
    }
    smpAES_in_PlainText(y);
    smpAES_En();

    
    Knl_CodeCpy_Isr(b, TAB_ZERO_128, 16);
    
    b[1] = 1;                                                                           
    b[2] = pMBlk->Para.MLL_LE_CCM_Manual_Para.HCI_Conn_Hdl_H;                           

    smpAES_out(y);
     
    for(i=0; i<16; i++)
    {
        y[i] ^= b[i];
    }
    smpAES_in_PlainText(y);
    smpAES_En();

    
    ctr[0] = 1;                                 
    ctr[14] = 0;                                
    ctr[15] = 1;                                
    Knl_MemCpy_Isr(&ctr[1], (Uint8 *)&mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterT, 13);

     
    Knl_CodeCpy_Isr(b, TAB_ZERO_128, 16);
    Knl_MemCpy_Isr(b, mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data, length);

    smpAES_out(y);                              

                 
    for(i=0; i<16; i++)
    {
        y[i] ^= b[i];
    }
    smpAES_in_PlainText(y);
    smpAES_En();
    smpAES_out(y);                              

     
    smpAES_in_PlainText(ctr);
    smpAES_En();
    smpAES_out(b);                              


    
    for(i=0; i<7; i++)
    {
        mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM[i] = mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data[i] ^ b[i];                 
    }

    
    i = 7;
    mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM[i] = (mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterT | 0xFF) ^ b[i];
    mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM[i+1] = ((mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterT >> 8)| 0xFF) ^ b[i+1];
    mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM[i+2] = ((mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterT >> 16)| 0xFF) ^ b[i+2];
    mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM[i+3] = ((mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterT >> 24)| 0xFF) ^ b[i+3];

    
    i = 11;
    mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM[i] = mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterTd ^ b[i];

    
    for(i=0; i<4; i++)
    {
        mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM[i+12] = mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_IV[i] ^ b[i+12];
    }

    
    ctr[14] = 0;
    ctr[15] = 0;

     
    smpAES_in_PlainText(ctr);
    smpAES_En();
    smpAES_out(b);                              
    for(i=0; i<4; i++)                                                                      
    {
        y[i] ^= b[i];
    }

    Knl_MemCpy_Isr(&mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM[length], y, 4); 

    mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Length_CCM = length+4;                   
    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = length;
}



extern void SMEC_LE_CODE_SMP_Pairing_Confirm(MBLK *pMBlk);
extern void SMEC_LE_CODE_SMP_Pairing_Random(MBLK *pMBlk);
extern void SMEC_LE_SMP_Key_Gen(MBLK *pMBlk);
void (* const EC_fsm[])(MBLK *) =
{
    MEC_HCI_Null,
    LLEC_LE_Encrypt,
    LLEC_LE_Rand,
    LLEC_LE_Start_encryption,
    LLEC_LE_Long_term_key_req_reply,
    ECHC_LE_Ctrl_Pkt,
    SMEC_LE_CODE_SMP_Pairing_Confirm,
    SMEC_LE_CODE_SMP_Pairing_Random,
    SMEC_LE_SMP_Key_Gen,

    LLEC_LE_CCM_Manual,

};


void EC_Root_Isr(void)
{
    Uint8 event;
    MBLK *pmblk;

    pmblk = MsgQueueEC.QOut;
    if (MsgQueueEC.QOut == MsgQueueEC.QIn)
    {
        MsgQueueEC.QOut = (MBLK *)0;
        MsgQueueEC.QIn = (MBLK *)0;
    }
    else
    {
        MsgQueueEC.QOut = pmblk->Next;
    }
    

    event = pmblk->Primitive - 0x50;
    (*(EC_fsm[event]))(pmblk);
    FreeMsgBlk_Isr(pmblk);
}


void Ble_EC_Chk_Isr(void)
{
    if(MsgQueueEC.QOut != (MBLK *)0)
    {
        EC_Root_Isr();
    }
}


#pragma Ospace
void initLL(void)
{
    Uint8 i;

    for(i=0; i<4+1; i++)
    {
        LL_ConnDataQ[i].QOut = (MBLK *)0;                           
        LL_ConnDataQ[i].QIn = (MBLK *)0;                            
        LL_ConnCtrlQ[i].QOut = (MBLK *)0;                           
        LL_ConnCtrlQ[i].QIn = (MBLK *)0;                            
        LL_ConnDataInQ[i].QOut = (MBLK *)0;                         
        LL_ConnDataInQ[i].QIn = (MBLK *)0;                          
        LL_ConnBuffPt[i] = (MBLK *)0;                               
        mblk_LL_conn_Para[i] = (MBLK *)0;                           
        LL_ReleaseConnID(i);                                        
    }
    LL_ConnID_Remaining = 4+1;                         
    LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID = 0;                 
    LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_AdvMap_ID = 0;          
    LL_Adv_Para.Adv_Para.LL_AdvMap_ID = 0;                          
    LL_Adv_Para.Adv_Para.LL_AdvConn_ID = 4+1;      
    LL_Adv_Para.Adv_Para.LL_ScanRsp_Data_Length = 0;                
    LL_Adv_Para.Adv_Para.LL_Adv_Data_Length = 6;          
    LL_Adv_Para_UpdBuf.Adv_Para.LL_AdvMap_ID = 0;                       
    LL_Adv_Para_UpdBuf.Adv_Para.LL_AdvConn_ID = 4+1;   
    LL_Adv_Para_UpdBuf.Adv_Para.LL_ScanRsp_Data_Length = 0;             
    LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Data_Length = 6;       
    LL_Init_Para.LE_Init_Para.LL_Conn_ID = 0xFF;          
    LL_Init_Para.LE_Init_Para.LL_AdvMap_ID = 0;                     
    LL_Msg_AdvScnConn = 0;                                          
    LL_Msg_AdvScnConnUpd = 0;                                       
    LL_Msg_AdvScnConnUpdSts = 0;                                    
    LL_Ref_Acs_Addr_ADVSCN = 0x8E89BED6;
    LL_Ref_crc_ini_ADVSCN = 0x55555555;
    LL_ConnDataInTmp.QOut = (MBLK *)0;                              
    LL_ConnDataInTmp.QIn = (MBLK *)0;                               
    LL_ConnDataTmpGate = 0;                                         

    
    LL_Adv_Para.Adv_Para.LL_Tx_PowerLevel = 8;
    LL_Adv_Para_UpdBuf.Adv_Para.LL_Tx_PowerLevel = 8;
    LL_Scan_Para.LE_Set_Scan_Para.LL_Tx_PowerLevel = 8;
    LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_Tx_PowerLevel = 8;
    LL_Init_Para.LE_Init_Para.LL_Tx_PowerLevel = 8;


    for(i=0; i<(2*(4+1+1)); i++)
    {
        TmrBlk_LL[i].Next = i + 1;                      
    }
    TBlk_Free_LL = 0;                                   
    TBlk_InUse_LL = (2*(4+1+1));                     

    status_LL_Tmr = 0x00;                          
    anchor_LL_Tmr = 0;                                  

    LL_SMP_DataCh = 0;                                  


    
    

    Knl_CodeCpy(LL_Ref_ChMap, LL_CHMAP_DEFAUT, 5);                          
    if((ble_device_param.ble_deviceChipId&0x70) == 0x70)
    {
        LL_Ref_Default_PHY.All_PHYs = 0x00;
        LL_Ref_Default_PHY.Tx_PHYs = 0x01;
        LL_Ref_Default_PHY.Rx_PHYs = 0x01;
        LL_Ref_Default_PHY.PHY_options = 0x00;
    }
    else
    {
        Knl_CodeCpy(&LL_Ref_Default_PHY.All_PHYs, LL_DEFAULT_PHY, sizeof(LL_DEFAULT_PHY));  
    }

    

     
    for(i=0; i<4; i++)
    {
        LL_WhiteList_Para[i].LE_WhiteList_Para.AddrType = 0xFC;         
    }
    MsgQueueEC.QIn = (MBLK *)0;                         
    MsgQueueEC.QOut = (MBLK *)0;                        
}


void LL_RfChRemap(LL_Conn *pLL_Conn)
{
    Uint8 i, j, ch_b, ch_idx, ch_re_idx;
    Uint8 * ChMapReM;
    Uint8 * LL_Data_Ch_ReMap;

    ChMapReM = &(pLL_Conn->LE_Conn_Para.LL_ChMapReM[0]);
    LL_Data_Ch_ReMap = &(pLL_Conn->LE_Conn_Para.LL_RF_Data_Ch_ReM[0]);

    ch_idx=0;
    ch_re_idx=0;
    for(i=0; i<5; i++)
    {
        j = *(ChMapReM+i);     
        for(ch_b=0x01; ch_b!=0; ch_b=(ch_b<<1))
        {
            if((j & ch_b) != 0)
            {
                *(LL_Data_Ch_ReMap+ch_re_idx) = ch_idx;
                ch_re_idx++;
            }
            ch_idx++;
        }
    }
    if((37-ch_re_idx))
    {
        Knl_MemCpy((LL_Data_Ch_ReMap+ch_re_idx), LL_Data_Ch_ReMap, (37-ch_re_idx));
        while(ch_re_idx)
        {
            ch_re_idx--;
            *(LL_Data_Ch_ReMap+*(LL_Data_Ch_ReMap+ch_re_idx)) = *(LL_Data_Ch_ReMap+ch_re_idx);
        }
    }
}


#pragma Otime
void LL_smp_SKD_IV_genIn(Uint8 *DataOut)
{
    Knl_CodeCpy_Isr(DataOut, &RAND_INI32[(((Uint8 *) &seedR16)[0]&0x0F)], 16);
    if((((Uint8 *) &seedR16)[0]&0x01))
    {
        
        smpAES_inInv((Uint8 *)(&seedR16), DataOut);
    }
    else
    {
        smpAES_in_key((Uint8 *)(&seedR16));
        smpAES_in_PlainText(DataOut);
        smpAES_En();
    }
}


void LL_ReleaseConnID_Isr(Uint8 LL_Conn_ID)
{
    MQUEUE *pqueue;
    extern MBLK *MBlk_Free;
    LL_Conn *pLL_conn;
    extern Uint8 MBlk_depth_Remaining;
    extern Uint8 LL_ConnID_Remaining;
    MBLK *mblk;
    Uint8 i;

    pLL_conn = &LL_Conn_Para[LL_Conn_ID];

    Knl_CodeCpy_Isr((Uint8 *)LL_Length_Prefer[LL_Conn_ID], (Uint8 *)LL_LENGTH_DEFAULT, sizeof(LL_LENGTH));
    Knl_CodeCpy_Isr(&pLL_conn->LE_Conn_Para.LL_Conn_ID, TAB_ZERO_128, (sizeof(LE_Conn_Para)));    

    pLL_conn->LE_Conn_Para.LL_Conn_ID = 0x01;
    pLL_conn->LE_Conn_Para.LL_Tx_PHYS = 0x01;                                  
    pLL_conn->LE_Conn_Para.LL_Rx_PHYS = 0x01;                                  
    pLL_conn->LE_Conn_Para.LL_Rx_length_1M = 27;                         
    pLL_conn->LE_Conn_Para.LL_Rx_length_2M = 27;                         
    pLL_conn->LE_Conn_Para.LL_Tx_length_1M = 27;                         
    pLL_conn->LE_Conn_Para.LL_Tx_length_2M = 27;                         
    LL_ConnID_Remaining++;

    if(LL_ConnBuffPt[LL_Conn_ID]!=(MBLK *)0)
    {
        mblk = LL_ConnBuffPt[LL_Conn_ID];
        while(1)
        {
            if(mblk!=(MBLK *)0)
            {
                MBlk_depth_Remaining++;
                if(mblk->Next!=(MBLK *)0)
                    mblk = mblk->Next;
                else
                    break;
            }
            else
                break;
        }

        if(mblk!=(MBLK *)0)
        {
            mblk->Next = MBlk_Free;
            MBlk_Free = LL_ConnBuffPt[LL_Conn_ID];
            LL_ConnBuffPt[LL_Conn_ID] = (MBLK *)0;
        }
    }
    
    for(i=0; i<3; i++)
    {
        switch(i)
        {
        case 0:
            pqueue = &LL_ConnDataQ[LL_Conn_ID];
            break;

        case 1:
            pqueue = &LL_ConnCtrlQ[LL_Conn_ID];
            break;

        case 2:
            pqueue = &LL_ConnDataInQ[LL_Conn_ID];
            break;

        default:
            break;
        }
        if(pqueue->QIn != (MBLK *)0)
        {
            mblk = pqueue->QOut;
            while(1)
            {
                MBlk_depth_Remaining++;
                if(mblk->Next != (MBLK *)0)
                {
                    mblk = mblk->Next;
                }
                else
                {
                    break;
                }
            }
            (pqueue->QIn)->Next = MBlk_Free;
            MBlk_Free = pqueue->QOut;
            pqueue->QOut = (MBLK *)0;
            pqueue->QIn = (MBLK *)0;
        }
        
    }

}


Uint8 RF_cmpFIFO_BDAddr(Uint8 HeaderSts_Rx, MBLK *pMBlk)
{
    if(LL_Adv_Para.Adv_Para.LL_Own_Addr_Type == 0)
    {
        if((HeaderSts_Rx&0x80))
        {
            return (!0);
        }
        else
        {
            return Knl_MemComp_Isr(ble_device_param.ble_deviceAddr_param.addr, &pMBlk->Para.Data[6], 6);

        }
    }
    else
    {
        if((HeaderSts_Rx&0x80))
        {
            return Knl_MemComp_Isr(BD_Rand_Addr, &pMBlk->Para.Data[6], 6);
        }
        else
        {
            return (!0);
        }
    }
}


Uint8 RF_cmpFIFO_WhiteList(Uint8 HeaderSts_Rx, MBLK *pMBlk)
{
    Uint8 j, status;

    j = 0;
    if((HeaderSts_Rx&0x40))
    {
        j = 0x01;
    }
    for(status=0; status<4; status++)
    {
        if(LL_WhiteList_Para[status].LE_WhiteList_Para.AddrType == j)
        {
            
            if(Knl_MemComp_Isr(pMBlk->Para.Data, &(LL_WhiteList_Para[status].LE_WhiteList_Para.Addr)[0], 6)==0)
            {
                break;
            }
        }
    }
    if(status == 4)
        return (!0);
    else
        return 0;
}


void LL_TmrBlk_Rls(void)
{
    Uint8 i;

    i = tblk_LL_pi->Next;
    tblk_LL_pi->Next = TBlk_Free_LL;
    TBlk_Free_LL = TBlk_InUse_LL;
    TBlk_InUse_LL = i;
    if(i == (2*(4+1+1)))
    {
        tblk_LL_pi = (TBLK_LLx *)0;
    }
    else
    {
        tblk_LL_pi = tblk_LL_pi2;
        i = tblk_LL_pi->Next;
    }
    if(i == (2*(4+1+1)))
    {
        tblk_LL_pi2 = (TBLK_LLx *)0;
    }
    else
    {
        tblk_LL_pi2 = &TmrBlk_LL[i];
    }
}


void LL_TmrBlk_Rls_Pair(void)
{
    Uint8 i;

    i = tblk_LL_pi2->Next;
    tblk_LL_pi2->Next = TBlk_Free_LL;
    TBlk_Free_LL = TBlk_InUse_LL;
    TBlk_InUse_LL = i;

    if(i == (2*(4+1+1)))
    {
        tblk_LL_pi = (TBLK_LLx *)0;
    }
    else
    {
        tblk_LL_pi = &TmrBlk_LL[i];
        tblk_LL_pi->Ticks = tblk_LL_pi->Ticks + tblk_LL_pi2->Ticks;
        i = tblk_LL_pi->Next;
    }
    if(i == (2*(4+1+1)))
    {
        tblk_LL_pi2 = (TBLK_LLx *)0;
    }
    else
    {
        tblk_LL_pi2 = &TmrBlk_LL[i];
    }
}


void LL_TmrBlk_Rls_NxtIntvl(void)
{
    if(TBlk_InUse_LL == (2*(4+1+1)))
    {
        RF_Tmr_Periodic_set_ISR(1600);
    }
    else
    {
        RF_Tmr_Periodic_set_ISR(tblk_LL_pi->Ticks);
    }
}


void LL_TmrBlk_Pt_PairRst(void)
{
    if(TBlk_InUse_LL != (2*(4+1+1)))
    {
        tblk_LL_pi = &TmrBlk_LL[TBlk_InUse_LL];
        if(tblk_LL_pi->Next == (2*(4+1+1)))
        {
            tblk_LL_pi2 = (TBLK_LLx *)0;
        }
        else
        {
            tblk_LL_pi2 = &TmrBlk_LL[tblk_LL_pi->Next];
        }
    }
}


void LL_Tmr_FIFO_ADVrpt(LL_Para_Header HeaderR, MBLK *pMBlk)
{
    MBLK *mblk;
    Uint8 headerSts;

    SPI_PDMA_SetRx_Isr(166U, (uint32_t)&rssi_read_data[0], 3);;  
    mblk = GetMsgBlk_Isr();
    mblk->Primitive = (0x60+0x0B);
    Knl_MemCpy_Isr(mblk->Para.MHC_Le_Adv_Report_Para.HCI_Addr0, pMBlk->Para.Data, 6);
    headerSts = HeaderR.HeaderSts;    
    if(headerSts&0x40)     
    {
        mblk->Para.MHC_Le_Adv_Report_Para.HCI_Addr_Type0 = 0x01;
    }
    else
    {
        mblk->Para.MHC_Le_Adv_Report_Para.HCI_Addr_Type0 = 0x00;
    }
    mblk->Para.MHC_Le_Adv_Report_Para.HCI_Event_Type0 = (headerSts&0x0F);
    if((headerSts&0x0F)==0x01)
    {
        headerSts = 0;
    }
    else
    {
        headerSts = (HeaderR.HeaderLen&0x3F)-6;   
    }
    mblk->Para.MHC_Le_Adv_Report_Para.HCI_Length0 = headerSts;
    if(headerSts != 0)
    {
        Knl_MemCpy_Isr(mblk->Para.MHC_Le_Adv_Report_Para.HCI_Data0, &pMBlk->Para.Data[6], headerSts);
    }
    mblk->Para.MHC_Le_Adv_Report_Para.HCI_Data0[headerSts] = RF_Get_LastRssi();     
    SndMsgBlk_Isr(mblk, 4);
}


void LL_RxEnter_Isr(void)
{
    uint8_t i;

    
    
    i = RFIP_reg_MEM[RFIP_REG_MEM_121] | 0x40;
    SPI_1BYT_SetTx_Isr(121U, i);
    RFIP_reg_MEM[RFIP_REG_MEM_121] = i;

    RF_Msg_RF0INT = 0x01;

    if(ble_device_param.ble_deviceChipId==102u)
    {
        ((uint8_t *)&Tmr37)[4] |= (RFIP_reg_MEM[RFIP_REG_MEM_119] | 0x40);   
        SPI_PDMA_SetTx(115U, (uint32_t)&Tmr37, 5);
    }
    else
    {
        i = RFIP_reg_MEM[RFIP_REG_MEM_119] | 0x40;
        SPI_1BYT_SetTx_Isr(119U, i);
    }
}




void LL_MsgBlk_LL_conn_Para_Rls(Uint8 ConnID)
{
    MBLK *mblk;

    if(mblk_LL_conn_Para[ConnID] != (MBLK *)0)
    {
        mblk = mblk_LL_conn_Para[ConnID];
        mblk->Primitive = (0xB0+0x00);
        SndMsgBlk_Isr(mblk, 0);
        mblk_LL_conn_Para[ConnID] = (MBLK *)0;
    }
}


void LL_TxEnter_Isr(void)
{
    uint8_t i;

    
    
    i = RFIP_reg_MEM[RFIP_REG_MEM_121] & (~0x40);
    SPI_1BYT_SetTx_Isr(121U, i);
    RFIP_reg_MEM[RFIP_REG_MEM_121] = i;

    RF_Msg_RF0INT = 0x02;

    if(ble_device_param.ble_deviceChipId==102u)
    {
        ((uint8_t *)&Tmr37)[4] |= (RFIP_reg_MEM[RFIP_REG_MEM_119] | 0x40);   
        SPI_PDMA_SetTx(115U, (uint32_t)&Tmr37, 5);
    }
    else
    {
        i = RFIP_reg_MEM[RFIP_REG_MEM_119] | 0x40;
        SPI_1BYT_SetTx_Isr(119U, i);
    }
}


Uint8 LL_Slv_Win_Width_Incr(void)
{
    LL_Slv_Win_Width += LL_Slv_Win_Width_Base;
    return ((((Uint8 *) &LL_Slv_Win_Width)[3])+2);      
}
void LL_Slv_Win_Width_Reset(void)
{
    Uint32 i32;

    LL_Slv_Win_Width = 0;
    i32 = LL_RF_PARA_WIN_WIDTH_CNVT[LL_conn_pi->LE_Conn_Para.LL_SCA];
    
    LL_Slv_Win_Width_Base = (LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg)*(i32);
}






void LL_NextConnCh(LL_Conn *LL_conn_pi)
{
    LL_conn_pi->LE_Conn_Para.LL_CurrentCH = LL_RF_DATA_CH[LL_conn_pi->LE_Conn_Para.LL_CurrentCH + LL_conn_pi->LE_Conn_Para.LL_HopIncrement];
}



void LL_Msg_Map_Upd(Uint8 LL_Conn_No)
{
    MBLK *mblk;

    mblk = GetMsgBlk_Isr();
    mblk->Primitive = (0xB0+0x39);
    mblk->Para.MLL_LL_CHN_Map_Upd_Para.LL_Conn_No = LL_Conn_No;
    SndMsgBlk_Isr(mblk, 0);
}


void LL_Msg_Event_Disconn_Complete(Uint8 HCI_Reason, Uint8 HCI_Conn_Hdl_L)
{
    MBLK *mblk;

    mblk = GetMsgBlk_Isr();
    mblk->Para.MHC_Disconn_Complete_Para.HCI_Reason = HCI_Reason;
    mblk->Para.MHC_Disconn_Complete_Para.HCI_Conn_Hdl_L = HCI_Conn_Hdl_L;
    mblk->Primitive = (0x60+0x03);
    SndMsgBlk_Isr(mblk, 4);
}


void LL_EventCounter_Map_Upd(Uint8 LL_Conn_No)
{
    MBLK *mblk;

    if(LL_conn_pi->LE_Conn_Para.LL_EventCounter > LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd)
    {
        switch(LL_conn_pi->LE_Conn_Para.LL_Conn_ID)
        {
        case (0x40|0x01):
        case (0x80|0x01):
            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
            LL_Msg_Map_Upd(LL_Conn_No);
            break;

        case (0x80|0x18):
            if(LL_conn_pi->LE_Conn_Para.LL_Tx_PHYsUpd == 0)
            {
                LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;

            }
        case (0x40|0x18):
            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
            if(LL_conn_pi->LE_Conn_Para.LL_Tx_PHYsUpd)
            {
                LL_conn_pi->LE_Conn_Para.LL_Tx_PHYS = LL_conn_pi->LE_Conn_Para.LL_Tx_PHYsUpd;
                LL_conn_pi->LE_Conn_Para.LL_Rx_PHYS = LL_conn_pi->LE_Conn_Para.LL_Rx_PHYsUpd;
                LL_conn_pi->LE_Conn_Para.LL_Tx_PHYsUpd = 0;
                LL_conn_pi->LE_Conn_Para.LL_Rx_PHYsUpd = 0;
            }
            mblk = GetMsgBlk_Isr();
            mblk->Primitive = (0x60+0x14);
            mblk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_Status = 0x00;
            mblk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_Conn_Hdl_L = LL_Conn_No;
            mblk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_TX_PHY = LL_conn_pi->LE_Conn_Para.LL_Tx_PHYS;
            mblk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_RX_PHY = LL_conn_pi->LE_Conn_Para.LL_Tx_PHYS;
            SndMsgBlk_Isr(mblk, 4);
            break;

        default:
            break;
        }
    }
}


void LL_SvPrToutAccu_Incr(void)
{
    Uint16 i16;

    i16 = LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg;
    if((LL_conn_pi->LE_Conn_Para.LL_Conn_ID&0x40))
    {
        if(LL_PR_CONN_ID_PAIR[(LL_conn_pi->LE_Conn_Para.LL_Conn_ID-0x40)])
        {
            LL_conn_pi->LE_Conn_Para.LL_PrToutAccu += i16;
        }
    }
    LL_conn_pi->LE_Conn_Para.LL_SvToutAccu += i16;
}


Uint8 LL_RxLen2Offset(Uint8 LenData)
{
    Uint8 i;

    if(LL_conn_pi->LE_Conn_Para.LL_Tx_PHYS==0x01)      
    {
        for(i=0; i<sizeof(LL_LEN_OFFSET_1M); i++)
        {
            if(LenData <= LL_LEN_OFFSET_1M[i])
            {
                break;
            }
        }
    }
    else
    {
        for(i=0; i<sizeof(LL_LEN_OFFSET_2M); i++)
        {
            if(LenData <= LL_LEN_OFFSET_2M[i])
            {
                break;
            }
        }
    }
    return (i+2);
}


MBLK* LL_Queue_To_Buffer_Isr(Uint8 LL_Conn_ID)
{
    MQUEUE *pqueue;
    Uint8 len;
    MBLK *mblk;

    len = 0;
    if(LL_ConnBuffPt[LL_Conn_ID] == (MBLK *)0)
    {
        pqueue = &LL_ConnCtrlQ[LL_Conn_ID];
        if(LL_conn_pi->LE_Conn_Para.LL_SMP_Gate != 0)
        {
            if((pqueue->QOut!=(MBLK *)0))
            {
                switch(pqueue->QOut->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode)
                {
                case 0x03:
                case 0x04:
                case 0x05:
                case 0x06:
                case 0x0A:
                case 0x0B:
                case 0x0D:
                case 0x11:
                    len = 1;
                    break;

                default:
                    break;
                }
            }
        }
        else
        {
            if((pqueue->QOut!=(MBLK *)0))
            {
                len = 1;
            }
            else
            {
                pqueue = &LL_ConnDataQ[LL_Conn_ID];
                if((pqueue->QOut!=(MBLK *)0))
                {
                    len = 1;
                }
            }
        }
    }
    

    if(len!=0)
    {
        LL_ConnBuffPt[LL_Conn_ID] = pqueue->QOut;
        mblk = pqueue->QOut;
        pqueue->QOut = (pqueue->QOut)->Next;
        while(1)
        {
            if(pqueue->QOut != (MBLK *)0)
            {
                if(pqueue->QOut->Primitive==(0xB0+0x40))
                {
                    mblk = pqueue->QOut;
                    pqueue->QOut = (pqueue->QOut)->Next;
                }
                else
                    break;
            }
            else
                break;
        }
        mblk->Next = (MBLK *)0;
        if(pqueue->QOut == (MBLK *)0)
        {
            pqueue->QIn = (MBLK *)0;
        }

    }

    return LL_ConnBuffPt[LL_Conn_ID];
}


void LLTimer_TmrRefUpd_Isr(void)
{
    extern uint32_t Timeline24;

    Tmr37 += 1*125;
    ((uint8_t *)&Tmr37)[4] &= 0x1F;

    Timeline24 += 1;
    if(((uint8_t *)&Timeline24)[3] > 0xEF)
    {
        
        ((uint8_t *)&Timeline24)[3] &= 0x0F;
    }
}


MBLK* LL_Buffer_To_FIFO_Isr(Uint8 LL_Conn_ID, Uint8 HeaderSts)
{
    MBLK *mblk;
    Uint8 i, Len, idxBuf;
    extern uint8_t Content_ioInt;

    mblk = LL_ConnBuffPt[LL_Conn_ID];

    LL_ConnBuffPrcs.BufPrcsF = mblk;
    LL_ConnBuffPrcs.BufPrcsN = mblk;

    
    switch(mblk->Primitive)
    {
    case (0xB0+0x3F):
        i = 0x03;
        
        break;

    case (0xB0+0x36):
        if(mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0==mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL)
        {
            i = 0x02;
        }
        else
        {
            i = 0x01;
        }
        break;

    case (0xB0+0x40):
        i = 0x01;
        break;

    default:
        i = 0x00;
        break;
    }

    Len = LL_conn_pi->LE_Conn_Para.LL_Tx_length_1M;

    if((Len-(Len&0xF0))==1)
    {
        Len -= 1;
    }


    LL_Para_Interval.HeaderSts = ((HeaderSts|i)&(~0xE0));
    LL_Para_Interval.HeaderLen = 0;
    switch(i)
    {
    case 0x03:
        mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_DataPkt_LthL;
        LL_Para_Interval.HeaderLen = mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_DataPkt_LthL;
        Knl_MemCpy_Isr(LL_Para_Interval.Data, mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_DataPkt_LthL);
        break;

    case 0x02:
    case 0x01:
        while(1)
        {
            if(Len+mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 < mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0)
            {
                mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 += Len;
                LL_Para_Interval.HeaderLen += Len;
                break;
            }
            else
            {
                if(Len)
                {
                    Len -= (mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0-mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1);
                    LL_Para_Interval.HeaderLen += (mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0-mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1);
                    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 += (mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0-mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1);
                }
                else
                    break;
            }
            if(mblk->Next != (MBLK *)0)
            {
                LL_ConnBuffPrcs.BufPrcsF = mblk;
                mblk = mblk->Next;
                LL_ConnBuffPrcs.BufPrcsN = mblk;
            }
            else
                break;
        }
        break;

    default:
        break;
    }

    if((LL_Para_Interval.HeaderLen>0x10)&&((LL_Para_Interval.HeaderLen&0x0F)==1))
    {
        if(LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 >= 9)
        {
            LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 -= 9;
        }
        else
        {
            LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 -= (9-LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1);
            LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
        }
        LL_Para_Interval.HeaderLen -= 9;
    }



    Len = 0;
    mblk = LL_ConnBuffPrcs.BufPrcsF;
    while(1)
    {
        if(mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 < mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0)
        {
            Len += (mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 - mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1);
        }
        if(Len > 7)
        {
            Len = 0;
            break;
        }
        else
        {
            if(mblk->Next != (MBLK *)0)
            {
                mblk = mblk->Next;
            }
            else
                break;
        }
    }
    if(Len)
    {
        if(Len <= 7)
        {
            Len = (7+1)-Len;
        }
        if(((LL_Para_Interval.HeaderLen-Len)>0x10)&&(((LL_Para_Interval.HeaderLen-Len)&0x0F)==1))
        {
            Len++;
        }
        if(LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 >= Len)
        {
            LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 -= Len;
        }
        else
        {
            LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 -= (Len-LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1);
            LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
        }
        LL_Para_Interval.HeaderLen -= Len;
    }

    mblk = LL_ConnBuffPt[LL_Conn_ID];
    if(LL_Para_Interval.HeaderLen > 32)
    {
        Len = 32;
    }
    else
    {
        Len = LL_Para_Interval.HeaderLen;
    }

    LL_Para_Interval.DataIdxN = Len;

    idxBuf = 0;
    while(1)
    {
        LL_ConnBuffPrcs.BufPrcsF = mblk;
        if(Len > mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1)
        {
            Knl_MemCpy_Isr(&LL_Para_Interval.Data[idxBuf], &mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL-mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0], mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1);
            Len -= mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1;
            idxBuf += mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1;
            LL_Para_Interval.DataIdxF = mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1;
        }
        else
        {
            Knl_MemCpy_Isr(&LL_Para_Interval.Data[idxBuf], &mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL-mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0], Len);
            idxBuf = mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL-mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0;
            LL_Para_Interval.DataIdxF = idxBuf+Len;
            Len = 0;
            break;
        }
        if(mblk->Next != (MBLK *)0)
        {
            mblk = mblk->Next;
        }
        else
            break;
    }

    LL_ConnBuffPrcs.BufPrcsF = mblk;
    if(mblk->Next != (MBLK *)0)
    {
        if(LL_Para_Interval.DataIdxF == mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL)
        {
            LL_ConnBuffPrcs.BufPrcsF = mblk->Next;
            LL_Para_Interval.DataIdxF = 0;
        }
    }
    if(LL_ConnBuffPrcs.BufPrcsF->Next != (MBLK *)0)
    {
        LL_ConnBuffPrcs.BufPrcsN = LL_ConnBuffPrcs.BufPrcsF->Next;
    }
    else
    {
        LL_ConnBuffPrcs.BufPrcsN = LL_ConnBuffPrcs.BufPrcsF;
    }

    if((LL_SMP_DataCh&0x01))
    {

        if(LL_Conn_ID<4)
        {
            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd |= 0x80;
        }

        SPI_PDMA_SetTx(126U, (uint32_t)&LL_conn_pi->LE_Conn_Para . LL_SMP_packetCounterT, 5);
    }
    RF_CCM_AES_Mode_set((LL_SMP_DataCh&(0x01|0x02)));
    LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd &= (~0x80);


    if((LL_SMP_DataCh&0x01))
    {
        if(LL_conn_pi->LE_Conn_Para.LL_Tx_PHYS==0x02)
        {
            if(LL_Para_Interval.HeaderLen <= 7)
            {
                if((mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Length_CCM==0)||(LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterT != mblk->Para.LLEC_LE_CCM_Manual_Para.LL_SMP_packetCounterT))
                {
                    
                    {
                        
                    }
                    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
                    
                    

                    LL_Para_Interval.HeaderSts = ((HeaderSts|0x01)&(~(0xE0|0x10)));
                    LL_Para_Interval.HeaderLen = 0;
                    RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);

                    mblk = GetMsgBlk_Isr();
                    mblk->Primitive = (0x50+0x09);
                    mblk->Para.MLL_LE_CCM_Manual_Para.mblk_LL_ConnDataQ = (Uint8 *)(LL_ConnBuffPt[LL_Conn_ID]);
                    mblk->Para.MLL_LE_CCM_Manual_Para.HCI_Conn_Hdl_L = LL_Conn_ID;
                    mblk->Para.MLL_LE_CCM_Manual_Para.HCI_Conn_Hdl_H = i;
                    SndMsgBlkEC(mblk);
                    return (MBLK *)0;
                }
                else
                {
                    RF_CCM_AES_Mode_set((LL_SMP_DataCh&0x02));

                    LL_Para_Interval.HeaderLen = mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Length_CCM;
                    LL_Para_Interval.DataIdxN = LL_Para_Interval.HeaderLen;
                    Knl_MemCpy_Isr(LL_Para_Interval.Data, mblk->Para.LLEC_LE_CCM_Manual_Para.HCI_DataPkt_Data_CCM, LL_Para_Interval.HeaderLen);
                }
            }
        }
    }

    Content_ioInt = (SPI_1BYT_SetRx_Isr(62U)&0x40);    
    if(Content_ioInt)
    {
        LL_DurRxPktAccu = LL_DurRxPktAccu - 1;
        tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
        LLTimer_TmrRefUpd_Isr();
        SPI_1BYT_SetTx_Isr(62U, Content_ioInt);                     
    }
    RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);
    if(LL_Para_Interval.HeaderLen)
    {
        RF_TxFIFO_LeData_set(LL_Para_Interval.Data, LL_Para_Interval.DataIdxN);
        LL_ConnDataTmpGate = 1;
    }

    return LL_ConnBuffPt[LL_Conn_ID];
}


MBLK* LLWTR_Buffer_wAck_Isr(Uint8 LL_Conn_ID)
{
    MBLK *mblk;
    Uint8 end;

    end = 0;
    mblk = LL_ConnBuffPt[LL_Conn_ID];
    while(1)
    {
        if(mblk != (MBLK *)0)
        {
            if(mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1)
            {
                if(mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 > mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1)
                {
                    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 -= mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1;
                    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
                    break;
                }
                else
                {
                    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = 0;
                    mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
                    LL_ConnBuffPt[LL_Conn_ID] = mblk->Next;
                }
            }
            else
            {
                mblk = (MBLK *)0;
                break;
            }
            
            if(LL_ConnBuffPt[LL_Conn_ID] != (MBLK *)0)
            {
                FreeMsgBlk_Isr(mblk);
                mblk = LL_ConnBuffPt[LL_Conn_ID];
                if(mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 == 0)
                {
                    break;
                }
            }
            else
            {
                end = 1;
                break;
            }
        }
        else
            break;
    }
    if(end)
    {
        
        {
            
            {
                switch(mblk->Primitive)
                {
                case (0xB0+0x36):
                case (0xB0+0x40):
                    mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_Conn_Hdl_H = (0x02|0x40);
                    break;

                case (0xB0+0x3F):
                    mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_Conn_Hdl_H = 0x03;
                    break;

                default:
                    break;
                }
            }
            








 
        }
        
        
    }

    return mblk;
}


MBLK* LLWTR_Buffer_nAck_Isr(Uint8 LL_Conn_ID)
{
    MBLK *mblk;

    mblk = LL_ConnBuffPt[LL_Conn_ID];
    while(1)
    {
        if(mblk != (MBLK *)0)
        {
            if(mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1)
            {
                mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
                mblk = mblk->Next;
            }
            else
                break;
        }
        else
            break;
    }

    return LL_ConnBuffPt[LL_Conn_ID];
}


void LLTimer_Isr(void)
{
    extern uint32_t Timeline24;
    extern uint32_t Tmr16Interval;

    Uint8 tblk_i, k, idxConnID;
    Uint16 j;
    Uint32 i16, j16;
    
    uint8_t *ptAddr;
    

    LL_Conn *pLL_Conn;
    MBLK *mblk;


    if(TBlk_InUse_LL == (2*(4+1+1)))
    {
        tblk_i = (LL_Msg_AdvScnConnUpdSts^LL_Msg_AdvScnConnUpd);
        if((tblk_i & (0x01|0x02|0x04)))    
        {
            LL_Msg_AdvScnConnUpdSts ^= tblk_i;
            TBlk_InUse_LL = TBlk_Free_LL;

            LL_TmrBlk_Pt_PairRst();

            tblk_LL_pi->TmrId = (0x00 + 1);                    
            tblk_LL_pi->Ticks = LL_Tmr_Ticks_RSV_Wakeup;  

            tblk_LL_pi2->TmrId = (0x00 + 255);                 
            TBlk_Free_LL = tblk_LL_pi2->Next;
            tblk_LL_pi2->Next = (2*(4+1+1));

            LL_Msg_AdvScnConn = (LL_Msg_AdvScnConnUpdSts & (0x01|0x02|0x04))<<4;
            if((LL_Msg_AdvScnConnUpdSts & 0x01))
            {
                Knl_MemCpy_Isr(&LL_Adv_Para.Adv_Para.LL_AdvMap_ID, &LL_Adv_Para_UpdBuf.Adv_Para.LL_AdvMap_ID, sizeof(Adv_Param));
                Knl_MemCpy_Isr(Ch_ADV_Ch_Hop_Table, Ch_ADV_Ch_Hop_TableBuf, 4);
                
                tblk_LL_pi->ConnId = 4+1;
                tblk_LL_pi2->ConnId = 4+1;
                tblk_LL_pi2->Ticks = LL_DUR_RSV[LL_Adv_Para.Adv_Para.LL_Adv_Type&0x0F] + LL_Tmr_Ticks_RSV_Wakeup;
            }
            else if((LL_Msg_AdvScnConnUpdSts & 0x02))
            {
                Knl_MemCpy_Isr(&LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID, &LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_AdvMap_ID, (sizeof(LE_Set_Scan_Para)));
                Knl_MemCpy_Isr(Ch_ADV_Ch_Hop_Table, Ch_ADV_Ch_Hop_TableBuf, 4);
                
                tblk_LL_pi->ConnId = (4+1+1);
                tblk_LL_pi2->ConnId = (4+1+1);
                tblk_LL_pi2->Ticks = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_Scan_Window;
            }

            else if((LL_Msg_AdvScnConnUpdSts & 0x04))
            {
                
                tblk_LL_pi->ConnId = (4+1+2);
                tblk_LL_pi2->ConnId = (4+1+2);
                tblk_LL_pi2->Ticks = LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window;
            }

            RF_Tmr_Periodic_set_ISR(LL_Tmr_Ticks_RSV_Wakeup);
        }
        else
        {
            
            Tmr37 += 1600*125;
            ((uint8_t *)&Tmr37)[4] &= 0x1F;
            Timeline24 += 1600;
            if(((uint8_t *)&Timeline24)[3] > 0xEF)
            {
                
                ((uint8_t *)&Timeline24)[3] &= 0x0F;
            }
            Ble_EC_Chk_Isr();

            RF_PowerSaving_En_Isr();

        }
        return;
    }
    else            
    {
        idxConnID = tblk_LL_pi->ConnId;
        i16 = tblk_LL_pi->Ticks;      
        if(i16 == Tmr16Interval)      
        {
            
            anchor_LL_Tmr += ((Uint8 *) &i16)[0];       
            if(tblk_LL_pi->TmrId < (0x00 + 2))           
            {
                if(tblk_LL_pi->TmrId != 0x00)      
                {
                    RF_Tmr_Periodic_set_ISR(LL_Tmr_Ticks_RSV_Wakeup);
                    tblk_i = (LL_Msg_AdvScnConnUpdSts^LL_Msg_AdvScnConnUpd);
                    LL_Msg_AdvScnConnUpdSts ^= tblk_i;
                    if(LL_Msg_AdvScnConnUpdSts & 0x01)
                    {
                        if((tblk_i & 0x20))
                        {
                            Knl_MemCpy_Isr(&LL_Adv_Para.Adv_Para.LL_AdvMap_ID, &LL_Adv_Para_UpdBuf.Adv_Para.LL_AdvMap_ID, sizeof(Adv_Param));
                            Knl_MemCpy_Isr(Ch_ADV_Ch_Hop_Table, Ch_ADV_Ch_Hop_TableBuf, 4);
                        }
                    }
                    if(LL_Msg_AdvScnConnUpdSts & 0x02)
                    {
                        if((tblk_i & 0x40))
                        {
                            Knl_MemCpy_Isr(&LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID, &LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_AdvMap_ID, (sizeof(LE_Set_Scan_Para)));
                            Knl_MemCpy_Isr(Ch_ADV_Ch_Hop_Table, Ch_ADV_Ch_Hop_TableBuf, 4);
                        }
                    }
                    LL_Msg_AdvScnConn = (LL_Msg_AdvScnConnUpdSts & (0x01|0x02|0x04))<<4;
                    LL_DurRxPktAccu = LL_DurRxPktAccu - LL_Tmr_Ticks_RSV_Wakeup;
                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - LL_Tmr_Ticks_RSV_Wakeup;
                    tblk_LL_pi->TmrId = 0x00;
                    tblk_LL_pi->Ticks = LL_Tmr_Ticks_RSV_Wakeup;
                    Tmr37 += (LL_Tmr_Ticks_RSV_Wakeup*125);
                    ((uint8_t *)&Tmr37)[4] &= 0x1F;
                    Timeline24 += LL_Tmr_Ticks_RSV_Wakeup;
                    if(((uint8_t *)&Timeline24)[3] > 0xEF)
                    {
                        
                        ((uint8_t *)&Timeline24)[3] &= 0x0F;
                    }
                }
                else      
                {
                    RF_Tmr_Periodic_set_ISR(2);

                    rafael_reset_phy_fsm_Isr();
                    SPI_1BYT_SetTx_Isr(107U, (RFIP_reg_MEM[RFIP_REG_MEM_107] | (0x80|0x40)));;
                    RF_WTR_intOn();  
                    
                    tblk_i = tblk_LL_pi->ConnId;

                    if(tblk_i<4+1)    
                    {
                        LL_conn_pi = &LL_Conn_Para[tblk_i];
                        RF_RxLengthLimit(255);
                        
                        
                        switch(status_LL_Tmr)
                        {
                        case 0x00:
                            RF_Set_TxPowerLevel_Isr(LL_conn_pi->LE_Conn_Para.LL_Tx_PowerLevel);

                            if(tblk_i < 4)    
                            {
                                LL_TxEnter_Isr();
                                status_LL_Tmr = (0x00 + 40);
                            }
                            else

                            {

                                LL_RxEnter_Isr();            
                                status_LL_Tmr = (0x00 + 50);  





                            }
                            LL_DurRxPktAccu = LL_DurRxPktAccu - 2;
                            tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 2;
                            tblk_LL_pi->TmrId = (0x00 + 3);
                            tblk_LL_pi->Ticks = 2;
                            break;

                        default:
                            ErrorEntry(1);
                            break;
                        }

                        SPI_PDMA_SetTx(122U, (uint32_t)&LL_conn_pi->LE_Conn_Para.LL_AccessAddr, 4);
                        LL_SMP_DataCh = LL_conn_pi->LE_Conn_Para.LL_SMP_DataCh;
                        

                        
                        setChannel_BLE(LL_conn_pi->LE_Conn_Para.LL_RF_Data_Ch_ReM[LL_conn_pi->LE_Conn_Para.LL_CurrentCH]);

                        RF_PLL_VCO_Bank_set(CH_PLL_bank_Table[LL_conn_pi->LE_Conn_Para.LL_RF_Data_Ch_ReM[LL_conn_pi->LE_Conn_Para.LL_CurrentCH]]);

                        LL_NextConnCh(LL_conn_pi);

                        RF_CRCInit(LL_conn_pi->LE_Conn_Para.LL_CRC_Init);

                        if(LL_conn_pi->LE_Conn_Para.LL_Tx_PHYS==0x02)      
                        {
                            RF_SymbolRate_Patch_1M_2M(1);
                        }
                        else
                        {
                            RF_SymbolRate_Patch_1M_2M(0);
                        }
                        RF_TxAutoAckOn();  

                        LL_SvPrToutAccu_Incr();

                        if(idxConnID < 4)
                        {
                            pLL_Conn = &LL_Conn_Para[idxConnID];
                            
                            mblk = LL_ConnBuffPt[idxConnID];

                            j = pLL_Conn->LE_Conn_Para.WinSize_DataHdr;
                            if(j&0x80)
                            {
                                if(mblk == (MBLK *)0)
                                {
                                    mblk = LL_Queue_To_Buffer_Isr(idxConnID);
                                }
                            }
                            j &= (0x08|0x04);
                            if(mblk != (MBLK *)0)
                            {
                                if(mblk->Primitive==(0xB0+0x3F))
                                {
                                    switch(mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode)
                                    {
                                    case 0x00:
                                        if((anchor_LL_Tmr&0x07))
                                        {
                                            LL_conn_pi->LE_Conn_Para.WinOffsetUpd = ((10)-(anchor_LL_Tmr&0x07));
                                        }
                                        else
                                        {
                                            LL_conn_pi->LE_Conn_Para.WinOffsetUpd = 10;
                                        }
                                        mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[1] = LL_conn_pi->LE_Conn_Para.WinOffsetUpd;
                                        mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[2] = LL_conn_pi->LE_Conn_Para.WinOffsetUpd>>8;
                                        i16 = pLL_Conn->LE_Conn_Para.LL_EventCounter+(4+1+3);
                                        if(pLL_Conn->LE_Conn_Para.LL_EventCounterUpd > i16)
                                        {
                                            mblk->Para.HCLL_LE_Ctrl_Pkt_Conn_Update_Para.HCI_CtrlPkt_Instant = i16;
                                            pLL_Conn->LE_Conn_Para.LL_EventCounterUpd = i16;

                                        }
                                        break;

                                    case 0x01:
                                        i16 = pLL_Conn->LE_Conn_Para.LL_EventCounter+(4+1+3);
                                        if(pLL_Conn->LE_Conn_Para.LL_EventCounterUpd > i16)
                                        {
                                            mblk->Para.HCLL_LE_Ctrl_Pkt_Set_Host_Ch_Para.HCI_CtrlPkt_Instant = i16;
                                            pLL_Conn->LE_Conn_Para.LL_EventCounterUpd = i16;
                                        }
                                        break;

                                    default:
                                        break;
                                    }
                                }

                                if((LL_ConnDataQ[idxConnID].QOut!=(MBLK *)0)||(LL_ConnCtrlQ[idxConnID].QOut!=(MBLK *)0))
                                {
                                    j |= 0x10;
                                }
                                LL_Buffer_To_FIFO_Isr(idxConnID, j);
                            }
                            else
                            {
                                LL_Para_Interval.HeaderSts = ((j|0x01)&(~0xE0));
                                LL_Para_Interval.HeaderLen = 0;
                                RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);
                            }
                            pLL_Conn->LE_Conn_Para.WinSize_DataHdr = j;
                        }

                    }
                    else       
                    {
                        RF_CRCInit((uint8_t *)&LL_Ref_crc_ini_ADVSCN);
                        SPI_PDMA_SetTx(122U, (uint32_t)&LL_Ref_Acs_Addr_ADVSCN, 4);
                        SPI_1BYT_SetTx_Isr(155U, (0u));
                        RF_RxLengthLimit((6+31));

                        switch(tblk_i)
                        {
                        case 4+1:   
                            LL_conn_pi = &LL_Conn_Para[LL_Adv_Para.Adv_Para.LL_AdvConn_ID];
                            if((LL_Msg_AdvScnConn & 0x10)==0)     
                            {
                                if(LL_Adv_Para.Adv_Para.LL_AdvConn_ID < 4+1)
                                {
                                    LL_MsgBlk_LL_conn_Para_Rls(LL_Adv_Para.Adv_Para.LL_AdvConn_ID);
                                    LL_Adv_Para.Adv_Para.LL_AdvConn_ID = 4+1;
                                }
                                RF_WTR_intOff();
                                LL_TmrBlk_Rls_Pair();
                                LL_TmrBlk_Rls_NxtIntvl();
                                return;
                            }
                            switch(status_LL_Tmr)
                            {
                            case 0x00:   
                                RF_Set_TxPowerLevel_Isr(LL_Adv_Para.Adv_Para.LL_Tx_PowerLevel);
                                LL_TxEnter_Isr();

                                
                                setChannel_BLE(CH_ADV_SEL_TABLE[LL_Adv_Para.Adv_Para.LL_AdvMap_ID]);

                                RF_PLL_VCO_Bank_set(CH_PLL_bank_Table[CH_ADV_SEL_TABLE[LL_Adv_Para.Adv_Para.LL_AdvMap_ID]]);


                                LL_Adv_Para.Adv_Para.LL_AdvMap_ID = Ch_ADV_Ch_Hop_Table[LL_Adv_Para.Adv_Para.LL_AdvMap_ID];

                                LL_Para_Interval.HeaderSts = LL_Adv_Para.Adv_Para.LL_Adv_Type;
                                tblk_i = LL_Adv_Para.Adv_Para.LL_Adv_Type;
                                tblk_i = tblk_i&0x0F;

                                if(tblk_i == 0x01)
                                {
                                    ptAddr = (uint8_t *)LL_Adv_Para.Adv_Para.LL_DirectAddr;
                                    j = (6+6);
                                }
                                else
                                {
                                    if((LL_Msg_AdvScnConnUpdSts & 0x08))
                                    {
                                        ptAddr = (uint8_t *)LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Data;
                                        j = LL_Adv_Para_UpdBuf.Adv_Para.LL_Adv_Data_Length;
                                    }
                                    else
                                    {
                                        ptAddr = (uint8_t *)LL_Adv_Para.Adv_Para.LL_Adv_Data;
                                        j = LL_Adv_Para.Adv_Para.LL_Adv_Data_Length;
                                    }
                                }
                                LL_Para_Interval.HeaderLen = j;
                                RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);  

                                if(LL_Adv_Para.Adv_Para.LL_Own_Addr_Type == 0x00)
                                {
                                    RF_TxFIFO_ADVaddr_set(ble_device_param.ble_deviceAddr_param.addr);
                                }
                                else
                                {
                                    RF_TxFIFO_ADVaddr_set(BD_Rand_Addr);
                                }

                                RF_SymbolRate_Patch_1M_2M(0);                               
                                if(tblk_i==0x02)
                                {
                                    RF_TxAutoAckOff();     
                                    status_LL_Tmr = (0x00 + 10);
                                }
                                else
                                {
                                    RF_TxAutoAckOn();      
                                    status_LL_Tmr = (0x00 + 11);
                                }
                                RF_TxFIFO_ADVData_set((uint8_t *)ptAddr);

                                tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 2;
                                tblk_LL_pi->TmrId = (0x00 + 3);
                                tblk_LL_pi->Ticks = 2;
                                break;

                            default:
                                ErrorEntry(2);
                                break;
                            }
                            break;

                        case (4+1+1):   
                            if((LL_Msg_AdvScnConn & 0x20)==0)
                            {
                                RF_WTR_intOff();
                                LL_TmrBlk_Rls_Pair();
                                LL_TmrBlk_Rls_NxtIntvl();
                                return;
                            }
                            switch(status_LL_Tmr)
                            {
                            case 0x00:
                                RF_Set_TxPowerLevel_Isr(LL_Scan_Para.LE_Set_Scan_Para.LL_Tx_PowerLevel);
                                LL_RxEnter_Isr();

                                setChannel_BLE(CH_ADV_SEL_TABLE[LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID]);

                                RF_PLL_VCO_Bank_set(CH_PLL_bank_Table[CH_ADV_SEL_TABLE[LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID]]);

                                RF_SymbolRate_Patch_1M_2M(0);                               
                                if(LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanType != 0)
                                {
                                    status_LL_Tmr = (0x00 + 20);   
                                    LL_Para_Interval.HeaderLen = (6+6);
                                    RF_TxAutoAckOn();             
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 2;
                                    tblk_LL_pi->Ticks = 2;
                                    tblk_LL_pi->TmrId = (0x00 + 3);
                                }
                                else       
                                {
                                    status_LL_Tmr = (0x00 + 21);
                                    RF_TxAutoAckOff();
                                    LL_TmrBlk_Rls();
                                }
                                break;

                            default:
                                ErrorEntry(3);
                                break;
                            }
                            break;


                        case (4+1+2):  
                            LL_conn_pi = &LL_Conn_Para[LL_Init_Para.LE_Init_Para.LL_Conn_ID];
                            if((LL_Msg_AdvScnConn & 0x40)==0)
                            {
                                if(LL_Init_Para.LE_Init_Para.LL_Conn_ID < 0xFF)
                                {
                                    LL_MsgBlk_LL_conn_Para_Rls(LL_Init_Para.LE_Init_Para.LL_Conn_ID);
                                }
                                RF_WTR_intOff();
                                LL_TmrBlk_Rls_Pair();
                                LL_TmrBlk_Rls_NxtIntvl();
                                return;
                            }
                            switch(status_LL_Tmr)
                            {
                            case 0x00:
                                RF_Set_TxPowerLevel_Isr(LL_Init_Para.LE_Init_Para.LL_Tx_PowerLevel);
                                LL_RxEnter_Isr();
                                setChannel_BLE(CH_ADV_SEL_TABLE[LL_Init_Para.LE_Init_Para.LL_AdvMap_ID]);

                                RF_PLL_VCO_Bank_set(CH_PLL_bank_Table[CH_ADV_SEL_TABLE[LL_Init_Para.LE_Init_Para.LL_AdvMap_ID]]);

                                status_LL_Tmr = (0x00 + 30);   
                                LL_Para_Interval.HeaderLen = (1*2+2*4+4*1+6*2+3+5);
                                RF_SymbolRate_Patch_1M_2M(0);                               
                                RF_TxAutoAckOn();             
                                tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 2;
                                tblk_LL_pi->Ticks = 2;
                                tblk_LL_pi->TmrId = (0x00 + 3);
                                break;

                            default:
                                ErrorEntry(4);
                                break;
                            }
                            break;


                        default:
                            __nop();    
                            break;
                        }
                    }
                }      
            }      
            else         
            {
                if(tblk_LL_pi->TmrId != (0x00 + 255))    
                {
                    if(tblk_LL_pi->TmrId == (0x00 + 3))
                    {
                        RF_Tmr_Periodic_set_ISR(1);
                        if(ble_device_param.ble_deviceChipId==102u)
                        {
                            ((uint8_t *)&Tmr37)[4] |= (RFIP_reg_MEM[RFIP_REG_MEM_119]);                 
                            SPI_PDMA_SetTx(115U, (uint32_t)&Tmr37, 5);
                        }
                        else
                        {
                            SPI_1BYT_SetTx_Isr(119U, RFIP_reg_MEM[RFIP_REG_MEM_119] & 0x3F);        
                        }
                        tblk_LL_pi->Ticks = 1;
                        tblk_LL_pi->TmrId = (0x00 + 2);
                    }
                    tblk_i = tblk_LL_pi->ConnId;
                    if(tblk_i<4+1)      
                    {

                        if(tblk_i < 4)     
                        {
                            switch(status_LL_Tmr)
                            {
                            case (0x00 + 40):
                            case (0x00 + 43):
                                LL_DurRxPktAccu = LL_DurRxPktAccu - 1;
                                tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                LLTimer_TmrRefUpd_Isr();
                                break;

                            case (0x00 + 41):
                            case (0x00 + 42):
                                if(tblk_LL_pi2->Ticks != 1)
                                {
                                    LL_DurRxPktAccu = LL_DurRxPktAccu - 1;
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                    LLTimer_TmrRefUpd_Isr();
                                    break;
                                }
                                status_LL_Tmr = (0x00 + 44);
                                RF_TxAutoAckOff();  
                                rafael_reset_phy_fsm_Isr();    
                                LL_conn_pi->LE_Conn_Para.WinSize_DataHdr &= (0x08|0x04);
                                LLWTR_Buffer_nAck_Isr(tblk_LL_pi->ConnId);
                            case (0x00 + 44):
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                tblk_i = tblk_LL_pi->Next;
                                j = tblk_LL_pi->ConnId;
                                LL_TmrBlk_Rls();
                                i16 = LL_DurRxPktAccu;
                                k = 0;
                                while(1)
                                {
                                    if((LL_conn_pi->LE_Conn_Para.LL_SvToutAccu >= LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout)                                            ||LL_conn_pi->LE_Conn_Para.LL_PrToutAccu >= 0x7D00)

                                    {
                                        break;       
                                    }
                                    if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID==(0x40|0x00))
                                    {
                                        if(LL_conn_pi->LE_Conn_Para.LL_EventCounter+k==LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd)
                                        {
                                            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x06;
                                            LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg = LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrgUpd;

                                            

                                            i16 += LL_conn_pi->LE_Conn_Para.WinOffsetUpd*10;




                                            Knl_MemCpy_Isr((Uint8 *)&LL_conn_pi->LE_Conn_Para.LL_ConnInterval, (Uint8 *)&LL_conn_pi->LE_Conn_Para.LL_ConnIntervalUpd, 6);
                                        }
                                    }
                                    if(tblk_i < (2*(4+1+1)))
                                    {
                                        tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                        if(tblk_LL_pi2->Ticks>=i16)
                                        {
                                            if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                            {
                                                if(i16)
                                                {
                                                    LL_DurRxPktAccu = 0;
                                                    if(tblk_LL_pi2->Ticks-i16 > ((36+1)+LL_Tmr_Ticks_RSV_Wakeup+2))
                                                    {
                                                        break;
                                                    }
                                                }
                                            }
                                            i16 = i16+LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                            LL_SvPrToutAccu_Incr();
                                            LL_NextConnCh(LL_conn_pi);
                                            k++;
                                        }
                                    }
                                    else
                                    {
                                        if(i16==0)
                                        {
                                            i16 = LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                            LL_SvPrToutAccu_Incr();
                                            LL_NextConnCh(LL_conn_pi);
                                            k++;
                                        }
                                        LL_DurRxPktAccu = 0;
                                        k++;
                                        break;
                                    }
                                    tblk_i = tblk_LL_pi2->Next;
                                    i16 = i16 - tblk_LL_pi2->Ticks;
                                    tblk_LL_pi = tblk_LL_pi2;
                                }      
                                if(LL_conn_pi->LE_Conn_Para.LL_PrToutAccu >= 0x7D00)
                                {
                                    status_LL_Tmr = (0x00 + 45);
                                    LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x22;
                                    break;
                                }
                                if(LL_conn_pi->LE_Conn_Para.LL_SvToutAccu<LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout)
                                {
                                    LL_conn_pi->LE_Conn_Para.LL_EventCounter += k;
                                    tblk_LL_pi->Next = TBlk_Free_LL;
                                    tblk_LL_pi = &TmrBlk_LL[TBlk_Free_LL];
                                    tblk_LL_pi->TmrId = (0x00 + 1);
                                    tblk_LL_pi->Ticks = i16;
                                    tblk_LL_pi2 = &TmrBlk_LL[tblk_LL_pi->Next];
                                    TBlk_Free_LL = tblk_LL_pi2->Next;

                                    tblk_LL_pi2->Next = tblk_i;
                                    tblk_LL_pi2->Ticks = (36+1);
                                    tblk_LL_pi2->TmrId = (0x00 + 255);

                                    tblk_LL_pi->ConnId = j;
                                    tblk_LL_pi2->ConnId = j;
                                    LL_EventCounter_Map_Upd(j);
                                    LL_DurRxPktAccu += LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                }
                                else      
                                {
                                    status_LL_Tmr = (0x00 + 45);
                                    switch(LL_conn_pi->LE_Conn_Para.LL_Conn_ID)
                                    {
                                    case 0x03:
                                        LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x3E;
                                        break;

                                    case (0x40|0x02):
                                        LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x22;
                                        break;

                                    default:
                                        LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x08;
                                        break;
                                    }
                                }
                                LL_TmrBlk_Pt_PairRst();
                                break;

                            case (0x00 + 45):
                                
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                LL_TmrBlk_Rls();
                                LL_TmrBlk_Pt_PairRst();
                                break;

                            default:
                                ErrorEntry(5);
                                break;
                            }
                        }
                        else     

                        {

                            switch(status_LL_Tmr)
                            {
                            case (0x00 + 51):  
                            case (0x00 + 52):  
                                LL_DurRxPktAccu = LL_DurRxPktAccu - 1;
                                tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                LLTimer_TmrRefUpd_Isr();
                                break;

                            case (0x00 + 50):   
                            case (0x00 + 53):   
                                if(tblk_LL_pi2->Ticks == 3)
                                {
                                    RF_TxAutoAckOff();  
                                    rafael_reset_phy_fsm_Isr();
                                    status_LL_Tmr = (0x00 + 54);  
                                }
                                else
                                {
                                    LL_DurRxPktAccu = LL_DurRxPktAccu - 1;
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                    LLTimer_TmrRefUpd_Isr();
                                    break;
                                }
                            case (0x00 + 54):  
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                tblk_i = tblk_LL_pi->Next;
                                LL_TmrBlk_Rls();
                                i16 = LL_DurRxPktAccu;
                                j = LL_Slv_Win_Width_Incr();
                                k = 0;
                                while(1)
                                {
                                    if((LL_conn_pi->LE_Conn_Para.LL_SvToutAccu >= LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout)                                            ||LL_conn_pi->LE_Conn_Para.LL_PrToutAccu >= 0x7D00)

                                    {
                                        break;    
                                    }
                                    if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID==(0x80|0x00))
                                    {
                                        if(LL_conn_pi->LE_Conn_Para.LL_EventCounter+k==LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd)
                                        {
                                            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x06;
                                            LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg = LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrgUpd;
                                            i16 += LL_conn_pi->LE_Conn_Para.WinOffsetUpd;
                                            Knl_MemCpy_Isr((Uint8 *)&LL_conn_pi->LE_Conn_Para.LL_ConnInterval, (Uint8 *)&LL_conn_pi->LE_Conn_Para.LL_ConnIntervalUpd, 6);
                                            if(i16 < LL_conn_pi->LE_Conn_Para.WinSizeUpd)
                                            {
                                                i16 = LL_conn_pi->LE_Conn_Para.WinSizeUpd;
                                            }
                                        }
                                    }
                                    if(tblk_i < (2*(4+1+1)))
                                    {
                                        tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                        if(tblk_LL_pi2->Ticks>=i16)
                                        {
                                            if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                            {
                                                if(i16>(j+(LL_Tmr_Ticks_RSV_Wakeup+2+1)))
                                                {
                                                    j16 = i16-(j+(LL_Tmr_Ticks_RSV_Wakeup+2+1));
                                                    LL_DurRxPktAccu = (j+(LL_Tmr_Ticks_RSV_Wakeup+2+1));
                                                    j = (j<<1)+((36+1)+LL_Tmr_Ticks_RSV_Wakeup+2);
                                                    switch(LL_conn_pi->LE_Conn_Para.LL_Conn_ID)
                                                    {
                                                    case 0x02:
                                                        j += LL_conn_pi->LE_Conn_Para.WinSize_DataHdr;
                                                        break;

                                                    case 0x06:
                                                        
                                                        break;

                                                    default:
                                                        break;
                                                    }
                                                    if(tblk_LL_pi2->Ticks-i16 > j)
                                                    {
                                                        break;
                                                    }
                                                }
                                            }
                                            i16 = i16+LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                            LL_SvPrToutAccu_Incr();
                                            LL_NextConnCh(LL_conn_pi);
                                            j = LL_Slv_Win_Width_Incr();
                                            k++;
                                        }
                                    }
                                    else
                                    {
                                        
                                        if(j+6+LL_Tmr_Ticks_RSV_Wakeup+2 >= i16)
                                        {
                                            j = i16-2-LL_Tmr_Ticks_RSV_Wakeup-6;
                                        }
                                        
                                        if(i16<=(j+(1+LL_Tmr_Ticks_RSV_Wakeup+2)))
                                        {
                                            i16 = i16+LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                            LL_SvPrToutAccu_Incr();
                                            LL_NextConnCh(LL_conn_pi);
                                            j = LL_Slv_Win_Width_Incr();
                                            k++;
                                        }
                                        j16 = i16-(j+(1+LL_Tmr_Ticks_RSV_Wakeup+2));
                                        LL_DurRxPktAccu = (j+(1+LL_Tmr_Ticks_RSV_Wakeup+2));
                                        j = (j<<1)+((36+1)+LL_Tmr_Ticks_RSV_Wakeup+2);
                                        switch(LL_conn_pi->LE_Conn_Para.LL_Conn_ID)
                                        {
                                        case 0x02:
                                            j += LL_conn_pi->LE_Conn_Para.WinSize_DataHdr;
                                            break;

                                        case 0x06:
                                            
                                            break;

                                        default:
                                            break;
                                        }
                                        k++;
                                        break;
                                    }
                                    tblk_i = tblk_LL_pi2->Next;
                                    i16 = i16 - tblk_LL_pi2->Ticks;
                                    tblk_LL_pi = tblk_LL_pi2;
                                }   
                                if(LL_conn_pi->LE_Conn_Para.LL_PrToutAccu >= 0x7D00)
                                {
                                    status_LL_Tmr = (0x00 + 55);
                                    LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x22;
                                    break;
                                }
                                if(LL_conn_pi->LE_Conn_Para.LL_SvToutAccu<LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout)
                                {
                                    LL_conn_pi->LE_Conn_Para.LL_EventCounter += k;
                                    tblk_LL_pi->Next = TBlk_Free_LL;
                                    tblk_LL_pi = &TmrBlk_LL[TBlk_Free_LL];
                                    tblk_LL_pi->TmrId = (0x00 + 1);
                                    tblk_LL_pi->Ticks = j16;
                                    tblk_LL_pi2 = &TmrBlk_LL[tblk_LL_pi->Next];
                                    TBlk_Free_LL = tblk_LL_pi2->Next;

                                    tblk_LL_pi2->Next = tblk_i;
                                    tblk_LL_pi2->Ticks = j;
                                    tblk_LL_pi2->TmrId = (0x00 + 255);

                                    tblk_LL_pi->ConnId = idxConnID;
                                    tblk_LL_pi2->ConnId = idxConnID;
                                    LL_EventCounter_Map_Upd(idxConnID);
                                    LL_DurRxPktAccu += LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                }
                                else      
                                {
                                    status_LL_Tmr = (0x00 + 55);
                                    switch(LL_conn_pi->LE_Conn_Para.LL_Conn_ID)
                                    {
                                    case 0x02:
                                        LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x3E;
                                        break;

                                    case (0x40|0x02):
                                        LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x22;
                                        break;

                                    default:
                                        LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x08;
                                        break;
                                    }
                                }
                                LL_TmrBlk_Pt_PairRst();
                                break;

                            case (0x00 + 55):    
                                
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                LL_TmrBlk_Rls();
                                LL_TmrBlk_Pt_PairRst();
                                break;

                            default:
                                ErrorEntry(6);
                                break;
                            }

                        }
                    }
                    else     
                    {
                        switch(tblk_i)
                        {
                        case 4+1:
                            switch(status_LL_Tmr)
                            {
                            case (0x00 + 14):
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                tblk_i = tblk_LL_pi->Next;
                                LL_TmrBlk_Rls();
                                i16 = LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu+(8-1);
                                LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu = 0;
                                j = LL_Slv_Win_Width_Incr();
                                k = 0;
                                while(1)
                                {
                                    if(LL_conn_pi->LE_Conn_Para.LL_SvToutAccu >= LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout)
                                    {
                                        break;      
                                    }
                                    if(tblk_i < (2*(4+1+1)))
                                    {
                                        tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                        if(tblk_LL_pi2->Ticks>=i16)
                                        {
                                            if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                            {
                                                if(i16>(j+(LL_Tmr_Ticks_RSV_Wakeup+2+1)))
                                                {
                                                    j16 = i16-(j+(LL_Tmr_Ticks_RSV_Wakeup+2+1));
                                                    LL_DurRxPktAccu = (j+(LL_Tmr_Ticks_RSV_Wakeup+2+1));
                                                    j = ((j<<1)+LL_conn_pi->LE_Conn_Para.WinSize_DataHdr)+((36+1)+LL_Tmr_Ticks_RSV_Wakeup+2);
                                                    if(tblk_LL_pi2->Ticks-i16 > j)
                                                    {
                                                        break;
                                                    }
                                                }
                                            }
                                            i16 = i16+LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                            LL_conn_pi->LE_Conn_Para.LL_SvToutAccu += LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg;
                                            LL_NextConnCh(LL_conn_pi);
                                            j = LL_Slv_Win_Width_Incr();
                                            k++;
                                        }
                                    }
                                    else
                                    {
                                        if(i16<=(j+(LL_Tmr_Ticks_RSV_Wakeup+2+1)))
                                        {
                                            i16 = i16+LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                            LL_conn_pi->LE_Conn_Para.LL_SvToutAccu += LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg;
                                            LL_NextConnCh(LL_conn_pi);
                                            j = LL_Slv_Win_Width_Incr();
                                            k++;
                                        }
                                        j16 = i16-(j+(LL_Tmr_Ticks_RSV_Wakeup+2+1));
                                        LL_DurRxPktAccu = (j+(LL_Tmr_Ticks_RSV_Wakeup+2+1));
                                        j = ((j<<1)+LL_conn_pi->LE_Conn_Para.WinSize_DataHdr)+((36+1)+LL_Tmr_Ticks_RSV_Wakeup+2);
                                        k++;
                                        break;
                                    }
                                    tblk_i = tblk_LL_pi2->Next;
                                    i16 = i16 - tblk_LL_pi2->Ticks;
                                    tblk_LL_pi = tblk_LL_pi2;
                                }    
                                if(LL_conn_pi->LE_Conn_Para.LL_SvToutAccu<LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout)
                                {
                                    LL_conn_pi->LE_Conn_Para.LL_EventCounter += k;
                                    tblk_LL_pi->Next = TBlk_Free_LL;
                                    tblk_LL_pi = &TmrBlk_LL[TBlk_Free_LL];
                                    tblk_LL_pi->TmrId = (0x00 + 1);
                                    tblk_LL_pi->Ticks = j16;
                                    tblk_LL_pi2 = &TmrBlk_LL[tblk_LL_pi->Next];
                                    TBlk_Free_LL = tblk_LL_pi2->Next;

                                    tblk_LL_pi2->Next = tblk_i;
                                    tblk_LL_pi2->Ticks = j;
                                    tblk_LL_pi2->TmrId = (0x00 + 255);

                                    tblk_LL_pi->ConnId = LL_Adv_Para.Adv_Para.LL_AdvConn_ID;
                                    tblk_LL_pi2->ConnId = LL_Adv_Para.Adv_Para.LL_AdvConn_ID;
                                    LL_Adv_Para.Adv_Para.LL_AdvConn_ID = 4+1;
                                    LL_DurRxPktAccu += LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                }
                                else        
                                {
                                    LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x01;
                                }
                                LL_TmrBlk_Pt_PairRst();
                                status_LL_Tmr = (0x00 + 15);
                                break;

                            case (0x00 + 15):
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                LL_TmrBlk_Rls();
                                break;

                            case (0x00 + 12):
                                if(tblk_LL_pi2->Ticks == 1)
                                {
                                    RF_TxAutoAckOff();
                                    rafael_reset_phy_fsm_Isr();
                                    LL_TmrBlk_Rls();
                                }
                                else
                                {
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                    LLTimer_TmrRefUpd_Isr();
                                }
                                break;

                            default:
                                if(tblk_LL_pi2->Ticks == 1)
                                {
                                    LL_TmrBlk_Rls();
                                }
                                else
                                {
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                    LLTimer_TmrRefUpd_Isr();
                                }
                                break;
                            }
                            break;

                        case (4+1+1):
                            switch(status_LL_Tmr)
                            {
                            case (0x00 + 20):
                                if(tblk_LL_pi2->Ticks == 6)
                                {
                                    RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                    RF_TxAutoAckOff();
                                    rafael_reset_phy_fsm_Isr();
                                    LL_TmrBlk_Rls();
                                    status_LL_Tmr = (0x00 + 24);
                                }
                                else
                                {
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                    LLTimer_TmrRefUpd_Isr();
                                }
                            case (0x00 + 21):
                                break;

                            case (0x00 + 22):
                            case (0x00 + 23):
                            case (0x00 + 24):
                                if(tblk_LL_pi2->Ticks == 6)
                                {
                                    i16 = 0;
                                    RF_Tmr_Periodic_set_ISR(6);
                                }
                                else
                                {
                                    i16 = tblk_LL_pi2->Ticks - (6+1);
                                    tblk_LL_pi2->Ticks = (6+1);
                                    RF_Tmr_Periodic_set_ISR((6+1));
                                }
                                LL_TmrBlk_Rls();
                                if(tblk_LL_pi2 != (TBLK_LLx *)0)
                                {
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks+i16;
                                }
                                LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl += i16;
                                break;

                            default:
                                ErrorEntry(7);
                                break;
                            }
                            break;


                        case (4+1+2):
                            switch(status_LL_Tmr)
                            {
                            case (0x00 + 32):
                                status_LL_Tmr = (0x00 + 33);
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                LL_TmrBlk_Rls();
                                LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu = LL_conn_pi->LE_Conn_Para.LL_ConnLatency;  
                                LL_TmrBlk_Pt_PairRst();
                                break;

                            case (0x00 + 31):
                                tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                LLTimer_TmrRefUpd_Isr();
                                break;

                            case (0x00 + 30):
                                if(tblk_LL_pi2->Ticks == 5)
                                {
                                    RF_Tmr_Periodic_set_ISR(tblk_LL_pi2->Ticks);
                                    RF_TxAutoAckOff();
                                    rafael_reset_phy_fsm_Isr();
                                    LL_TmrBlk_Rls();
                                }
                                else
                                {
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks - 1;
                                    LLTimer_TmrRefUpd_Isr();
                                }
                                break;

                            case (0x00 + 33):
                                if(tblk_LL_pi2->Ticks == 5)
                                {
                                    i16 = 0;
                                    RF_Tmr_Periodic_set_ISR(5);
                                }
                                else
                                {
                                    i16 = tblk_LL_pi2->Ticks - (5+1);
                                    tblk_LL_pi2->Ticks = (5+1);
                                    RF_Tmr_Periodic_set_ISR((5+1));
                                }
                                LL_TmrBlk_Rls();
                                if(tblk_LL_pi2 != (TBLK_LLx *)0)
                                {
                                    tblk_LL_pi2->Ticks = tblk_LL_pi2->Ticks+i16;
                                }
                                LL_Init_Para.LE_Init_Para.LL_LE_ScanAccl += i16;
                                break;

                            default:
                                ErrorEntry(8);
                                break;
                            }
                            break;


                        default:
                            ErrorEntry(9);
                            break;
                        }
                    }
                }       
                else    
                {
                    tblk_i = tblk_LL_pi->ConnId;
                    if(tblk_i<4+1)    
                    {
                        switch(status_LL_Tmr)
                        {
                        case (0x00 + 40):
                        case (0x00 + 43):
                        case (0x00 + 51):
                        case (0x00 + 52):
                            ErrorEntry(10);
                            break;

                        case (0x00 + 45):
                        case (0x00 + 55):
                            LL_Msg_Event_Disconn_Complete(LL_conn_pi->LE_Conn_Para.ErrCode_DisConn, tblk_i);
                            LL_ReleaseConnID_Isr(tblk_i);
                            LL_MsgBlk_LL_conn_Para_Rls(tblk_i);
                        default:
                            LL_TmrBlk_Rls();
                            break;
                        }
                        LL_TmrBlk_Rls_NxtIntvl();
                    }
                    else     
                    {
                        switch(tblk_i)
                        {
                        case 4+1:   
                            switch(status_LL_Tmr)
                            {
                            case (0x00 + 13):
                                __nop();
                                break;

                            case (0x00 + 14):    
                                ErrorEntry(11);
                                break;

                            default:
                                break;
                            }
                            if((LL_Msg_AdvScnConn & 0x10))
                            {
                                k = LL_DUR_RSV[(LL_Adv_Para.Adv_Para.LL_Adv_Type&0x0F)]+(LL_Tmr_Ticks_RSV_Wakeup+1+1);
                                j = tblk_LL_pi->Next;
                                tblk_i = tblk_LL_pi->Next;
                                while(tblk_i < (2*(4+1+1)))
                                {
                                    tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                    if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                    {
                                        
                                        if(tblk_LL_pi2->Ticks >= k)
                                        {
                                            j16 = tblk_LL_pi2->Ticks-k;
                                            if(LL_Adv_Para.Adv_Para.LL_AdvMap_ID)
                                            {
                                                tblk_LL_pi2->Ticks = j16;
                                                break;
                                            }
                                            else
                                            {
                                                if(LL_Adv_Para.Adv_Para.LL_Adv_Interval_Min < LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max)
                                                {
                                                    tblk_LL_pi2->Ticks = j16;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    tblk_i = tblk_LL_pi2->Next;
                                    tblk_LL_pi = tblk_LL_pi2;
                                    LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max = LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max + tblk_LL_pi2->Ticks;
                                }
                                tblk_LL_pi->Next = TBlk_Free_LL;
                                tblk_LL_pi->TmrId = (0x00 + 1);
                                tblk_LL_pi2 = &TmrBlk_LL[TBlk_Free_LL];
                                TBlk_Free_LL = tblk_LL_pi2->Next;
                                tblk_LL_pi2->Next = tblk_i;
                                k = k-(1+1);
                                tblk_LL_pi2->Ticks = k;
                                j16 = k;

                                tblk_LL_pi2->ConnId = 4+1;
                                tblk_LL_pi2->TmrId = (0x00 + 255);

                                if(tblk_i!=(2*(4+1+1)))
                                {
                                    i16 = 1;
                                }
                                else
                                {
                                    if(tblk_i != j)
                                    {
                                        i16 = 1;
                                    }
                                    else
                                    {
                                        if(LL_Adv_Para.Adv_Para.LL_Adv_Interval_Min == (17*3))
                                        {
                                            i16 = ((17*3)/3)-k;
                                        }
                                        else
                                        {
                                            if(LL_Adv_Para.Adv_Para.LL_AdvMap_ID)
                                            {
                                                if(LL_Adv_Para.Adv_Para.LL_Adv_Interval_Min >= (80<<2))
                                                {
                                                    i16 = 80-k;
                                                }
                                            }
                                            else
                                            {
                                                if(LL_Adv_Para.Adv_Para.LL_Adv_Interval_Min>LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max)
                                                {
                                                    i16 = (LL_Adv_Para.Adv_Para.LL_Adv_Interval_Min-LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max);
                                                }
                                                else
                                                {
                                                    i16 = 1;
                                                }
                                            }
                                        }
                                    }
                                }
                                tblk_LL_pi->Ticks = i16;
                                LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max = LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max + (i16+j16);
                                if(LL_Adv_Para.Adv_Para.LL_AdvMap_ID == 0)
                                {
                                    if(LL_Adv_Para.Adv_Para.LL_Adv_Interval_Min != (17*3))
                                    {
                                        LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max = 0;
                                    }
                                    LL_Adv_Para.Adv_Para.LL_AdvMap_ID = Ch_ADV_Ch_Hop_Table[0];
                                }
                                if(LL_Adv_Para.Adv_Para.LL_Adv_Interval_Max >= 10240)
                                {
                                    mblk = mblk_LL_conn_Para[LL_Adv_Para.Adv_Para.LL_AdvConn_ID];
                                    SndMsgBlk_Isr(mblk, 4);
                                    mblk_LL_conn_Para[LL_Adv_Para.Adv_Para.LL_AdvConn_ID] = (MBLK *)0;      
                                    LL_Msg_AdvScnConn &= (~0x10);
                                    LL_Msg_AdvScnConnUpdSts &= (~0x01);
                                    LL_Msg_AdvScnConnUpd &= (~0x01);
                                    
                                }
                                if(j != (2*(4+1+1)))
                                {
                                    if(j != tblk_i)
                                    {
                                        TBlk_InUse_LL = j;
                                        LL_TmrBlk_Pt_PairRst();
                                    }
                                }
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi->Ticks);
                            }
                            else
                            {
                                if(LL_Adv_Para.Adv_Para.LL_AdvConn_ID < 4+1)
                                {
                                    LL_MsgBlk_LL_conn_Para_Rls(LL_Adv_Para.Adv_Para.LL_AdvConn_ID);
                                    LL_Adv_Para.Adv_Para.LL_AdvConn_ID = 4+1;
                                }
                                RF_WTR_intOff();
                                LL_TmrBlk_Rls();
                                LL_TmrBlk_Rls_NxtIntvl();
                            }
                            break;

                        case (4+1+1):    
                            switch(status_LL_Tmr)
                            {
                            case (0x00 + 21):
                            case (0x00 + 23):
                                rafael_reset_phy_fsm_Isr();
                                break;

                            default:
                                break;
                            }
                            if((LL_Msg_AdvScnConn & 0x20))
                            {
                                if(LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl >= ((4<<2)+2))
                                {
                                    i16 = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl+1+1;
                                    j16 = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_Scan_Interval;
                                }
                                else
                                {
                                    i16 = ((4<<2)+2)+1+1;
                                    j16 = 0;
                                }
                                j = tblk_LL_pi->Next;
                                tblk_i = tblk_LL_pi->Next;
                                while(tblk_i < (2*(4+1+1)))
                                {
                                    tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                    if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                    {
                                        if(j16 >= LL_Scan_Para.LE_Set_Scan_Para.LL_LE_Scan_Interval)
                                        {
                                            if(tblk_LL_pi2->Ticks>=((4<<2)+2)+1+1)
                                            {
                                                if((tblk_LL_pi2->Ticks-i16)>0)
                                                {
                                                    tblk_LL_pi2->Ticks = (tblk_LL_pi2->Ticks-i16)+1;
                                                    i16 = i16-(1+1);
                                                }
                                                else
                                                {
                                                    i16 = tblk_LL_pi2->Ticks-(1+1);
                                                    tblk_LL_pi2->Ticks = 1;
                                                }
                                                break;
                                            }
                                        }
                                    }
                                    tblk_i = tblk_LL_pi2->Next;
                                    if(i16 == ((4<<2)+2)+1+1)
                                    {
                                        j16 = j16+tblk_LL_pi2->Ticks;
                                    }
                                    tblk_LL_pi->Next = tblk_LL_pi2->Next;
                                    tblk_LL_pi2->Next = TBlk_InUse_LL;
                                }
                                if(tblk_i==(2*(4+1+1)))
                                {
                                    if(j == (2*(4+1+1)))
                                    {
                                        i16 = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_Scan_Window;
                                    }
                                    else
                                    {
                                        i16 = ((4<<2)+2);
                                    }
                                }
                                if(j16 < LL_Scan_Para.LE_Set_Scan_Para.LL_LE_Scan_Interval)
                                {
                                    j16 = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_Scan_Interval - j16;
                                }
                                tblk_LL_pi->Next = TBlk_Free_LL;
                                tblk_LL_pi->TmrId = (0x00 + 1);
                                tblk_LL_pi->Ticks = j16+1;

                                tblk_LL_pi2 = &TmrBlk_LL[TBlk_Free_LL];
                                TBlk_Free_LL = tblk_LL_pi2->Next;
                                tblk_LL_pi2->Next = tblk_i;

                                tblk_LL_pi2->Ticks = i16;
                                if(LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl > i16)
                                {
                                    LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl - i16;
                                }
                                else
                                {
                                    LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_Scan_Window;
                                    LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID = CH_ADV_CH_HOP_BY_MAP_TABLE[0x07][LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID];
                                    if(LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID == 0)
                                    {
                                        LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID = CH_ADV_CH_HOP_BY_MAP_TABLE[0x07][0];
                                    }
                                }
                                tblk_LL_pi2->ConnId = (4+1+1);
                                tblk_LL_pi2->TmrId = (0x00 + 255);

                                if(j != (2*(4+1+1)))
                                {
                                    if(j != tblk_i)
                                    {
                                        TBlk_InUse_LL = j;
                                        LL_TmrBlk_Pt_PairRst();
                                    }
                                }
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi->Ticks);
                            }
                            else
                            {
                                LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID = 0;
                                LL_TmrBlk_Rls();
                                LL_TmrBlk_Rls_NxtIntvl();
                            }
                            break;


                        case (4+1+2):    
                            switch(status_LL_Tmr)
                            {
                            default:
                                break;
                            }
                            if((LL_Msg_AdvScnConn & 0x40))
                            {
                                if(LL_Init_Para.LE_Init_Para.LL_LE_ScanAccl >= ((4<<2)+2))
                                {
                                    i16 = LL_Init_Para.LE_Init_Para.LL_LE_ScanAccl+1+1;
                                    j16 = LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval;
                                }
                                else
                                {
                                    i16 = ((4<<2)+2)+1+1;
                                    j16 = 0;
                                }
                                j = tblk_LL_pi->Next;
                                tblk_i = tblk_LL_pi->Next;
                                while(tblk_i < (2*(4+1+1)))
                                {
                                    tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                    if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                    {
                                        if(j16 >= LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval)
                                        {
                                            if(tblk_LL_pi2->Ticks>=((4<<2)+2)+1+1)
                                            {
                                                if((tblk_LL_pi2->Ticks-i16)>0)
                                                {
                                                    tblk_LL_pi2->Ticks = (tblk_LL_pi2->Ticks-i16)+1;
                                                    i16 = i16-(1+1);
                                                }
                                                else
                                                {
                                                    i16 = tblk_LL_pi2->Ticks-(1+1);
                                                    tblk_LL_pi2->Ticks = 1;
                                                }
                                                break;
                                            }
                                        }
                                    }
                                    tblk_i = tblk_LL_pi2->Next;
                                    if(i16 == ((4<<2)+2)+1+1)
                                    {
                                        j16 = j16+tblk_LL_pi2->Ticks;
                                    }
                                    tblk_LL_pi->Next = tblk_LL_pi2->Next;
                                    tblk_LL_pi2->Next = TBlk_InUse_LL;
                                }
                                if(tblk_i==(2*(4+1+1)))
                                {
                                    if(j == (2*(4+1+1)))
                                    {
                                        i16 = LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window;
                                    }
                                    else
                                    {
                                        i16 = ((4<<2)+2);
                                    }
                                }
                                if(j16 < LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval)
                                {
                                    j16 = LL_Init_Para.LE_Init_Para.LL_LE_Scan_Interval - j16;
                                }
                                tblk_LL_pi->Next = TBlk_Free_LL;
                                tblk_LL_pi->TmrId = (0x00 + 1);
                                tblk_LL_pi->Ticks = j16+1;

                                tblk_LL_pi2 = &TmrBlk_LL[TBlk_Free_LL];
                                TBlk_Free_LL = tblk_LL_pi2->Next;
                                tblk_LL_pi2->Next = tblk_i;

                                tblk_LL_pi2->Ticks = i16;
                                if(LL_Init_Para.LE_Init_Para.LL_LE_ScanAccl > i16)
                                {
                                    LL_Init_Para.LE_Init_Para.LL_LE_ScanAccl = LL_Init_Para.LE_Init_Para.LL_LE_ScanAccl - i16;
                                }
                                else
                                {
                                    LL_Init_Para.LE_Init_Para.LL_LE_ScanAccl = LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window;
                                    LL_Init_Para.LE_Init_Para.LL_AdvMap_ID = CH_ADV_CH_HOP_BY_MAP_TABLE[0x07][LL_Init_Para.LE_Init_Para.LL_AdvMap_ID];
                                    if(LL_Init_Para.LE_Init_Para.LL_AdvMap_ID == 0)
                                    {
                                        LL_Init_Para.LE_Init_Para.LL_AdvMap_ID = CH_ADV_CH_HOP_BY_MAP_TABLE[0x07][0];
                                    }
                                }
                                tblk_LL_pi2->ConnId = (4+1+2);
                                tblk_LL_pi2->TmrId = (0x00 + 255);

                                if(j != (2*(4+1+1)))
                                {
                                    if(j != tblk_i)
                                    {
                                        TBlk_InUse_LL = j;
                                        LL_TmrBlk_Pt_PairRst();
                                    }
                                }
                                RF_Tmr_Periodic_set_ISR(tblk_LL_pi->Ticks);
                            }
                            else
                            {
                                LL_Init_Para.LE_Init_Para.LL_AdvMap_ID = 0;
                                LL_TmrBlk_Rls();
                                LL_TmrBlk_Rls_NxtIntvl();
                            }
                            break;


                        default:
                            ErrorEntry(12);
                            break;
                        }
                    }
                    status_LL_Tmr = 0x00;
                    if(RF_WTR_EnChk()==0)
                    {
                        
                        rafael_reset_phy_fsm_Isr();
                    }
                    
                    
                    if(tblk_LL_pi->TmrId < (0x00 + 2))    
                    {
                        if(tblk_LL_pi->ConnId < 4+1)                            
                        {
                            j = tblk_LL_pi->ConnId;
                            pLL_Conn = &LL_Conn_Para[j];
                            LL_SMP_DataCh = pLL_Conn->LE_Conn_Para.LL_SMP_DataCh;
                            if(LL_SMP_DataCh&(0x01|0x02))                                
                            {
                                SPI_PDMA_SetTx(131U, (uint32_t)pLL_Conn->LE_Conn_Para . LL_SMP_IV, 4+4+16);

                                if(tblk_LL_pi->ConnId < 4)      
                                {
                                    pLL_Conn->LE_Conn_Para.LL_SMP_packetCounterTd |= 0x80;
                                    SPI_PDMA_SetTx(126U, (uint32_t)&pLL_Conn->LE_Conn_Para . LL_SMP_packetCounterT, 5);
                                    SPI_PDMA_waitFinish();
                                    pLL_Conn->LE_Conn_Para.LL_SMP_packetCounterTd &= (~0x80);
                                }
                                else

                                {

                                    pLL_Conn->LE_Conn_Para.LL_SMP_packetCounterRd |= 0x80;
                                    SPI_PDMA_SetTx(126U, (uint32_t)&pLL_Conn->LE_Conn_Para . LL_SMP_packetCounterR, 5);
                                    SPI_PDMA_waitFinish();
                                    pLL_Conn->LE_Conn_Para.LL_SMP_packetCounterRd &= (~0x80);

                                }
                            }
                            RF_CCM_AES_Mode_set((LL_SMP_DataCh&(0x01|0x02)));
                        }
                    }

                    if(tblk_LL_pi->Ticks<(1+1))
                    {
                        return;
                    }
                    tblk_i = (LL_Msg_AdvScnConnUpdSts^LL_Msg_AdvScnConnUpd);
                    if((tblk_i & (0x01|0x02|0x04)))
                    {
                        LL_Msg_AdvScnConnUpdSts ^= tblk_i;
                        LL_Msg_AdvScnConn = (LL_Msg_AdvScnConnUpdSts & (0x01|0x02|0x04))<<4;
                        if((tblk_i & 0x01))
                        {
                            Knl_MemCpy_Isr(&LL_Adv_Para.Adv_Para.LL_AdvMap_ID, &LL_Adv_Para_UpdBuf.Adv_Para.LL_AdvMap_ID, sizeof(Adv_Param));
                            Knl_MemCpy_Isr(Ch_ADV_Ch_Hop_Table, Ch_ADV_Ch_Hop_TableBuf, 4);
                            LL_Msg_AdvScnConn |= 0x10;
                            i16 = LL_DUR_RSV[(LL_Adv_Para.Adv_Para.LL_Adv_Type&0x0F)]+(LL_Tmr_Ticks_RSV_Wakeup+1+1);

                            tblk_i = tblk_LL_pi2->Next;
                            while(tblk_i < (2*(4+1+1)))
                            {
                                tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                {
                                    if(tblk_LL_pi2->Ticks >= i16)
                                    {
                                        tblk_LL_pi2->Ticks = (tblk_LL_pi2->Ticks-i16)+1;
                                        break;
                                    }
                                }
                                tblk_i = tblk_LL_pi2->Next;
                                tblk_LL_pi = tblk_LL_pi2;
                            }
                            tblk_LL_pi->Next = TBlk_Free_LL;
                            tblk_LL_pi = &TmrBlk_LL[TBlk_Free_LL];
                            tblk_LL_pi->TmrId = (0x00 + 1);
                            tblk_LL_pi->Ticks = 1;
                            tblk_LL_pi2 = &TmrBlk_LL[tblk_LL_pi->Next];
                            TBlk_Free_LL = tblk_LL_pi2->Next;

                            tblk_LL_pi2->Next = tblk_i;
                            tblk_LL_pi2->Ticks = (i16-1-1);
                            tblk_LL_pi2->TmrId = (0x00 + 255);

                            tblk_LL_pi->ConnId = 4+1;
                            tblk_LL_pi2->ConnId = 4+1;

                            LL_TmrBlk_Pt_PairRst();
                        }
                        else if((tblk_i & 0x02))
                        {
                            if((tblk_i & 0x40))
                            {
                                Knl_MemCpy_Isr(&LL_Scan_Para.LE_Set_Scan_Para.LL_AdvMap_ID, &LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_AdvMap_ID, (sizeof(LE_Set_Scan_Para)));
                                Knl_MemCpy_Isr(Ch_ADV_Ch_Hop_Table, Ch_ADV_Ch_Hop_TableBuf, 4);
                            }
                            LL_Msg_AdvScnConn |= 0x20;
                            i16 = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_Scan_Window+1+1;

                            tblk_i = tblk_LL_pi2->Next;
                            while(tblk_i < (2*(4+1+1)))
                            {
                                tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                {
                                    if(tblk_LL_pi2->Ticks>=((4<<2)+2)+1+1)
                                    {
                                        if((tblk_LL_pi2->Ticks-i16)>0)
                                        {
                                            tblk_LL_pi2->Ticks = (tblk_LL_pi2->Ticks-i16)+1;
                                            i16 = i16-(1+1);
                                        }
                                        else
                                        {
                                            i16 = tblk_LL_pi2->Ticks-(1+1);
                                            tblk_LL_pi2->Ticks = 1;
                                        }
                                        break;
                                    }
                                }
                                tblk_LL_pi = tblk_LL_pi2;
                                tblk_i = tblk_LL_pi2->Next;
                            }
                            if(tblk_i==(2*(4+1+1)))
                            {
                                i16 = ((4<<2)+2);
                            }
                            tblk_LL_pi->Next = TBlk_Free_LL;
                            tblk_LL_pi = &TmrBlk_LL[TBlk_Free_LL];
                            tblk_LL_pi->TmrId = (0x00 + 1);
                            tblk_LL_pi->Ticks = 1;
                            tblk_LL_pi2 = &TmrBlk_LL[tblk_LL_pi->Next];
                            TBlk_Free_LL = tblk_LL_pi2->Next;
                            tblk_LL_pi2->Next = tblk_i;
                            tblk_LL_pi2->Ticks = i16;
                            LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl = LL_Scan_Para.LE_Set_Scan_Para.LL_LE_ScanAccl - i16;
                            tblk_LL_pi2->TmrId = (0x00 + 255);

                            tblk_LL_pi->ConnId = (4+1+1);
                            tblk_LL_pi2->ConnId = (4+1+1);

                            LL_TmrBlk_Pt_PairRst();
                        }

                        else if((tblk_i & 0x04))
                        {
                            LL_Msg_AdvScnConn |= 0x40;
                            i16 = LL_Init_Para.LE_Init_Para.LL_LE_Scan_Window+1+1;

                            tblk_i = tblk_LL_pi2->Next;
                            while(tblk_i < (2*(4+1+1)))
                            {
                                tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                {
                                    if(tblk_LL_pi2->Ticks>=((4<<2)+2)+1+1)
                                    {
                                        if((tblk_LL_pi2->Ticks-i16)>0)
                                        {
                                            tblk_LL_pi2->Ticks = (tblk_LL_pi2->Ticks-i16)+1;
                                            i16 = i16-tblk_LL_pi2->Ticks;
                                        }
                                        else
                                        {
                                            i16 = tblk_LL_pi2->Ticks-(1+1);
                                            tblk_LL_pi2->Ticks = 1;
                                        }
                                        break;
                                    }
                                }
                                tblk_LL_pi = tblk_LL_pi2;
                                tblk_i = tblk_LL_pi2->Next;
                            }
                            if(tblk_i==(2*(4+1+1)))
                            {
                                i16 = ((4<<2)+2);
                            }
                            tblk_LL_pi->Next = TBlk_Free_LL;
                            tblk_LL_pi = &TmrBlk_LL[TBlk_Free_LL];
                            tblk_LL_pi->TmrId = (0x00 + 1);
                            tblk_LL_pi->Ticks = 1;
                            tblk_LL_pi2 = &TmrBlk_LL[tblk_LL_pi->Next];
                            TBlk_Free_LL = tblk_LL_pi2->Next;
                            tblk_LL_pi2->Next = tblk_i;
                            tblk_LL_pi2->Ticks = i16;
                            tblk_LL_pi2->TmrId = (0x00 + 255);

                            tblk_LL_pi->ConnId = (4+1+2);
                            tblk_LL_pi2->ConnId = (4+1+2);

                            LL_TmrBlk_Pt_PairRst();
                        }

                    }
                    
                }   
            }     
            if(TBlk_InUse_LL != (2*(4+1+1)))
            {
                if(tblk_LL_pi->Ticks >= 3)
                {
                    Ble_EC_Chk_Isr();
                }

                if(tblk_LL_pi->Ticks > 80)
                {
                    RF_PowerSaving_En_Isr();
                }

            }
        }
        else     
        {
            if(i16 > Tmr16Interval)      
            {
                tblk_LL_pi->Ticks = i16 - Tmr16Interval;
                tblk_i = tblk_LL_pi->ConnId;
                if(tblk_i<4+1)        
                {

                    if(tblk_i < 4)
                    {
                        __nop();
                    }
                    else

                    {

                        __nop();

                    }
                }
                else      
                {
                    switch(tblk_i)
                    {
                    case 4+1:
                        __nop();
                        break;

                    case (4+1+1):
                        __nop();
                        break;


                    case (4+1+2):
                        __nop();
                        break;


                    default:
                        __nop();
                        break;
                    }
                }
                __nop();
            }
            else
            {
                __nop();
            }
        }
    }
}


void LLWTR_TxEndRpt(Uint8 ConnId)
{
    MBLK *mblk;

    mblk = GetMsgBlk_Isr();
    mblk->Primitive = (0x60+0x04);
    mblk->Para.MHC_Num_Of_Completed_Pckts_Para.HCI_Conn_Hdl_L = ConnId;
    SndMsgBlk_Isr(mblk, 4);
}


Uint8 LLWTR_Rx2Hc(Uint8 ConnId, LL_Para_Header HeaderR)
{
    MBLK *mblk;
    Uint8 i, llid, LL_SMP_Gate;
    Uint8 len;
    Uint8 mskPhy;
    MQUEUE *pqueue;

    i = HeaderR.HeaderLen;
    llid = HeaderR.HeaderSts&0x03;
    pqueue = &LL_ConnDataInQ[ConnId];
    LL_SMP_Gate = LL_conn_pi->LE_Conn_Para.LL_SMP_Gate;

    if((LL_SMP_Gate != 0))
    {
        switch(llid)
        {
        case 0x03:
            break;

        case 0x01:
        case 0x02:
            if(i)
            {
                LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x3D;
                return (0xB0+0x00);
            }
            break;

        default:
            return (0xB0+0x00);
        }
    }
    if(pqueue->QOut != (MBLK *)0)
    {
        switch(llid)
        {
        case 0x01:
            if(i)
            {
                break;
            }
            len = 0;
            mblk = pqueue->QOut;
            
            while(mblk != (MBLK *)0)
            {
                len += mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_DataPkt_LthL;
                mblk = mblk->Next;
                
                
            }
            if(len!=((pqueue->QOut)->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_DataPkt_Data[0]+4))
            {
                break;
            }
        case 0x02:
            (pqueue->QOut)->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_Conn_Hdl_L = ConnId;
            (pqueue->QOut)->Primitive = (0x60+0x11);
            SndMsgBlks_Isr((pqueue->QOut), 4);
            pqueue->QOut = (MBLK *)0;
            pqueue->QIn = (MBLK *)0;
            break;

        default:
            break;
        }
    }

    if(i)
    {
#line 7147 "..\\..\\..\\source\\LL.c"
        mblk = LL_ConnDataInTmp.QOut;
        if(mblk==(MBLK *)0)
        {
            return (0xB0+0x00);
        }
        else
        {
            if(llid==0x03)
            {
                mblk->Primitive = (0xB0+0x37);
            }
            else if(llid==0x02)
            {
                mblk->Primitive = (0x60+0x11);
            }
            else
            {
                mblk->Primitive = (0x60+0x18);
            }
        }

        if(llid==0x03)
        {
            mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_DataPkt_LthL = i;
            mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_Conn_Hdl_L = ConnId;
            if(mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode > 0x19)
            {
                i = mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode;
            }
            else
            {
                if((mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_DataPkt_LthL-1)==LL_RX_CONN_ID_OP_LENGTH[mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode])
                {
                    i = mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode;

                    if(ConnId < 4)
                    {
                        i = LL_RX_CONN_ID_MASTER_SIDE_ONLY[i];
                    }
                    else

                    {

                        i = LL_RX_CONN_ID_SLAVE_SIDE_ONLY[i];

                    }
                }
                else
                {
                    i = (0x19+1);
                }
                mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode = i;
            }
            if((LL_conn_pi->LE_Conn_Para.LL_Conn_ID&0x40))
            {
                if(i == LL_PR_CONN_ID_PAIR[(LL_conn_pi->LE_Conn_Para.LL_Conn_ID-0x40)])
                {
                    LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                    LL_conn_pi->LE_Conn_Para.LL_PrToutAccu = 0;
                }
                else
                {
                    switch(i)
                    {
                    case 0x07:
                    case 0x0D:
                    case 0x11:
                        LL_conn_pi->LE_Conn_Para.LL_PrToutAccu = 0;
                        break;

                    case 0x17:        
                        if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == (0x40|0x16))
                        {
                            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                            LL_conn_pi->LE_Conn_Para.LL_PrToutAccu = 0;
                        }
                        break;

                    default:
                        break;
                    }
                }
            }
            else
            {
                if(i > 0x19)
                {

                }
                else
                {

                    if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == 0x04)
                    {
                        LL_conn_pi->LE_Conn_Para.LL_Conn_ID = LL_RX_CONN_ID_INIT[i];
                    }
                    else
                    {
                        if(LL_RX_CONN_ID_INIT[i] != 0x04)
                        {
                            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = LL_RX_CONN_ID_INIT[i];
                        }
                    }
                }
            }
            switch(i)
            {
            case 0x00:
            case 0x01:
                
                if((LL_SMP_Gate != 0))
                {
                    LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x3D;
                    mblk->Primitive = (0xB0+0x00);
                    break;
                }
                if(i)
                {
                    Knl_MemCpy_Isr(LL_conn_pi->LE_Conn_Para.LL_ChMapReM, &mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_DataPkt_Data[1], 5);
                    Knl_MemCpy_Isr((uint8_t *)&LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd, &mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_DataPkt_Data[1+5], 2);    
                }
                else
                {
                    Knl_MemCpy_Isr(&LL_conn_pi->LE_Conn_Para.WinSizeUpd, &mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_DataPkt_Data[1], 11);
                }
                if((LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd-LL_conn_pi->LE_Conn_Para.LL_EventCounter==0)                         ||(LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd-LL_conn_pi->LE_Conn_Para.LL_EventCounter>=32767))

                {
                    LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x28;
                    mblk->Primitive = (0xB0+0x00);
                }

                else
                {
                    LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrgUpd = LL_conn_pi->LE_Conn_Para.LL_ConnIntervalUpd;

                    LL_conn_pi->LE_Conn_Para.WinSizeUpd = (LL_conn_pi->LE_Conn_Para.WinSizeUpd*10);                        
                    LL_conn_pi->LE_Conn_Para.WinOffsetUpd = (LL_conn_pi->LE_Conn_Para.WinOffsetUpd*10);                    
                    LL_conn_pi->LE_Conn_Para.LL_ConnIntervalUpd = (LL_conn_pi->LE_Conn_Para.LL_ConnIntervalUpd*10);        
                    LL_conn_pi->LE_Conn_Para.LL_SvisionTimeoutUpd = (LL_conn_pi->LE_Conn_Para.LL_SvisionTimeoutUpd<<3);    
#line 7293 "..\\..\\..\\source\\LL.c"
                }

                break;

            case 0x02:
                LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[0];
                mblk->Primitive = (0xB0+0x00);
                break;

            case 0x03:
                LL_SMP_DataCh = 0x02;
                if(LL_SMP_Gate == 0)
                {
                    LL_conn_pi->LE_Conn_Para.LL_SMP_Gate = 1;
                }
                break;

            case 0x04:
                LL_SMP_DataCh = 0x01;
                LL_conn_pi->LE_Conn_Para.LL_Conn_ID = (0x40|0x04);
                if(LL_SMP_Gate == 0)
                {
                    LL_conn_pi->LE_Conn_Para.LL_SMP_Gate = 1;
                }
                break;

            case 0x06:
                mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[0] = LL_SMP_Gate;
                if((LL_SMP_DataCh&(0x01|0x02)) == (0x01|0x02))
                {
                    LL_conn_pi->LE_Conn_Para.LL_SMP_Gate = 0;
                }
            case 0x05:
                LL_SMP_DataCh = (0x01|0x02);
                break;

            case 0x0A:
                LL_SMP_DataCh = 0x01;
                break;

            case 0x0B:
                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterR = 0;
                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterRd = 0;
                LL_SMP_DataCh = 0;
                break;

            case 0x08:
            case 0x09:
            case 0x0E:
                LL_conn_pi->LE_Conn_Para.LL_Feature = (LL_FEATURE[0] & mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[0]);
                LL_conn_pi->LE_Conn_Para.LL_Feature1 = (LL_FEATURE[1] & mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[1]);
            case 0x0C:
                break;

            case 0x0D:
            case 0x11:
                if((LL_conn_pi->LE_Conn_Para.LL_Conn_ID == (0x40|0x03))||(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == (0x40|0x04)))
                {
                    LL_conn_pi->LE_Conn_Para.LL_SMP_Gate = 0;
                    LL_SMP_DataCh = 0;
                }
                break;

            case 0x18:
                LL_conn_pi->LE_Conn_Para.LL_Conn_ID = (0x80|0x18);
                Knl_MemCpy_Isr((Uint8 *)&LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd, &mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[2], 2);
                if((LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd-LL_conn_pi->LE_Conn_Para.LL_EventCounter==0)                         ||(LL_conn_pi->LE_Conn_Para.LL_EventCounterUpd-LL_conn_pi->LE_Conn_Para.LL_EventCounter>=32767))

                {
                    LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x28;
                    mblk->Primitive = (0xB0+0x00);
                    break;
                }
                else
                {

                    mskPhy = ((mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[0]&mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[1])&(~(0|0x04|0xF8)));     
                    if(mskPhy)
                    {
                        if(mskPhy!=LL_conn_pi->LE_Conn_Para.LL_Tx_PHYS)                       
                        {
                            LL_conn_pi->LE_Conn_Para.LL_Tx_PHYsUpd = mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[1];   
                            LL_conn_pi->LE_Conn_Para.LL_Rx_PHYsUpd = mblk->Para.LLHC_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Data[0];   
                            
                        }
                        else
                        {
                            LL_conn_pi->LE_Conn_Para.LL_Tx_PHYsUpd = 0;
                            LL_conn_pi->LE_Conn_Para.LL_Rx_PHYsUpd = 0;
                        }
                    }
                    else
                    {
                        LL_conn_pi->LE_Conn_Para.LL_Tx_PHYsUpd = 0;
                        LL_conn_pi->LE_Conn_Para.LL_Rx_PHYsUpd = 0;
                    }

                }
            default:
                if((LL_SMP_Gate != 0))
                {
                    LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x3D;
                    mblk->Primitive = (0xB0+0x00);
                }
                break;
            }

            if(ConnId < 4)
            {
                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd |= 0x80;   
            }

            LL_conn_pi->LE_Conn_Para.LL_SMP_DataCh = LL_SMP_DataCh;
            __nop();
            SndMsgBlks_Isr(mblk, 0);
        }
        else
        {
            if(LL_ConnDataInTmp.QIn != (MBLK *)0)
            {
                if(pqueue->QIn != (MBLK *)0)
                {
                    if((LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0)
                    {
                        Knl_MemCpy_Isr(&((pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data)[(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL],
                                       &((LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data)[(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL],
                                       (((6+31+7)-2-1)-(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL));

                        (pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = ((6+31+7)-2-1);
                    }
                    else
                    {
                        if((LL_ConnDataInTmp.QIn) != LL_ConnBuffPrcsR.BufPrcsF)
                        {
                            Knl_MemCpy_Isr(&((pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data)[(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL],
                                           &((LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data)[(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL],
                                           (((6+31+7)-2-1)-(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL));

                            (pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = ((6+31+7)-2-1);
                        }
                        else
                        {
                            if(LL_ConnBuffPrcsR.BufPrcsF != LL_ConnBuffPrcsR.BufPrcsN)
                            {
                                Knl_MemCpy_Isr(&((pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data)[(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL],
                                               &((LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data)[(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL],
                                               (((6+31+7)-2-1)-(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL));

                                (pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = ((6+31+7)-2-1);
                            }
                            else
                            {
                                Knl_MemCpy_Isr(&((pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data)[(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL],
                                               &((LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data)[(pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL],
                                               LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL);

                                (pqueue->QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL += LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL;
                            }
                        }
                    }

                    if((LL_ConnDataInTmp.QOut)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0)
                    {
                        if((LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 == 0)
                        {
                            if(LL_ConnDataInTmp.QIn != LL_ConnBuffPrcsR.BufPrcsF)
                            {
                                FreeMsgBlk_Isr(LL_ConnDataInTmp.QIn);
                                LL_ConnDataInTmp.QIn = LL_ConnBuffPrcsR.BufPrcsF;
                                if(LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 == ((6+31+7)-2-1))
                                {
                                    if(LL_ConnBuffPrcsR.BufPrcsN != (MBLK *)0)
                                    {
                                        LL_ConnDataInTmp.QIn->Next = LL_ConnBuffPrcsR.BufPrcsN;
                                        LL_ConnDataInTmp.QIn = LL_ConnBuffPrcsR.BufPrcsN;
                                        (LL_ConnDataInTmp.QIn)->Next = (MBLK *)0;
                                    }
                                    else
                                    {
                                        (LL_ConnDataInTmp.QIn)->Next = (MBLK *)0;
                                    }
                                }
                                else
                                {
                                    (LL_ConnDataInTmp.QIn)->Next = (MBLK *)0;
                                }
                            }
                        }

                        if((LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 != (LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL)
                        {
                            (LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = (LL_ConnDataInTmp.QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0;
                        }
                        (pqueue->QIn)->Next = LL_ConnDataInTmp.QOut;
                        pqueue->QIn = LL_ConnDataInTmp.QIn;
                    }
                    else
                    {
                        FreeMsgBlks_Isr(LL_ConnDataInTmp.QOut);
                    }
                }
                else
                {
                    pqueue->QOut = LL_ConnDataInTmp.QOut;
                    pqueue->QIn = LL_ConnDataInTmp.QIn;
                }
            }
        }
        LL_ConnDataInTmp.QOut = (MBLK *)0;
        LL_ConnDataInTmp.QIn = (MBLK *)0;

        i = mblk->Primitive;
    }
    return i;
}


Uint8 LLWTR_MDset(TBLK_LLx *tblk_LL_pi, Uint8 DUR_LL_Rsv_MD_Type)
{
    Uint32 *pi16;
    Uint16 i16;

    i16 = (LL_conn_pi->LE_Conn_Para.LL_ConnInterval>>5)+(3+3*5);
    i16 += DUR_LL_Rsv_MD_Type;
    if(tblk_LL_pi->Next<(2*(4+1+1)))
    {
        pi16 = &TmrBlk_LL[tblk_LL_pi->Next].Ticks;
        if((*pi16+tblk_LL_pi->Ticks) > i16)
        {
            *pi16 = (*pi16+tblk_LL_pi->Ticks)-i16;
            tblk_LL_pi->Ticks = i16;
            return 0;
        }
        else
        {
            return (!0);
        }
    }
    else
    {
        if(LL_DurRxPktAccu<i16)
        {
            return (!0);
        }
        else
        {
            tblk_LL_pi->Ticks = DUR_LL_Rsv_MD_Type;
            return 0;
        }
    }
}


void LLWTR_Isr(void)
{
    Uint8 tblk_i, j;
    Uint8 k, len;
    uint8_t *ptAddr;
    Uint32 i16;
    MBLK *mblk;
    LL_Para_Header HeaderR;
    struct Conn_Req_Para * connReqRx;

    Uint16 j16, *pi16;
    struct Conn_Req_Para_Alt * connReqTx;
    Uint32 u32i;

    extern uint32_t Timeline24;
    extern const uint8_t TIMELINE24_3750US_IDX[];
    extern void LLWTRFIFOGet_Isr(Uint8 len);

    tblk_i = tblk_LL_pi->ConnId;
    if(tblk_i<4+1)       
    {
        LL_ConnDataTmpGate = 0;

        if(tblk_i<4)      
        {
            switch(status_LL_Tmr)
            {
            case (0x00 + 40):
            case (0x00 + 43):
                if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == (0x80|0x02))
                {
                    rafael_reset_phy_fsm_Isr();     
                    status_LL_Tmr = (0x00 + 45);
                    break;
                }
                status_LL_Tmr = (0x00 + 42);
                if(LL_conn_pi->LE_Conn_Para.LL_SMP_Gate != 0)
                {
                    RF_TxAutoAckOff();
                    rafael_reset_phy_fsm_Isr();    
                    status_LL_Tmr = (0x00 + 44);
                }
                if(LLWTR_MDset(tblk_LL_pi2, (55+1)))
                {
                    RF_TxAutoAckOff();
                    rafael_reset_phy_fsm_Isr();    
                    status_LL_Tmr = (0x00 + 44);
                }
                if((LL_SMP_DataCh&0x02))
                {
                    SPI_PDMA_SetTx(126U, (uint32_t)&LL_conn_pi->LE_Conn_Para . LL_SMP_packetCounterR, 5);
                }
                RF_CCM_AES_Mode_set((LL_SMP_DataCh&(0x01|0x02)));
                break;

            case (0x00 + 41):
            case (0x00 + 42):
                HeaderR.HeaderSts = LL_Para_IntervalR.HeaderSts;
                HeaderR.HeaderLen = LL_Para_IntervalR.HeaderLen;
                tblk_i = LL_conn_pi->LE_Conn_Para.WinSize_DataHdr&(0x08|0x04);
                if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == 0x03)
                {
                    LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                    LL_conn_pi->LE_Conn_Para.LL_SvToutAccu = 0;
                    LL_conn_pi->LE_Conn_Para.LL_PrToutAccu = 0;
                    tblk_i = 0;
                    LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout = LL_conn_pi->LE_Conn_Para.LL_SvisionTimeoutUpd;  
                }
                status_LL_Tmr = (0x00 + 44);
                if(RF_CRCchk() == (0UL))
                {
                    rafael_reset_phy_fsm_Isr();     
                    if(LL_ConnDataInTmp.QIn != (MBLK *)0)
                    {
                        FreeMsgBlks_Isr(LL_ConnDataInTmp.QOut);
                        LL_ConnDataInTmp.QOut = (MBLK *)0;
                        LL_ConnDataInTmp.QIn = (MBLK *)0;
                    }
                    LL_conn_pi->LE_Conn_Para.WinSize_DataHdr = tblk_i;
                    LLWTR_Buffer_nAck_Isr(tblk_LL_pi->ConnId);
                    break;
                }
                SPI_PDMA_SetRx_Isr(166U, (uint32_t)&rssi_read_data[0], 3);;
                
                
                k = (HeaderR.HeaderSts&(0x08|0x04))>>2;
                if(LL_ACK_FLOW[(tblk_i|k)]&0x40)
                {
                    if(HeaderR.HeaderLen)
                    {
                        len = ((SPI_1BYT_SetRx_Isr(106U)));
                        LLWTRFIFOGet_Isr(len);
                        if(LL_ConnDataInTmp.QOut==(MBLK *)0)
                        {
                            HeaderR.HeaderSts &= (~0x10);
                            k = LL_ACK_FLOW_RE_TO[k];
                        }
                    }
                    tblk_i = LL_ACK_FLOW[(tblk_i|k)];
                }
                else
                {
                    tblk_i = LL_ACK_FLOW[(tblk_i|k)];
                }
                if(tblk_i&0x40)
                {
                    if(HeaderR.HeaderLen)
                    {
                        if((LL_SMP_DataCh&0x02))
                        {
                            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterR++;
                            if(LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterR==0)
                            {
                                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterRd++;
                                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterRd &= (~0x80);
                            }
                        }
                    }
                }
                if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == 0x06)
                {
                    LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                    LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout = LL_conn_pi->LE_Conn_Para.LL_SvisionTimeoutUpd;    
                }

                j = 0;
                if(tblk_i&0x80)
                {
                    mblk = LLWTR_Buffer_wAck_Isr(tblk_LL_pi->ConnId);
                    if((LL_ConnBuffPt[tblk_LL_pi->ConnId] != (MBLK *)0)||(mblk != (MBLK *)0))
                    {
                        if(LL_ConnBuffPt[tblk_LL_pi->ConnId] == (MBLK *)0)
                        {
                            j = (mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_Conn_Hdl_H)|0x20;
                            k = mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode;
                            
                            FreeMsgBlk_Isr(mblk);
                        }
                        if((LL_SMP_DataCh&0x01)&&(mblk != (MBLK *)0))
                        {
                            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterT++;
                            if(LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterT==0)
                            {
                                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd++;
                                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd &= (~0x80);
                            }
                        }
                    }
                    mblk = LL_Queue_To_Buffer_Isr(tblk_LL_pi->ConnId);
                    if(LL_conn_pi->LE_Conn_Para.LL_SMP_Gate == 0)
                    {
                        if((HeaderR.HeaderSts&0x10))
                        {
                            status_LL_Tmr = (0x00 + 43);
                        }
                        else
                        {
                            if(LL_conn_pi->LE_Conn_Para.WinSize_DataHdr & 0x10)
                            {
                                status_LL_Tmr = (0x00 + 43);
                            }
                        }
                    }
                    if(status_LL_Tmr == (0x00 + 44))
                    {
                        rafael_reset_phy_fsm_Isr();     
                    }
                    else
                    {
                        if(LLWTR_MDset(tblk_LL_pi2, (37+1)))
                        {
                            rafael_reset_phy_fsm_Isr();     
                            status_LL_Tmr = (0x00 + 44);
                        }
                        else
                        {
                            if(mblk == (MBLK *)0)
                            {
                                LL_Para_Interval.HeaderSts = ((tblk_i|0x01)&(~0xE0));
                                LL_Para_Interval.HeaderLen = 0;
                                RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);
                            }
                            else
                            {
                                if((LL_ConnDataQ[tblk_LL_pi->ConnId].QOut!=(MBLK *)0)||(LL_ConnCtrlQ[tblk_LL_pi->ConnId].QOut!=(MBLK *)0))
                                {
                                    tblk_i |= 0x10;
                                }
                                
                                if((LL_Buffer_To_FIFO_Isr(tblk_LL_pi->ConnId, tblk_i) == (MBLK *)0) && (LL_ConnBuffPt[tblk_LL_pi->ConnId] != (MBLK *)0))
                                {
                                    rafael_reset_phy_fsm_Isr();     
                                    status_LL_Tmr = (0x00 + 44);
                                }
                            }
                        }
                    }
                }
                else
                {
                    mblk = LLWTR_Buffer_nAck_Isr(tblk_LL_pi->ConnId);
                    rafael_reset_phy_fsm_Isr();     
                }
                LL_conn_pi->LE_Conn_Para.WinSize_DataHdr = tblk_i;
                if((j&0x20))
                {
                    if((j&0x03) != 0x03)
                    {
                        if(j&0x40)
                        {
                            LLWTR_TxEndRpt(tblk_LL_pi->ConnId);
                        }
                    }
                    else
                    {
                        if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == 0x04)
                        {
                            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = LL_TX_CONN_ID_INIT[k];
                            LL_conn_pi->LE_Conn_Para.LL_PrToutAccu = 0;
                        }
                        switch(k)
                        {
                        case 0x02:
                            rafael_reset_phy_fsm_Isr();     
                            status_LL_Tmr = (0x00 + 45);
                            LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x16;
                            break;

                        case 0x0B:
                            LL_conn_pi->LE_Conn_Para.LL_SMP_Gate = 2;
                            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterT = 0;
                            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd = 0;

                            mblk = GetMsgBlk_Isr();
                            mblk->Primitive = (0xB0+0x3A);
                            mblk->Para.MLL_LL_Enc_Req_Para.LL_Conn_No = tblk_LL_pi->ConnId;
                            SndMsgBlk_Isr(mblk, 0);
                            break;

                        case 0x03:
                            LL_conn_pi->LE_Conn_Para.LL_SMP_Gate = 1;
                            break;

                        case 0x0C:
                            if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == (0x80|0x0C))
                            {
                                LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                            }
                            break;

                        case 0x00:
                            mblk = mblk_LL_conn_Para[tblk_LL_pi->ConnId];
                            mblk_LL_conn_Para[tblk_LL_pi->ConnId] = (MBLK *)0;
                            SndMsgBlk_Isr(mblk, 4);
                            break;

                        default:
                            break;
                        }
                        if(status_LL_Tmr == (0x00 + 45))
                        {
                            break;
                        }
                    }
                }
                if(tblk_i&0x40)
                {
                    if(HeaderR.HeaderLen)
                    {
                        if((LL_SMP_DataCh&0x02))
                        {
                            
                            if((((SPI_1BYT_SetRx_Isr(155U)&0x10))==0)||((HeaderR.HeaderSts&0x03)==0x00))
                            {
                                rafael_reset_phy_fsm_Isr();     
                                LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x3D;
                                status_LL_Tmr = (0x00 + 45);
                                break;
                            }
                        }
                    }
                    if(LLWTR_Rx2Hc(tblk_LL_pi->ConnId, HeaderR) == (0xB0+0x00))
                    {
                        if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID != (0x80|0x02))
                        {
                            rafael_reset_phy_fsm_Isr();     
                            status_LL_Tmr = (0x00 + 45);
                        }
                    }
                }
                LL_conn_pi->LE_Conn_Para.LL_SvToutAccu = 0;             
                LL_conn_pi->LE_Conn_Para.RF_Rssi = RF_Get_LastRssi();
                break;

            case (0x00 + 44):
                rafael_reset_phy_fsm_Isr();     
                break;

            default:
                break;
            }
        }
        else             

        {

            tblk_i = 1u;
            switch(status_LL_Tmr)
            {
            case (0x00 + 50):   
                HeaderR.HeaderSts = LL_Para_IntervalR.HeaderSts;
                HeaderR.HeaderLen = LL_Para_IntervalR.HeaderLen;
                LL_Slv_Win_Width_Reset();
                tblk_i = 0;    
            case (0x00 + 53):   
                if(LLWTR_MDset(tblk_LL_pi2, (20+1)))  
                {
                    rafael_reset_phy_fsm_Isr();     
                    status_LL_Tmr = (0x00 + 54);   
                    break;
                }
                if(tblk_i)
                {
                    HeaderR.HeaderSts = LL_Para_IntervalR.HeaderSts;
                    HeaderR.HeaderLen = LL_Para_IntervalR.HeaderLen;
                }
                tblk_i = LL_conn_pi->LE_Conn_Para.WinSize_DataHdr&(0x08|0x04);
                if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == 0x02)
                {
                    LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                    LL_conn_pi->LE_Conn_Para.LL_SvToutAccu = 0;
                    LL_conn_pi->LE_Conn_Para.LL_PrToutAccu = 0;
                    tblk_i = 0;
                    LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout = LL_conn_pi->LE_Conn_Para.LL_SvisionTimeoutUpd;    
                }
                RX_CRC_valid_flag = RF_CRCchk();
                if(RX_CRC_valid_flag == (0UL))   
                {
                    j = 0;
                    
                    LL_Para_Interval.HeaderSts = tblk_i;
                    LL_Para_Interval.HeaderLen = 251;
                    RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);
                    
                    mblk = LLWTR_Buffer_nAck_Isr(tblk_LL_pi->ConnId);
                    LL_conn_pi->LE_Conn_Para.WinSize_DataHdr = tblk_i;
                    status_LL_Tmr = (0x00 + 52);   
                    if(LL_ConnDataInTmp.QIn != (MBLK *)0)
                    {
                        FreeMsgBlks_Isr(LL_ConnDataInTmp.QOut);
                        LL_ConnDataInTmp.QOut = (MBLK *)0;
                        LL_ConnDataInTmp.QIn = (MBLK *)0;
                    }
                }
                else     
                {
                    SPI_PDMA_SetRx_Isr(166U, (uint32_t)&rssi_read_data[0], 3);;
                    
                    
                    
                    
                    j = (HeaderR.HeaderSts&(0x08|0x04))>>2;
                    if(LL_ACK_FLOW[(tblk_i|j)]&0x40)
                    {
                        if(HeaderR.HeaderLen)
                        {
                            len = ((SPI_1BYT_SetRx_Isr(106U)));
                            LLWTRFIFOGet_Isr(len);
                            if(LL_ConnDataInTmp.QOut==(MBLK *)0)
                            {
                                HeaderR.HeaderSts &= (~0x10);
                                j = LL_ACK_FLOW_RE_TO[j];
                            }
                        }
                        tblk_i = LL_ACK_FLOW[(tblk_i|j)];
                    }
                    else
                    {
                        tblk_i = LL_ACK_FLOW[(tblk_i|j)];
                    }
                    LL_conn_pi->LE_Conn_Para.WinSize_DataHdr = tblk_i;
                    status_LL_Tmr = (0x00 + 52);  
                    if(HeaderR.HeaderSts&0x10)     
                    {
                        status_LL_Tmr = (0x00 + 51);  
                    }
                    if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == 0x06)
                    {
                        LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                        LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout = LL_conn_pi->LE_Conn_Para.LL_SvisionTimeoutUpd;    
                    }
                    j = 0;
                    
                    LL_Para_Interval.HeaderSts = tblk_i;
                    LL_Para_Interval.HeaderLen = 251;
                    RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);
                    
                    if(tblk_i&0x80)
                    {
                        mblk = LLWTR_Buffer_wAck_Isr(tblk_LL_pi->ConnId);
                        if((LL_ConnBuffPt[tblk_LL_pi->ConnId] != (MBLK *)0)||(mblk != (MBLK *)0))
                        {
                            if(LL_ConnBuffPt[tblk_LL_pi->ConnId] == (MBLK *)0)
                            {
                                j = (mblk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_Conn_Hdl_H)|0x20;
                                k = mblk->Para.HCLL_LE_Ctrl_Pkt_Para.HCI_CtrlPkt_Opcode;
                                
                                FreeMsgBlk_Isr(mblk);
                            }
                            if((LL_SMP_DataCh&0x01)&&(mblk != (MBLK *)0))
                            {
                                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterT++;
                                if(LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterT==0)
                                {
                                    LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd++;
                                    LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd &= (~0x80);
                                }
                            }
                        }
                        mblk = LL_Queue_To_Buffer_Isr(tblk_LL_pi->ConnId);
                    }
                    else
                    {
                        mblk = LLWTR_Buffer_nAck_Isr(tblk_LL_pi->ConnId);
                    }
                    if(tblk_i&0x40)
                    {
                        if(HeaderR.HeaderLen)
                        {
                            if((LL_SMP_DataCh&0x02))
                            {
                                LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterR++;
                                if(LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterR==0)
                                {
                                    LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterRd++;
                                    LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterRd &= (~0x80);
                                }
                            }
                        }
                    }
                }  
                if(mblk != (MBLK *)0)
                {
                    
                    if(RX_CRC_valid_flag != (0UL))   
                    {
                        if((LL_ConnDataQ[tblk_LL_pi->ConnId].QOut!=(MBLK *)0)||(LL_ConnCtrlQ[tblk_LL_pi->ConnId].QOut!=(MBLK *)0))
                        {
                            tblk_i |= 0x10;
                            status_LL_Tmr = (0x00 + 51);   
                        }
                    }
                    if((LL_Buffer_To_FIFO_Isr(tblk_LL_pi->ConnId, tblk_i) == (MBLK *)0) && (LL_ConnBuffPt[tblk_LL_pi->ConnId] != (MBLK *)0))
                    {
                        status_LL_Tmr = (0x00 + 52);   
                    }
                }
                else
                {
                    LL_Para_Interval.HeaderSts = ((tblk_i|0x01)&(~0xE0));
                    LL_Para_Interval.HeaderLen = 0;
                    RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);
                    if(LL_conn_pi->LE_Conn_Para.LL_ConnLatency)
                    {
                        if(LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu < LL_conn_pi->LE_Conn_Para.LL_ConnLatency)
                        {
                            if(tblk_i&0x40)
                            {
                                if((HeaderR.HeaderLen) == 0)
                                {
                                    if((tblk_i&0x80))
                                    {
                                        rafael_reset_phy_fsm_Isr();     
                                        LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu++;         
                                        LL_conn_pi->LE_Conn_Para.LL_SvToutAccu = 0;            
                                        status_LL_Tmr = (0x00 + 54);
                                    }
                                    else
                                    {
                                        if(LL_conn_pi->LE_Conn_Para.LL_SvToutAccu)
                                        {
                                            rafael_reset_phy_fsm_Isr();     
                                            LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu++;     
                                            LL_conn_pi->LE_Conn_Para.LL_SvToutAccu = 0;        
                                            status_LL_Tmr = (0x00 + 54);
                                        }
                                        else
                                        {
                                            LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu = 0;   
                                        }
                                    }
                                }
                                else
                                {
                                    LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu = 0;           
                                }
                            }
                            else
                            {
                                if(LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu)
                                {
                                    rafael_reset_phy_fsm_Isr();     
                                    LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu++;             
                                    LL_conn_pi->LE_Conn_Para.LL_SvToutAccu = 0;                
                                    status_LL_Tmr = (0x00 + 54);
                                }
                            }
                        }
                        else
                        {
                            LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu = 0;                   
                        }
                    }
                }
                if((j&0x20))
                {
                    if((j&0x03) != 0x03)
                    {
                        if(j&0x40)
                        {
                            LLWTR_TxEndRpt(tblk_LL_pi->ConnId);
                        }
                    }
                    else
                    {
                        if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == 0x04)
                        {
                            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = LL_TX_CONN_ID_INIT[k];
                            LL_conn_pi->LE_Conn_Para.LL_PrToutAccu = 0;
                        }
                        switch(k)
                        {
                        case 0x02:
                            rafael_reset_phy_fsm_Isr(); 
                            LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x16;
                            status_LL_Tmr = (0x00 + 55); 
                            break;

                        case 0x06:
                            LL_conn_pi->LE_Conn_Para.LL_SMP_Gate = 0;
                            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                            break;

                        case 0x0B:
                            LL_conn_pi->LE_Conn_Para.LL_SMP_Gate = 2;
                            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterT = 0;
                            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterTd = 0;
                            break;

                        case 0x0C:
                            if(LL_conn_pi->LE_Conn_Para.LL_Conn_ID == (0x80|0x0C))
                            {
                                LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x04;
                            }
                            break;

                        default:
                            break;
                        }
                        if(status_LL_Tmr == (0x00 + 55))     
                        {
                            break;
                        }
                    }
                }
                if(RX_CRC_valid_flag != (0UL))
                {
                    if(tblk_i&0x40)
                    {
                        if(HeaderR.HeaderLen)
                        {
                            if((LL_SMP_DataCh&0x02))
                            {
                                if((((SPI_1BYT_SetRx_Isr(155U)&0x10))==0)||((HeaderR.HeaderSts&0x03)==0x00))
                                {
                                    LL_conn_pi->LE_Conn_Para.ErrCode_DisConn = 0x3D;
                                    status_LL_Tmr = (0x00 + 55);     
                                    rafael_reset_phy_fsm_Isr();     
                                    break;
                                }
                            }
                        }
                        if(LLWTR_Rx2Hc(tblk_LL_pi->ConnId, HeaderR) == (0xB0+0x00))
                        {
                            rafael_reset_phy_fsm_Isr();     
                            status_LL_Tmr = (0x00 + 55);
                        }
                    }
                    LL_conn_pi->LE_Conn_Para.LL_SvToutAccu = 0;
                    LL_conn_pi->LE_Conn_Para.RF_Rssi = RF_Get_LastRssi();
                }
                break;

            case (0x00 + 51):  
                if(LL_conn_pi->LE_Conn_Para.LL_SMP_Gate == 0)     
                {
                    if(LLWTR_MDset(tblk_LL_pi2, (37+1)))
                    {
                        rafael_reset_phy_fsm_Isr();
                        status_LL_Tmr = (0x00 + 54);  
                    }
                    else
                    {
                        status_LL_Tmr = (0x00 + 53);  
                        if((LL_SMP_DataCh&0x02))
                        {
                            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterRd |= 0x80;
                            SPI_PDMA_SetTx(126U, (uint32_t)&LL_conn_pi->LE_Conn_Para . LL_SMP_packetCounterR, 5);
                            SPI_PDMA_waitFinish();
                            LL_conn_pi->LE_Conn_Para.LL_SMP_packetCounterRd &= (~0x80);
                        }
                        RF_CCM_AES_Mode_set((LL_SMP_DataCh&(0x01|0x02)));
                    }
                }
                else
                {
                    rafael_reset_phy_fsm_Isr();
                    status_LL_Tmr = (0x00 + 54);     
                }
                break;

            case (0x00 + 52):   
                rafael_reset_phy_fsm_Isr();        
                status_LL_Tmr = (0x00 + 54);        
                break;

            default:
                rafael_reset_phy_fsm_Isr();     
                break;
            }

        }
    }
    else        
    {
        switch(tblk_i)
        {
        case 4+1:
            switch(status_LL_Tmr)
            {
            case (0x00 + 11):
                status_LL_Tmr = (0x00 + 12);
                break;

            case (0x00 + 12):
                status_LL_Tmr = (0x00 + 15);
                if(RF_CRCchk() == (0UL))
                {
                    rafael_reset_phy_fsm_Isr();     
                    break;
                }
                SPI_PDMA_SetRx_Isr(255U, (uint32_t) &HeaderR.HeaderSts, 2);
                tblk_i = LL_Adv_Para.Adv_Para.LL_Adv_Type& 0x0F;
                mblk = GetMsgBlk_Isr();
                connReqRx = (struct Conn_Req_Para *)mblk->Para.Data;
                mblk->Primitive = (0xB0+0x00);

                SPI_PDMA_SetRx_Isr(255U, (uint32_t)mblk->Para.Data, (1*2+2*4+4*1+6*2+3+5));
                switch((HeaderR.HeaderSts&0x0F))
                {
                case 0x05:
                    rafael_reset_phy_fsm_Isr();     

                    if((tblk_i) == 0x00)
                    {
                        if((LL_Adv_Para.Adv_Para.LL_Adv_Filter_Policy == 0x03)||(LL_Adv_Para.Adv_Para.LL_Adv_Filter_Policy == 0x02))
                        {
                            if(RF_cmpFIFO_WhiteList(HeaderR.HeaderSts, mblk)!=0)
                                break;
                        }
                    }
                    else if((tblk_i) == 0x01)
                    {
                        if((HeaderR.HeaderSts&0x40) == 0)
                        {
                            if(LL_Adv_Para.Adv_Para.LL_Adv_Type&0x80)
                            {
                                break;
                            }
                        }
                        else
                        {
                            if((LL_Adv_Para.Adv_Para.LL_Adv_Type&0x80) == 0)
                            {
                                break;
                            }
                        }
                        if(Knl_MemComp_Isr(connReqRx->IniAddr, LL_Adv_Para.Adv_Para.LL_DirectAddr, 6) != 0)
                        {
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                    if(RF_cmpFIFO_BDAddr(HeaderR.HeaderSts, mblk) == 0)
                    {
                        tblk_i = LL_Adv_Para.Adv_Para.LL_AdvConn_ID;
                        if((HeaderR.HeaderSts&0x40))
                        {
                            mblk_LL_conn_Para[tblk_i]->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddrType = 0x01;
                        }
                        else
                        {
                            mblk_LL_conn_Para[tblk_i]->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddrType = 0x00;
                        }
                        Knl_MemCpy_Isr(&mblk_LL_conn_Para[tblk_i]->Para.MHC_Le_Conn_Complete_Para.HCI_Conn_IntervalL, (Uint8 *)&connReqRx->ConnInterval, 6);
                        mblk_LL_conn_Para[tblk_i]->Para.MHC_Le_Conn_Complete_Para.HCI_Status = 0x00;
                        mblk_LL_conn_Para[tblk_i]->Para.MHC_Le_Conn_Complete_Para.HCI_Master_Clk_Accuracy = connReqRx->mSCA;
                        LL_conn_pi->LE_Conn_Para.LL_SCA = connReqRx->mSCA;
                        Knl_MemCpy_Isr(mblk_LL_conn_Para[tblk_i]->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddr, (Uint8 *)&connReqRx->IniAddr, 6);

                        SndMsgBlk_Isr(mblk_LL_conn_Para[tblk_i], 4);
                        mblk_LL_conn_Para[tblk_i] = (MBLK *)0;
                        if(LL_ConnID_Remaining!=0)
                        {
                            LL_ConnID_Remaining--;
                        }
                        Knl_MemCpy_Isr((Uint8 *)&LL_conn_pi->LE_Conn_Para.LL_AccessAddr, (uint8_t *)&connReqRx->AccessAddr, 21);  

                        Knl_CodeCpy_Isr(LL_conn_pi->LE_Conn_Para.LL_RF_Data_Ch_ReM, LL_RF_DATA_CH, 37);
                        LL_Slv_Win_Width_Reset();

                        LL_conn_pi->LE_Conn_Para.WinSize_DataHdr = connReqRx->trWinSize*10;



                        LL_conn_pi->LE_Conn_Para.LL_HopIncrement = connReqRx->Hop;
                        LL_conn_pi->LE_Conn_Para.LL_CurrentCH = connReqRx->Hop;
                        LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x02;


                        i16 = LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu;
                        LL_conn_pi->LE_Conn_Para.WinOffset_LtcyAccu = (i16*10);

                        i16 = LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout;           
                        LL_conn_pi->LE_Conn_Para.LL_SvisionTimeoutUpd = (i16<<3);   

                        i16 = LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                        LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg = i16;      
                        LL_conn_pi->LE_Conn_Para.LL_SvToutAccu = (i16*4);       
                        LL_conn_pi->LE_Conn_Para.LL_ConnInterval = (i16*10);    
                        LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout = (i16*10);  

#line 8306 "..\\..\\..\\source\\LL.c"

                        if(LL_conn_pi->LE_Conn_Para.LL_ConnInterval==LL_conn_pi->LE_Conn_Para.WinSize_DataHdr+(0x01<<3))
                        {
                            LL_conn_pi->LE_Conn_Para.WinSize_DataHdr = LL_conn_pi->LE_Conn_Para.WinSize_DataHdr-((36+1)+(2*2)+1-(0x01<<3));
                        }
                        LL_Msg_Map_Upd(tblk_i);
                        LL_conn_pi->LE_Conn_Para.LL_Tx_PowerLevel = LL_Adv_Para.Adv_Para.LL_Tx_PowerLevel;           
                        LL_Msg_AdvScnConn &= (~0x10);
                        LL_Msg_AdvScnConnUpdSts &= (~0x01);
                        LL_Msg_AdvScnConnUpd &= (~0x01);
                        status_LL_Tmr = (0x00 + 14);
                    }
                    break;

                case 0x03:
                    switch(tblk_i)
                    {
                    case 0x00:
                    case 0x06:
                        SPI_PDMA_waitFinish();
                        switch(LL_Adv_Para.Adv_Para.LL_Adv_Filter_Policy)
                        {
                        case 0x03:
                        case 0x01:
                            if(RF_cmpFIFO_WhiteList(HeaderR.HeaderSts, mblk)!=0)
                                break;
                        default:
                            if(RF_cmpFIFO_BDAddr(HeaderR.HeaderSts, mblk) == 0)
                            {
                                LL_Para_Interval.HeaderSts = ((LL_Adv_Para.Adv_Para.LL_Adv_Type&~0x0F)|0x04);
                                if((LL_Msg_AdvScnConnUpdSts & 0x10))
                                {
                                    LL_Para_Interval.HeaderLen = LL_Adv_Para_UpdBuf.Adv_Para.LL_ScanRsp_Data_Length+6;
                                    ptAddr = LL_Adv_Para_UpdBuf.Adv_Para.LL_ScanRsp_Data;
                                }
                                else
                                {
                                    LL_Para_Interval.HeaderLen = LL_Adv_Para.Adv_Para.LL_ScanRsp_Data_Length+6;
                                    ptAddr = LL_Adv_Para.Adv_Para.LL_ScanRsp_Data;
                                }
                                RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);
                                if(LL_Adv_Para.Adv_Para.LL_Own_Addr_Type == 0x00)
                                {
                                    RF_TxFIFO_ADVaddr_set(ble_device_param.ble_deviceAddr_param.addr);
                                }
                                else
                                {
                                    RF_TxFIFO_ADVaddr_set(BD_Rand_Addr);
                                }
                                RF_TxFIFO_ADVData_set(ptAddr);
                                status_LL_Tmr = (0x00 + 13);
                            }
                            break;
                        }
                        break;

                    default:
                        break;
                    }
                    break;

                default:
                    break;
                }
                SndMsgBlk_Isr(mblk, 0);
                if(status_LL_Tmr == (0x00 + 15))
                {
                    rafael_reset_phy_fsm_Isr();     
                }
                break;

            case (0x00 + 13):
                rafael_reset_phy_fsm_Isr();         
            case (0x00 + 10):
                status_LL_Tmr = (0x00 + 15);
            default:
                break;
            }
            break;  

        case (4+1+1):
            switch(status_LL_Tmr)
            {
            case (0x00 + 20):    
                status_LL_Tmr = (0x00 + 24);
                if((RF_CRCchk() == (0UL))||(CheckMsgBlk_L2_wSizeUsed_Isr((2*sizeof(mblk->Para.MHC_Le_Adv_Report_Para)))!=0))
                {
                    rafael_reset_phy_fsm_Isr();     
                    break;
                }
                SPI_PDMA_SetRx_Isr(255U, (uint32_t) &HeaderR.HeaderSts, 2);

                mblk = GetMsgBlk_Isr();
                mblk->Primitive = (0xB0+0x00);     

                SPI_PDMA_SetRx_Isr(255U, (uint32_t)mblk->Para.Data, (6+31));

                if(LL_Scan_Para.LE_Set_Scan_Para.LL_ScanFilterPolicy)
                {
                    SPI_PDMA_waitFinish();
                    if(RF_cmpFIFO_WhiteList(HeaderR.HeaderSts, mblk)!=0)
                    {
                        rafael_reset_phy_fsm_Isr();     
                        SndMsgBlk_Isr(mblk, 0);
                        break;
                    }
                }
                tblk_i = 0;
                switch((HeaderR.HeaderSts&0x0F))
                {
                case 0x01:
                case 0x02:
                    rafael_reset_phy_fsm_Isr();     
                    tblk_i++;           
                case 0x00:
                case 0x06:
                    if(tblk_i == 0)
                    {
                        if((HeaderR.HeaderSts&0x40))
                        {
                            tblk_i = 0x80;
                        }
                        if(LL_Scan_Para.LE_Set_Scan_Para.LL_Own_Addr_Type)
                        {
                            tblk_i |= 0x40;
                        }
                        LL_Para_Interval.HeaderSts = (tblk_i|0x03);
                        RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);
                        if(LL_Scan_Para.LE_Set_Scan_Para.LL_Own_Addr_Type == 0x00)
                        {
                            RF_TxFIFO_ADVaddr_set( ble_device_param.ble_deviceAddr_param.addr);
                        }
                        else
                        {
                            RF_TxFIFO_ADVaddr_set(BD_Rand_Addr);
                        }
                        status_LL_Tmr = (0x00 + 22);
                        RF_TxFIFO_ADVData_set(mblk->Para.Data);
                    }
                    LL_Tmr_FIFO_ADVrpt(HeaderR, mblk);         
                    break;

                default:
                    rafael_reset_phy_fsm_Isr();     
                    break;
                }
                SndMsgBlk_Isr(mblk, 0);        
                break;

            case (0x00 + 21):      
                status_LL_Tmr = (0x00 + 24);
                rafael_reset_phy_fsm_Isr();     
                if((RF_CRCchk() == (0UL))||(CheckMsgBlk_L2_wSizeUsed_Isr((2*sizeof(mblk->Para.MHC_Le_Adv_Report_Para)))!=0))
                {
                    
                    break;
                }
                SPI_PDMA_SetRx_Isr(255U, (uint32_t) &HeaderR.HeaderSts, 2);
                mblk = GetMsgBlk_Isr();
                mblk->Primitive = (0xB0+0x00);

                SPI_PDMA_SetRx_Isr(255U, (uint32_t)mblk->Para.Data, (6+31));
                if(LL_Scan_Para.LE_Set_Scan_Para.LL_ScanFilterPolicy)
                {
                    SPI_PDMA_waitFinish();
                    if(RF_cmpFIFO_WhiteList(HeaderR.HeaderSts, mblk)!=0)
                    {
                        SndMsgBlk_Isr(mblk, 0);
                        break;
                    }
                }
                switch((HeaderR.HeaderSts&0x0F))
                {
                case 0x00:
                case 0x01:
                case 0x02:
                case 0x06:
                    LL_Tmr_FIFO_ADVrpt(HeaderR, mblk);
                    break;

                default:
                    break;
                }
                SndMsgBlk_Isr(mblk, 0);
                break;

            case (0x00 + 23):   
                status_LL_Tmr = (0x00 + 24);
                rafael_reset_phy_fsm_Isr();     
                if((RF_CRCchk() == (0UL))||(CheckMsgBlk_L2_wSizeUsed_Isr((2*sizeof(mblk->Para.MHC_Le_Adv_Report_Para)))!=0))
                {
                    
                    break;
                }
                SPI_PDMA_SetRx_Isr(255U, (uint32_t) &HeaderR.HeaderSts, 2);
                mblk = GetMsgBlk_Isr();
                mblk->Primitive = (0xB0+0x00);

                SPI_PDMA_SetRx_Isr(255U, (uint32_t)mblk->Para.Data, (6+31));

                switch((HeaderR.HeaderSts&0x0F))
                {
                case 0x04:
                    LL_Tmr_FIFO_ADVrpt(HeaderR, mblk);
                    break;

                default:
                    break;
                }
                SndMsgBlk_Isr(mblk, 0);
                break;

            case (0x00 + 22):   
                RF_TxAutoAckOff();
                status_LL_Tmr = (0x00 + 23);
                break;

            default:
                break;
            }
            break;   


        case (4+1+2):
            switch(status_LL_Tmr)
            {
            case (0x00 + 30):
                status_LL_Tmr = (0x00 + 33);
                if(RF_CRCchk() == (0UL))
                {
                    rafael_reset_phy_fsm_Isr();     
                    break;
                }
                SPI_PDMA_SetRx_Isr(255U, (uint32_t) &HeaderR.HeaderSts, 2);

                mblk = GetMsgBlk_Isr();
                mblk->Primitive = (0xB0+0x00);
                connReqTx = (struct Conn_Req_Para_Alt *)mblk->Para.Data;

                
                SPI_PDMA_SetRx_Isr(255U, (uint32_t)mblk->Para.Data, (6+6));

                
                j = HeaderR.HeaderSts;
                switch((j&0x0F))
                {
                case 0x00:
                case 0x01:
                    SPI_PDMA_waitFinish();
                    if(LL_Init_Para.LE_Init_Para.LL_InitFilterPolicy)
                    {
                        if(RF_cmpFIFO_WhiteList(HeaderR.HeaderSts, mblk)==0)
                        {
                            
                            if((HeaderR.HeaderSts&0x40))
                            {
                                
                                mblk_LL_conn_Para[LL_Init_Para.LE_Init_Para.LL_Conn_ID]->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddrType = 0x01;
                            }
                            else
                            {
                                mblk_LL_conn_Para[LL_Init_Para.LE_Init_Para.LL_Conn_ID]->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddrType = 0x00;
                            }
                            Knl_MemCpy_Isr(mblk_LL_conn_Para[LL_Init_Para.LE_Init_Para.LL_Conn_ID]->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddr, mblk->Para.Data, 6);
                            tblk_i = 6;
                        }
                    }
                    else
                    {
                        if((j&0x40))
                        {
                            tblk_i = 0x01;
                        }
                        else
                        {
                            tblk_i= 0x00;
                        }
                        if(LL_Init_Para.LE_Init_Para.LL_PeerAddrType == tblk_i)
                        {
                            if(Knl_MemComp_Isr(LL_Init_Para.LE_Init_Para.LL_PeerAddr, mblk->Para.Data, 6)==0)
                            {
                                tblk_i = 6;
                            }
                        }
                    }
                    if(tblk_i == 6)
                    {
                        LL_Msg_AdvScnConn &= (~0x40);
                        LL_Msg_AdvScnConnUpdSts &= (~0x04);
                        LL_Msg_AdvScnConnUpd &= (~0x04);

                        status_LL_Tmr = (0x00 + 31);
                        if((j&0x40))
                        {
                            j = (0x80|0x05);
                        }
                        else
                        {
                            j = 0x05;
                        }
                        if(LL_Init_Para.LE_Init_Para.LL_Own_Addr_Type)
                        {
                            j = j | 0x40;
                        }
                        LL_Para_Interval.HeaderSts = j;
                        RF_LE_HeaderStsLen_Tx(&LL_Para_Interval.HeaderSts);

                        if(LL_Init_Para.LE_Init_Para.LL_Own_Addr_Type == 0x00)
                        {
                            RF_TxFIFO_ADVaddr_set(ble_device_param.ble_deviceAddr_param.addr);
                        }
                        else
                        {
                            RF_TxFIFO_ADVaddr_set(BD_Rand_Addr);
                        }
                        Knl_MemCpy_Isr(&mblk->Para.Data[6], mblk->Para.Data, 6);

                        u32i = ((Timeline24+(5+10))>>1);
                        i16 = ((Uint8 *)&u32i)[3] + ((Uint8 *)&u32i)[2] + ((Uint8 *)&u32i)[1] + ((Uint8 *)&u32i)[0];
                        j = ((Uint8 *)&i16)[0]+((Uint8 *)&i16)[1];
                        k = j>>4;
                        if(j)
                        {
                            if(j > TIMELINE24_3750US_IDX[k])
                            {
                                k = TIMELINE24_3750US_IDX[k+1];
                            }
                            else
                            {
                                k = TIMELINE24_3750US_IDX[k];
                            }
                        }
                        k = k - j;
                        k = (k<<1);

                        
                        if((((Uint8 *)&Timeline24)[0]+(5+10))&0x01)
                        {
                            if(k)
                            {
                                k = k - 1;
                            }
                            else
                            {
                                
                                k = (30-1);
                            }
                        }

                        i16 = k+(5+10);
                        j16 = 0;
                        if(k<10)
                        {
                            j = 0;
                        }
                        else if(k<(10*2))
                        {
                            j = 1;
                        }
                        else
                        {
                            j = 2;
                        }

                        pi16 = &connReqTx->trWinOffset;
                        *pi16 = j;
                        connReqTx->trWinSize = LL_conn_pi->LE_Conn_Para.WinSize_DataHdr+1;
                        LL_conn_pi->LE_Conn_Para.WinSize_DataHdr = 0;       
                        tblk_i = tblk_LL_pi->Next;
                        k = 0;
                        while(1)
                        {
                            if(LL_conn_pi->LE_Conn_Para.LL_SvToutAccu >= LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout)    
                            {
                                break;
                            }
                            if(tblk_i < (2*(4+1+1)))
                            {
                                tblk_LL_pi2 = &TmrBlk_LL[tblk_i];
                                if(tblk_LL_pi2->Ticks>=i16)
                                {
                                    if(tblk_LL_pi2->TmrId < (0x00 + 2))
                                    {
                                        if(i16)
                                        {
                                            LL_DurRxPktAccu = 0;
                                            if(tblk_LL_pi2->Ticks-i16 > (36+1))
                                            {
                                                break;
                                            }
                                        }
                                    }
                                    j16 += (10*3);
                                    if(j16>=LL_conn_pi->LE_Conn_Para.LL_ConnInterval)
                                    {
                                        j16-=LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                        i16 += j16;
                                        LL_conn_pi->LE_Conn_Para.LL_SvToutAccu += LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg;
                                        LL_NextConnCh(LL_conn_pi);
                                        
                                        j = *pi16;
                                        k++;
                                    }
                                    else
                                    {
                                        i16 += (10*3);
                                        j += 3;
                                    }
                                }
                            }
                            else
                            {
                                if(i16==0)
                                {
                                    j16 += (10*3);
                                    if(j16>=LL_conn_pi->LE_Conn_Para.LL_ConnInterval)
                                    {
                                        j16-=LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                                        i16 += j16;
                                        LL_conn_pi->LE_Conn_Para.LL_SvToutAccu += LL_conn_pi->LE_Conn_Para.LL_ConnIntervalOrg;
                                        LL_NextConnCh(LL_conn_pi);
                                        
                                        j = *pi16;
                                        k++;
                                    }
                                    else
                                    {
                                        i16 += (10*3);
                                        j += 3;
                                    }
                                }
                                LL_DurRxPktAccu = 0;
                                k++;
                                break;
                            }
                            tblk_i = tblk_LL_pi2->Next;
                            i16 = i16 - tblk_LL_pi2->Ticks;
                            tblk_LL_pi = tblk_LL_pi2;
                        }     
                        *pi16 = j;
                        if(LL_conn_pi->LE_Conn_Para.LL_SvToutAccu<LL_conn_pi->LE_Conn_Para.LL_SvisionTimeout)
                        {
                            LL_conn_pi->LE_Conn_Para.LL_EventCounter += k;
                            tblk_LL_pi->Next = TBlk_Free_LL;
                            tblk_LL_pi = &TmrBlk_LL[TBlk_Free_LL];
                            tblk_LL_pi->TmrId = (0x00 + 1);
                            tblk_LL_pi->Ticks = i16;
                            tblk_LL_pi2 = &TmrBlk_LL[tblk_LL_pi->Next];
                            TBlk_Free_LL = tblk_LL_pi2->Next;

                            tblk_LL_pi2->Next = tblk_i;
                            tblk_LL_pi2->Ticks = (36+1);
                            tblk_LL_pi2->TmrId = (0x00 + 255);

                            tblk_LL_pi->ConnId = LL_Init_Para.LE_Init_Para.LL_Conn_ID;
                            tblk_LL_pi2->ConnId = LL_Init_Para.LE_Init_Para.LL_Conn_ID;
                            LL_DurRxPktAccu += LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
                        }
                        else
                        {
                            LL_conn_pi->LE_Conn_Para.LL_Conn_ID = 0x01;
                        }
                        LL_TmrBlk_Pt_PairRst();

                        

                        
                        Knl_MemCpy_Isr(&mblk->Para.Data[(6+6)], (Uint8 *)&LL_conn_pi->LE_Conn_Para.LL_AccessAddr, 7);
                        Knl_MemCpy_Isr((Uint8 *)connReqTx->ChMap, (Uint8 *)LL_conn_pi->LE_Conn_Para.LL_ChMapReM, 5);

                        Knl_MemCpy_Isr((Uint8 *)&connReqTx->ConnInterval, (Uint8 *)&LL_Init_Para.LE_Init_Para.LL_ConnInterval, 6);
                        connReqTx->Hop_mSCA = (LL_conn_pi->LE_Conn_Para.LL_HopIncrement|(0<<5));
                        RF_TxFIFO_ADVData_set(&mblk->Para.Data[6]);
                        LL_conn_pi->LE_Conn_Para.LL_Tx_PowerLevel = LL_Init_Para.LE_Init_Para.LL_Tx_PowerLevel;     
                        k = LL_Init_Para.LE_Init_Para.LL_Conn_ID;
                        SndMsgBlk_Isr(mblk_LL_conn_Para[k], 4);
                        mblk_LL_conn_Para[k] = (MBLK *)0;
                        if(LL_ConnID_Remaining!=0)
                        {
                            LL_ConnID_Remaining--;
                        }
                        break;
                    }
                default:  
                    rafael_reset_phy_fsm_Isr();     
                    
                    break;
                }       
                SndMsgBlk_Isr(mblk, 0);
                break;  

            case (0x00 + 31):
                status_LL_Tmr = (0x00 + 32);
                rafael_reset_phy_fsm_Isr();     
                break;

            default:
                break;
            }
            break;  


        default:
            __nop();
            break;
        }
    }
}


void LLHeaderRdy_Isr(void)
{
    

    switch(status_LL_Tmr)
    {
    case (0x00 + 50):
        LL_DurRxPktAccu = LL_conn_pi->LE_Conn_Para.LL_ConnInterval;
        switch(LL_conn_pi->LE_Conn_Para.LL_Rx_PHYS)
        {
        case 0x01:
            LL_DurRxPktAccu -= (1*1);
            break;
        default:
            break;
        }
    case (0x00 + 41):
    case (0x00 + 42):
    case (0x00 + 53):
        LL_ConnDataTmpGate = 1;
        LL_Para_IntervalR.HeaderLen = 0;

        LL_Para_IntervalR.DataIdxF = 0xFF;          
        LL_Para_IntervalR.DataIdxN = 0;             
        if(LL_ConnDataInTmp.QIn != (MBLK *)0)
        {
            FreeMsgBlks_Isr(LL_ConnDataInTmp.QOut);
            LL_ConnDataInTmp.QOut = (MBLK *)0;
            LL_ConnDataInTmp.QIn = (MBLK *)0;
        }
        SPI_PDMA_SetRx_Isr(255U, (uint32_t) &LL_Para_IntervalR.HeaderSts, 2);
        break;

    default:
        break;
    }
}


void LLWTRFIFOGet_Isr(Uint8 len)
{
    Uint8 i, idx, Len2;
    MBLK *mblk;
    MQUEUE *pqueue;

    if(len == 0)
    {
        return;
    }


    pqueue = &LL_ConnDataInTmp;
    if(LL_Para_IntervalR.DataIdxF == 0xFF)
    {
        if(LL_Para_IntervalR.HeaderLen)
        {
            if(LL_Para_IntervalR.DataIdxN==0)
            {
                i = LL_Para_IntervalR.HeaderLen;



                
                mblk = GetMsgBlk_L1_wSize_Isr(LL_Para_IntervalR.HeaderLen);

                pqueue->QOut = mblk;

                if(mblk==(MBLK *)0)
                {
                    LL_Para_IntervalR.DataIdxN = LL_Para_IntervalR.HeaderLen;
                    pqueue->QIn = (MBLK *)0;
                    len = 0;
                }
                else
                {
                    SPI_PDMA_SetRx_Isr(255U, (uint32_t)&LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN], len);

                    while(1)
                    {
                        pqueue->QIn = mblk;
                        mblk->Primitive = (0x60+0x18);
                        mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = 0;
                        mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;

                        if(i)
                        {
                            if(i>((6+31+7)-2-1))
                            {
                                mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = ((6+31+7)-2-1);
                                i -= ((6+31+7)-2-1);
                            }
                            else
                            {
                                mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = i;
                                pqueue->QIn = mblk;
                                mblk->Next = (MBLK *)0;
                                
                            }
                        }
                        if(mblk->Next != (MBLK *)0)
                        {
                            mblk = mblk->Next;
                        }
                        else
                            break;
                    }
                    if((LL_Para_IntervalR.HeaderSts&0x03)==0x01)
                    {
                        LL_ConnBuffPrcsR.BufPrcsF = pqueue->QIn;
                        LL_ConnBuffPrcsR.BufPrcsN = pqueue->QOut;
                    }
                    else
                    {
                        LL_ConnBuffPrcsR.BufPrcsF = pqueue->QOut;
                        if((pqueue->QOut)->Next !=(MBLK *)0)
                        {
                            LL_ConnBuffPrcsR.BufPrcsN = (pqueue->QOut)->Next;
                        }
                        else
                        {
                            LL_ConnBuffPrcsR.BufPrcsN = LL_ConnBuffPrcsR.BufPrcsF;
                        }
                        LL_Para_IntervalR.DataIdxF = 0;
                    }
                }
            }
            else
            {
                if(LL_Para_IntervalR.HeaderLen > LL_Para_IntervalR.DataIdxN)
                {
                    if(len)
                    {
                        if(pqueue->QIn != (MBLK *)0)
                        {
                            SPI_PDMA_SetRx_Isr(255U, (uint32_t)&LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN], len);
                            
                        }
                        else
                        {
                            len = 0;
                        }
                    }
                }
                else
                {
                    len = 0;
                }
            }
        }
        else
        {
            if(LL_Para_IntervalR.DataIdxN==0)
            {
                pqueue->QOut = (MBLK *)0;
                pqueue->QIn = (MBLK *)0;
                len = 0;
            }
        }
    }
    else
    {
        if(LL_Para_IntervalR.HeaderLen > LL_Para_IntervalR.DataIdxN)
        {
            if(len)
            {
                if(pqueue->QIn != (MBLK *)0)
                {
                    SPI_PDMA_SetRx_Isr(255U, (uint32_t)&LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN], len);
                    
                }
                else
                {
                    len = 0;
                }
            }
        }
        else
        {
            len = 0;
        }
    }
    if(len)
    {
        SPI_PDMA_waitFinish();
        Len2 = len;
        while(Len2 != 0)
        {

            if(len > ((6+31+7)-2-1))
            {
                len = (((6+31+7)-2-1)-1);
                Len2 -= (((6+31+7)-2-1)-1);
            }
            else
            {
                len = Len2;
                Len2 -= len;
            }

            if(LL_Para_IntervalR.DataIdxF == 0xFF)
            {
                if(LL_ConnDataInQ[tblk_LL_pi->ConnId].QOut != (MBLK *)0)
                {
                    idx = (LL_ConnDataInQ[tblk_LL_pi->ConnId].QIn)->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL + LL_Para_IntervalR.DataIdxN;

                    if((idx+len)<((6+31+7)-2-1))
                    {
                        Knl_MemCpy_Isr(&LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[idx], &LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN], len);
                    }
                    else
                    {
                        if(idx<((6+31+7)-2-1))
                        {
                            Knl_MemCpy_Isr(&LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[idx], &LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN], (((6+31+7)-2-1)-idx));
                            i = len-(((6+31+7)-2-1)-idx);
                            Knl_MemCpy_Isr(LL_ConnBuffPrcsR.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, &LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN+(((6+31+7)-2-1)-idx)], i);
                            LL_ConnBuffPrcsR.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = i;
                        }
                        else
                        {
                            i = len-(idx-((6+31+7)-2-1));
                            Knl_MemCpy_Isr(LL_ConnBuffPrcsR.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, &LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN], i);
                            LL_ConnBuffPrcsR.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = i;
                        }
                        if(LL_ConnBuffPrcsR.BufPrcsN != (MBLK *)0)
                        {
                            LL_ConnBuffPrcsR.BufPrcsF = LL_ConnBuffPrcsR.BufPrcsN;
                            
                            
                            LL_ConnBuffPrcsR.BufPrcsN = LL_ConnBuffPrcsR.BufPrcsN->Next;
                            
                        }
                        LL_Para_IntervalR.DataIdxF = 0;
                    }
                    LL_Para_IntervalR.DataIdxN += len;
                }
                else
                {
                    FreeMsgBlks_Isr(LL_ConnDataInTmp.QOut);
                    LL_ConnDataInTmp.QOut = (MBLK *)0;
                    LL_ConnDataInTmp.QIn = (MBLK *)0;
                }
            }
            else
            {
                if(LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL != LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0)
                {
                    i = LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL-LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0;
                    idx = LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0;
                    if((idx+len)>((6+31+7)-2-1))
                    {
                        Knl_MemCpy_Isr(&LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[idx], &LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN], i);
                        LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 += i;
                        if(LL_ConnBuffPrcsR.BufPrcsF != LL_ConnBuffPrcsR.BufPrcsN)
                        {
                            Knl_MemCpy_Isr(LL_ConnBuffPrcsR.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, &LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN+i], (len-i));
                            LL_ConnBuffPrcsR.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 += (len-i);
                        }
                    }
                    else
                    {
                        Knl_MemCpy_Isr(&LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[idx], &LL_Para_IntervalR.Data[LL_Para_IntervalR.DataIdxN], len);
                        LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 += len;
                    }
                    if(LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL == LL_ConnBuffPrcsR.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0)
                    {
                        if(LL_ConnBuffPrcsR.BufPrcsN !=(MBLK *)0)
                        {
                            LL_ConnBuffPrcsR.BufPrcsF = LL_ConnBuffPrcsR.BufPrcsN;
                            
                            
                            LL_ConnBuffPrcsR.BufPrcsN = LL_ConnBuffPrcsR.BufPrcsN->Next;
                        }
                    }
                    LL_Para_IntervalR.DataIdxN += len;
                }
            }
        }
    }
}

void LLTimerFIFOGetSend_Isr(void)
{
    uint8_t Len, idx;

    if(LL_ConnDataTmpGate != 0)
    {
        switch(status_LL_Tmr)
        {
        case (0x00 + 40):
        case (0x00 + 43):
        case (0x00 + 51):
        case (0x00 + 52):
            if(LL_Para_Interval.HeaderLen != LL_Para_Interval.DataIdxN)
            {
                if(LL_Para_Interval.HeaderLen > LL_Para_Interval.DataIdxN+32)
                {
                    Len = 32;
                }
                else
                {
                    Len = LL_Para_Interval.HeaderLen - LL_Para_Interval.DataIdxN;
                }

                if(Len)
                {
                    idx = (LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 - LL_Para_Interval.DataIdxF);

                    if(Len > (uint8_t)(idx + (LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL - LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0)))
                    {
                        Knl_MemCpy_Isr(LL_Para_Interval.Data, &LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[LL_Para_Interval.DataIdxF], idx+(LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL - LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0));

                        Len -= idx;
                        if(LL_ConnBuffPrcs.BufPrcsF != LL_ConnBuffPrcs.BufPrcsN)
                        {
                            Knl_MemCpy_Isr(&LL_Para_Interval.Data[(uint8_t)(idx+(LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL - LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0))], LL_ConnBuffPrcs.BufPrcsN->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, Len);
                            LL_Para_Interval.DataIdxF = Len-(LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL-LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0);
                            LL_ConnBuffPrcs.BufPrcsF = LL_ConnBuffPrcs.BufPrcsN;
                            if(LL_ConnBuffPrcs.BufPrcsN->Next !=(MBLK *)0)
                            {
                                LL_ConnBuffPrcs.BufPrcsN = LL_ConnBuffPrcs.BufPrcsN->Next;
                            }
                        }
                        Len += idx;
                    }
                    else
                    {
                        Knl_MemCpy_Isr(LL_Para_Interval.Data, &LL_ConnBuffPrcs.BufPrcsF->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[LL_Para_Interval.DataIdxF], Len);
                        LL_Para_Interval.DataIdxF += Len;
                        if(Len == idx)
                        {
                            LL_Para_Interval.DataIdxF = 0;
                            LL_ConnBuffPrcs.BufPrcsF = LL_ConnBuffPrcs.BufPrcsN;
                            if(LL_ConnBuffPrcs.BufPrcsN->Next !=(MBLK *)0)
                            {
                                LL_ConnBuffPrcs.BufPrcsN = LL_ConnBuffPrcs.BufPrcsN->Next;
                            }
                        }
                    }
                    
                }
                if(Len)
                {
                    RF_TxFIFO_LeData_wIdx_set(LL_Para_Interval.Data, Len);
                    LL_Para_Interval.DataIdxN += Len;
                }
            }
            break;

        case (0x00 + 41):
        case (0x00 + 42):
        case (0x00 + 50):
        case (0x00 + 53):





            while(1)
            {
                if((LL_Para_IntervalR.HeaderLen - LL_Para_IntervalR.DataIdxN) > 16)
                {
                    if(((SPI_1BYT_SetRx_Isr(106U))) >= 16)
                        LLWTRFIFOGet_Isr(16);
                    else
                        break;
                }
                else
                    break;
            }

            break;

        default:
            break;
        }
    }
}



void LL_GPIO_Isr(void)
{
    uint8_t i;
    extern void LLTimer_Isr(void);
    extern void LLWTR_Isr(void);
    extern uint8_t Content_ioInt;
    extern uint8_t Content_ioInt_exceptTmr;





    extern void RF_DC_Rstr_Isr(void);

    
    FT_WakeupFlag = 1;
    RF_DC_Rstr_Isr();


    
    if(ble_device_param.ble_deviceChipId==102u)
    {
        
        while(1)
        {
            i = SPI_1BYT_SetRx_Isr(62U);    
            if((i&(~(0x04|0x10|0x40|0x01)))==0)
                break;
        }
        i&=(0x04|0x10|0x40|0x01);

        __disable_irq();
        if(i)     
        {
            Content_ioInt = i;                      
            if(i&(~0x40))
            {
                Content_ioInt_exceptTmr = (i&(~0x40));
            }
            SPI_1BYT_SetTx_Isr(62U, i);     
        }
        else      
        {
            
            if(Content_ioInt&0x40)                 
            {
                if(Content_ioInt == 0x40)          
                {
                    
                    
                    switch(Content_ioInt_exceptTmr)
                    {
                    case 0x01:
                        i = 0x10;
                        break;

                    case 0x04:
                        i = 0x01;
                        break;

                    case 0x10:
                        i = 0x04;
                        break;

                    default:
                        i = 0;
                        break;
                    }
                }
            }
            else                                            
            {
                i = 0x40;                          
            }
        }
    }
    else  
    {
        i = (SPI_1BYT_SetRx_Isr(62U)&(0x04|0x10|0x40|0x01));    

        __disable_irq();
        SPI_1BYT_SetTx_Isr(62U, i);                     
    }
    __enable_irq();
    if((i&0x40))                           
    {
        __disable_irq();



        LLTimerFIFOGetSend_Isr();
        LLTimer_Isr();

        __enable_irq();
    }
    if((i&0x01))
    {
        __disable_irq();
        LLHeaderRdy_Isr();
        __enable_irq();
    }

    if((i&((0x04|0x10|0x40|0x01)&(~(0x40|0x01)))))        
    {
        if((i&0x04))
        {
            SPI_1BYT_SetTx_Isr(107U, (RFIP_reg_MEM[RFIP_REG_MEM_107] | (0x80|0x40)));;
        }
        __disable_irq();




        Content_ioInt = 0;

        LLWTR_Isr();


        __enable_irq();
    }
}


Uint8 chkBLE_LL_State_Adv(void)
{
    Uint8 i;
    __disable_irq();
    i = LL_Msg_AdvScnConnUpd;
    __enable_irq();

    return (i&(0x01));
}   


Uint8 chkBLE_LL_State_Scan(void)
{
    Uint8 i;
    __disable_irq();
    i = LL_Msg_AdvScnConnUpd;
    __enable_irq();

    return (i&(0x02));
}   


Uint8 chkBLE_LL_State_Init(void)
{
    Uint8 i;
    __disable_irq();
    i = LL_Msg_AdvScnConnUpd;
    __enable_irq();

    return (i&(0x04));
}   


Uint8 setBLE_LL_TxPowerLevel_Adv(Uint8 powerLevel)
{
    Uint8 i;
    __disable_irq();
    i = LL_Msg_AdvScnConnUpd;
    __enable_irq();

    if(i&(0x01))
    {
        return (!0);
    }
    else
    {
        LL_Adv_Para_UpdBuf.Adv_Para.LL_Tx_PowerLevel = powerLevel;
        return 0;
    }
}   


Uint8 setBLE_LL_TxPowerLevel_Scan(Uint8 powerLevel)
{
    Uint8 i;
    __disable_irq();
    i = LL_Msg_AdvScnConnUpd;
    __enable_irq();

    if(i&(0x02))
    {
        return (!0);
    }
    else
    {
        LL_Scan_Para_UpdBuf.LE_Set_Scan_Para.LL_Tx_PowerLevel = powerLevel;
        return 0;
    }
}   


Uint8 setBLE_LL_TxPowerLevel_Init(Uint8 powerLevel)
{
    Uint8 i;
    __disable_irq();
    i = LL_Msg_AdvScnConnUpd;
    __enable_irq();

    if(i&(0x04))
    {
        return (!0);
    }
    else
    {
        LL_Init_Para.LE_Init_Para.LL_Tx_PowerLevel = powerLevel;
        return 0;
    }
}   

#pragma pop

