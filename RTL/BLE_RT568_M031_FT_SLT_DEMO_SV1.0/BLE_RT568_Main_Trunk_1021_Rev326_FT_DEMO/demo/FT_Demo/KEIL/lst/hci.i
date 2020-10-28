#line 1 "..\\..\\..\\source\\hci.c"












 
#pragma push
#pragma Otime



#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 985 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 20 "..\\..\\..\\source\\hci.c"
#line 1 "..\\..\\..\\include\\rf_phy.h"
 







 




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

#line 21 "..\\..\\..\\source\\hci.c"
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
#line 22 "..\\..\\..\\source\\hci.c"
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




#line 23 "..\\..\\..\\source\\hci.c"
#line 24 "..\\..\\..\\source\\hci.c"
#line 1 "..\\..\\..\\include\\rf_include\\host_client.h"
 







 




#line 15 "..\\..\\..\\include\\rf_include\\host_client.h"






#line 25 "..\\..\\..\\source\\hci.c"
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





#line 26 "..\\..\\..\\source\\hci.c"
#line 1 "..\\..\\..\\include\\rf_include\\L2CAP.h"













 







typedef struct L2CAPBuf
{
    Uint8 DataBuf[251];         
    Uint8 connID_LL;
    Uint8 len_L2CAPBuf;
} L2CAPBuf;
extern L2CAPBuf L2CAP_Buf;
#line 27 "..\\..\\..\\source\\hci.c"


Uint8 Event_Mask[5];      
Uint8 Event_Mask2[1];     
Uint8 LE_Event_Mask[3];   

MHCCallBack mhcCallBack;

#line 48 "..\\..\\..\\source\\hci.c"


const Uint8 EVENT_MASK_INI[]=
{
    (
        0x10 |
        0x80 |
        0
    ),
    (
        0x08 |
        0x80 |
        0
    ),
    (
        0x02 |
        0
    ),
    (
        0x80 |
        0
    ),
    (
        0x20 |
        0
    ),
};


const Uint8 LE_EVENT_MASK_INI[]=
{
    (
        0x01 |
        0x02 |
        0x04 |
        0x08 |
        0x10 |
        
        0x40 |
        
        0
    ),
    (
        
        
        
        0x08 |
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
};


const Uint8 TAB_SUPPORTED_COMMANDS_MASK[64] =
{
    (
        
        
        
        
        
        0x20 |
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        0x80 |
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        0x40 |
        0x80 |
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        0x20 |
        0x40 |
        0x80 |
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        0x08 |
        
        0x20 |
        
        0x80 |
        0
    ),
    (
        
        0x02 |
        
        
        
        0x20 |
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        0x01 |
        0x02 |
        0x04 |
        0x00 |
        0x10 |
        0x20 |
        0x40 |
        0x80 |
        0
    ),
    (
        0x01 |
        0x02 |
        0x04 |
        0x08 |
        0x10 |
        0x20 |
        0x40 |
        0x80 |
        0
    ),
    (
        0x01 |
        0x02 |
        0x04 |
        0x08 |
        0x10 |
        0x20 |
        0x40 |
        0x80 |
        0
    ),
    (
        0x01 |
        0x02 |
        0x04 |
        0x08 |
        0x10 |
        0x20 |
        0x40 |
        0x00 |
        0
    ),
    (
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        
        
        0
    ),
    (
        
        
        
        
        
        
        0
    ),
    0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0
};



const Uint8 TAB_LMP_FEATURES_MASK[] =
{
    (
        
        
        0x04 |
        
        
        
        
        
        0x00
    ),
    (
        0x01 |
        0x02 |
        0x04 |
        0x08 |
        0x10 |
        0x20 |
        
        
        0x00
    ),
    (
        0x01 |
        0x02 |
        0x04 |
        0x08 |
        
        
        
        
        0x00
    ),
    (
        
        
        
        
        
        
        
        0x00
    ),
    (
        0x01 |
        0x02 |
        0x08 |
        0x10 |
        0x20 |
        0x40 |
        
        0x00
    ),
    (
        
        
        0x04 |
        0x08 |
        0x10 |
        
        
        
        0x00
    ),
    (
        0x01 |
        0x02 |
        0x08 |
        0x10 |
        0x20 |
        0x40 |
        0x00
    ),
    (
        0x01 |
        0x02 |
        0x04 |
        0x80 |
        0x00
    ),
};









void BleMHC_PostEvent(BleCmdEvent event, void *parms)
{
    mhcCallBack(event, parms);
}


void initLLCallBack(MHCCallBack callback)
{
    mhcCallBack = callback;
}



void MHC_Event_Null(MBLK *pMBlk)
{
    
}


void MHC_Event_Cmd_Complete(MBLK *pMBlk)
{
    extern void MCU_WDTmr_En(void);

    extern Uint8 smp_Local_Addr_advTemp[2+6];


#line 633 "..\\..\\..\\source\\hci.c"
    extern void initBLEsmp_Re(void);
    if(pMBlk->Para.MHC_Cmd_Complete_Para.By_Primitive == (0xB0+0x04))
    {

        ErrorEntry(13);
        
        

    }



    
    switch(pMBlk->Para.MHC_Cmd_Complete_Para.By_Primitive)
    {
    
    case (0xB0+0x1C):
        BleMHC_PostEvent(0x01,(void *)&pMBlk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0]);

        smp_Local_Addr_advTemp[0] = ble_device_param.ble_deviceAddr_param.addrType;
        Knl_MemCpy(&smp_Local_Addr_advTemp[2], ble_device_param.ble_deviceAddr_param.addr, 6);

        break;

    case (0xB0+0x1E):
        BleMHC_PostEvent(0x0E,0);
        break;

    case (0xB0+0x20):
        BleMHC_PostEvent(0x04,0);



        break;

    case (0xB0+0x3B):
    {





        BleMHC_PostEvent(0x09,(void *)&(pMBlk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1]));



    }
    break;

    case (0xB0+0x13):
    {
        BLE_Event_Rssi_Param rssiParam;
        rssiParam.connID = (Uint8)(pMBlk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2]);
        rssiParam.rssi = (int8_t)(pMBlk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[3]);

        BleMHC_PostEvent(0x0A,(void *)&(rssiParam));



    }
    break;

    case (0xB0+0x2D):

        initBLEsmp_Re();
        pMBlk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[2] = 0x10;
        BleMHC_PostEvent(0x0D,(void *)&pMBlk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[1]);

        break;

    case (0xB0+0x1F):
        BleMHC_PostEvent(0x0F, (void *)&pMBlk->Para.MHC_Cmd_Complete_Para.HCI_EventPara[0]);
        break;

    default:
        break;
    }

}


void MHC_Event_Cmd_Status(MBLK *pMBlk)
{
#line 727 "..\\..\\..\\source\\hci.c"
    switch(pMBlk->Para.MHC_Cmd_Status_Para.By_Primitive)
    {
    case (0xB0+0x1F):
        BleMHC_PostEvent(0x0F, (void *)&pMBlk->Para.MHC_Cmd_Status_Para.HCI_Status);
        break;

    default:
        break;
    }

}


void MHC_Event_Disconn_Complete(MBLK *pMBlk)
{




#line 761 "..\\..\\..\\source\\hci.c"


    extern void initBLEsmp_Re(void);

    extern uint8_t* cmd_FlashBond(uint8_t opcode, uint8_t * para_data);

    initBLEsmp_Re();


    if((Event_Mask[0] & 0x10) != 0)
    {
        __nop();
    }




    BleMHC_PostEvent(0x07,(void *)&(pMBlk->Para.MHC_Disconn_Complete_Para));


    cmd_FlashBond(0x0A, (uint8_t*)smp_Para_Bond_tmp);



}


void MHC_Event_Num_Of_Completed_Pckts(MBLK *pMBlk)
{
#line 804 "..\\..\\..\\source\\hci.c"
    __nop();

}

void MHC_Event_Read_Remote_Ver_Info_Complete(MBLK *pMBlk)
{
#line 825 "..\\..\\..\\source\\hci.c"
    if((Event_Mask[1] & 0x08) != 0)
    {
        __nop();
    }

}

void MHC_Event_Encrypt_Change(MBLK *pMBlk)
{
#line 849 "..\\..\\..\\source\\hci.c"
    Uint8 i;
    MBLK *mblk;

    extern void fil_INFO_FLASHBONDwEXST_PBID(void);
    extern void fil_DATA_FLASHBONDwEXST_PBID(void);

    if(smp_Para_Bond[18]==0x00)
    {
        if(pMBlk->Para.MHC_Encrypt_Change_Para.HCI_Encrpt_En == 1)
        {
            i = smp_PReq_Resp_Key_Distr & rsp_pairing_request_table[10];
            if((i&0x01))
            {
                setBLE_ConnTxData_SMP(pMBlk->Para.MHC_Encrypt_Change_Para.HCI_Conn_Hdl_L, 0x06, (Uint8 *)LTK, 16);
                setBLE_ConnTxData_SMP(pMBlk->Para.MHC_Encrypt_Change_Para.HCI_Conn_Hdl_L, 0x07, (Uint8 *)LTK_SEED, (2 + 8));
                Knl_MemCpy(&smp_Para_Bond[20], LTK, 16);
                Knl_MemCpyInv(&smp_Para_Bond[7], LTK_SEED, 8);
                Knl_MemCpyInv(&smp_Para_Bond[15], &LTK_SEED[8], 2);
                smp_Para_Bond[18] = 0x01;
            }
            if((i&0x02))
            {
                setBLE_ConnTxData_SMP(pMBlk->Para.MHC_Encrypt_Change_Para.HCI_Conn_Hdl_L, 0x08, (Uint8 *)IRK_FIXED, 16);
                setBLE_ConnTxData_SMP(pMBlk->Para.MHC_Encrypt_Change_Para.HCI_Conn_Hdl_L, 0x09, smp_Para_Bond, (1+6));
            }
            if((i&0x04))
            {
                setBLE_ConnTxData_SMP(pMBlk->Para.MHC_Encrypt_Change_Para.HCI_Conn_Hdl_L, 0x0A, (Uint8 *)CSRK_FIXED, 16);
            }
        }
    }
    if(smp_Para_Bond[19] != 0x00)        
    {
        smp_Security_Mode = 0x02;
    }
    else
    {
        smp_Security_Mode = 0x01;
    }
    smp_Encryption = 1;
    smp_Phase = 0;
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x17);
    mblk->Para.MHC_Le_Auth_status_Para.HCI_Conn_Hdl_L = pMBlk->Para.MHC_Encrypt_Change_Para.HCI_Conn_Hdl_L;
    mblk->Para.MHC_Le_Auth_status_Para.HCI_AUTH_STATUS = 0x00;
    SndMsgBlk(mblk, 4);


    fil_INFO_FLASHBONDwEXST_PBID();
    fil_DATA_FLASHBONDwEXST_PBID();



    if((Event_Mask[0] & 0x80) != 0)
    {
        __nop();
    }

}


void MHC_Event_Encrypt_Key_Refresh_Complete(MBLK *pMBlk)
{
#line 928 "..\\..\\..\\source\\hci.c"
    MBLK *mblk;

    extern void fil_INFO_FLASHBONDwEXST_PBID(void);
    fil_INFO_FLASHBONDwEXST_PBID();

    if(smp_Para_Bond[19] != 0x00)        
    {
        smp_Security_Mode = 0x02;
    }
    else
    {
        smp_Security_Mode = 0x01;
    }
    smp_Encryption = 1;
    smp_Phase = 0;
    mblk = GetMsgBlk();
    mblk->Primitive = (0x60+0x17);
    mblk->Para.MHC_Le_Auth_status_Para.HCI_Conn_Hdl_L = pMBlk->Para.MHC_Encrypt_Key_Refresh_Complete_Para.HCI_Conn_Hdl_L;
    mblk->Para.MHC_Le_Auth_status_Para.HCI_AUTH_STATUS = 0x00;
    SndMsgBlk(mblk, 4);


    if((Event_Mask[3] & 0x80) != 0)
    {
        __nop();
    }

}


void MHC_Event_Data_Buffer_Overflow(MBLK *pMBlk)
{
    if((Event_Mask[2] & 0x02) != 0)
    {
    }
    else
    {
    }
}

void MHC_Event_Hardware_Error(MBLK *pMBlk)
{

    if((Event_Mask[1] & 0x80) != 0)
    {
    }
    else
    {
    }
}

void MHC_Event_Le_Conn_Complete(MBLK *pMBlk)
{
    extern Uint8 ATT_INDICATION_Packet_Sent;




    extern Uint32 seedR16;
    extern Uint8 smp_Local_Addr_advTemp[2+6];
    extern Uint8 smp_Local_Addr[4+1][2+6];


#line 1012 "..\\..\\..\\source\\hci.c"

    if((Event_Mask[4] & 0x20) != 0)
    {
        if((LE_Event_Mask[0] & 0x01) != 0)
        {
            __nop();
#line 1028 "..\\..\\..\\source\\hci.c"
            Knl_MemCpy(smp_Para_Bond, &pMBlk->Para.MHC_Le_Conn_Complete_Para.HCI_PeerAddrType, (6+1));    
            Knl_MemCpy(&smp_Local_Addr[pMBlk->Para.MHC_Le_Conn_Complete_Para.HCI_Conn_Hdl_L][0], smp_Local_Addr_advTemp, (6+2));    
            PassEC_BLE_CODE_SMP(pMBlk->Para.MHC_Le_Conn_Complete_Para.HCI_Conn_Hdl_L, (0x50+0x08), (Uint8 *)&seedR16, 4);   

        }
    }
#line 1044 "..\\..\\..\\source\\hci.c"
    initBLE_AttServParamter();
    aTT_MTU[pMBlk->Para.MHC_Le_Conn_Complete_Para.HCI_Conn_Hdl_L] = 23;
    ATT_INDICATION_Packet_Sent = 0;

    BleMHC_PostEvent(0x03,(void *)&(pMBlk->Para.MHC_Le_Conn_Complete_Para));

}


void MHC_Event_Le_Adv_Report(MBLK *pMBlk)
{
#line 1076 "..\\..\\..\\source\\hci.c"
    if((Event_Mask[4] & 0x20) != 0)
    {
        if((LE_Event_Mask[0] & 0x02) != 0)
        {

            BleMHC_PostEvent(0x02,(void *)&(pMBlk->Para.MHC_Le_Adv_Report_Para));
            __nop();
        }
    }

}


void MHC_Event_Le_Conn_Update_Complete(MBLK *pMBlk)
{
#line 1110 "..\\..\\..\\source\\hci.c"
    if((Event_Mask[4] & 0x20) != 0)
    {
        if((LE_Event_Mask[0] & 0x04) != 0)
        {
            __nop();
        }
    }



    pMBlk->Para.MHC_Le_Conn_Update_Complete_Para.HCI_Status = 0x00;
    BleMHC_PostEvent(0x06,(void *)&(pMBlk->Para.MHC_Le_Conn_Update_Complete_Para));


}


void MHC_Event_Le_Read_Remote_Used_Features_Complete(MBLK *pMBlk)
{
#line 1147 "..\\..\\..\\source\\hci.c"
    if((Event_Mask[4] & 0x20) != 0)
    {
        if((LE_Event_Mask[0] & 0x08) != 0)
        {
            __nop();
        }
    }

}


void MHC_Event_Le_Long_Term_Key_Req(MBLK *pMBlk)    
{
#line 1179 "..\\..\\..\\source\\hci.c"
    MBLK    *mblk;
    Uint8 i;
    extern const Uint8 TAB_ZERO_128[];

    Uint8 *k;

    extern uint8_t* cmd_FlashBond(uint8_t opcode, uint8_t * para_data);
    for(i=6+1; i<(36+1); i++)   
    {
        if(smp_Para_Bond[i] != 0)
        {
            break;
        }
    }
    if(i == (36+1))
    {
        k = cmd_FlashBond(0x05, (uint8_t*)smp_Para_Bond_tmp);
        if(*k == 0x00)
        {
            k = cmd_FlashBond(0x06, (uint8_t*)smp_Para_Bond_tmp);
            if(*k == 0x00)
            {
                for(i=0; i<6; i++)
                {
                }
                if(i == 6)
                {
                    while(i < (36+1))
                    {
                        smp_Para_Bond[i] = *(k+3+i);
                        i++;
                    }
                }
            }
        }
    }


    mblk = GetMsgBlk();
    if(smp_Para_Bond[18] == 0x00)
    {
        if(Knl_CodeComp(pMBlk->Para.MHC_Le_Long_Term_Key_Req_Para.HCI_Random_Num, TAB_ZERO_128, (8+2))==0)
        {
            mblk->Primitive = (0xB0+0x2C);
            mblk->Para.HCLL_LE_Long_Term_key_Req_Reply_Para.HCI_Conn_Hdl_L = pMBlk->Para.MHC_Le_Long_Term_Key_Req_Para.HCI_Conn_Hdl_L;
            if(smp_Para_Bond[17] > 16)
            {
                i = 16 - 16;
            }
            else
            {
                i = 16 - smp_Para_Bond[17];
            }
            if(i)
            {
                
                Knl_CodeCpy(smp_Temp16, TAB_ZERO_128, i);
            }
            Knl_MemCpyInv(mblk->Para.HCLL_LE_Long_Term_key_Req_Reply_Para.HCI_LongTermKey, smp_Temp16, 16);
            smp_Encryption = 0;

            cmd_FlashBond(0x01, (uint8_t*)smp_Para_Bond_tmp);

        }
        else
        {
            
            mblk->Primitive = (0xB0+0x2D);
            mblk->Para.HCLL_LE_Long_Term_key_Req_Neg_Reply_Para.HCI_Conn_Hdl_L = pMBlk->Para.MHC_Le_Long_Term_Key_Req_Para.HCI_Conn_Hdl_L;
        }
    }
    else    
    {
        if(Knl_MemComp(pMBlk->Para.MHC_Le_Long_Term_Key_Req_Para.HCI_Random_Num, &smp_Para_Bond[7], (8+2))==0)
        {
            mblk->Primitive = (0xB0+0x2C);
            mblk->Para.HCLL_LE_Long_Term_key_Req_Reply_Para.HCI_Conn_Hdl_L = pMBlk->Para.MHC_Le_Long_Term_Key_Req_Para.HCI_Conn_Hdl_L;
            Knl_MemCpyInv(mblk->Para.HCLL_LE_Long_Term_key_Req_Reply_Para.HCI_LongTermKey, &smp_Para_Bond[20], 16);
            if(smp_Para_Bond[17] > 16)
            {
                i = 16 - 16;
            }
            else
            {
                i = 16 - smp_Para_Bond[17];
            }
            if(i)
            {
                Knl_CodeCpy(mblk->Para.HCLL_LE_Long_Term_key_Req_Reply_Para.HCI_LongTermKey, TAB_ZERO_128, i);
            }
            smp_Encryption = 0;
        }
        else
        {
            
            mblk->Primitive = (0xB0+0x2D);
            mblk->Para.HCLL_LE_Long_Term_key_Req_Neg_Reply_Para.HCI_Conn_Hdl_L = pMBlk->Para.MHC_Le_Long_Term_Key_Req_Para.HCI_Conn_Hdl_L;
        }
    }
    SndMsgBlk(mblk, 0);


    if((Event_Mask[4] & 0x20) != 0)
    {
        if((LE_Event_Mask[0] & 0x10) != 0)
        {
            __nop();
        }
    }

}


void MHC_Event_Le_Remote_Conn_param_Req_Event(MBLK *pMBlk)      
{
    if((Event_Mask[4] & 0x20) != 0)
    {
        if((LE_Event_Mask[0] & 0x20) != 0)
        {
        }
    }
}

void MHC_Event_Authen_Payload_Timeout_Expired(MBLK *pMBlk)      
{
    if((Event_Mask2[0] & 0x80) != 0)
    {
    }
}



void MHC_LE_ACL_Data_Pkt(MBLK *pMBlk)
{
    Uint8 i;
#line 1327 "..\\..\\..\\source\\hci.c"
    MBLK *mblk;
    extern L2CAPBuf L2CAP_Buf;

    if(pMBlk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_DataPkt_Data[(0+2)] < 7)      
    {
        mblk = pMBlk;
        mblk->Primitive = (0x60+0x18);
        L2CAP_Buf.connID_LL = pMBlk->Para.HCLL_LE_ACL_Data_Pkt_Para.HCI_Conn_Hdl_L;
        i = 0;
        while(1)
        {
            if(mblk != (MBLK *)0)
            {
                if(mblk->Primitive == (0x60+0x18))
                {
                    Knl_MemCpy(&L2CAP_Buf.DataBuf[i], mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data, mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL);
                    i += mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL;
                    mblk = mblk->Next;
                }
                else
                    break;
            }
            else
                break;
        }
    }

}


void MHC_Event_HCI_Raw(MBLK *pMBlk)                             
{
#line 1366 "..\\..\\..\\source\\hci.c"
    __nop();




}


void MHC_Event_Le_Data_Length_Change(MBLK *pMBlk)
{
#line 1394 "..\\..\\..\\source\\hci.c"
    if((Event_Mask[4] & 0x20) != 0)
    {
        if((LE_Event_Mask[0] & 0x40) != 0)
        {
            BleMHC_PostEvent(0x11, (void *)&(pMBlk->Para.MHC_Le_Data_Length_Change_Para.HCI_Conn_Hdl_L));
        }
    }

}


void MHC_Event_Le_PHY_Update_Complete(MBLK *pMBlk)
{
#line 1425 "..\\..\\..\\source\\hci.c"
    if((Event_Mask[4] & 0x20) != 0)
    {
        if((LE_Event_Mask[1] & 0x08) != 0)
        {






            
            BleMHC_PostEvent(0x08,(void *)&(pMBlk->Para.MHC_Le_PHY_Update_Complete_Para.HCI_Status));
            



            __nop();
        }
    }

}

void MHC_Event_STK_Gen_Method(MBLK *pMBlk)
{

    BleMHC_PostEvent(0x0B,(void *)&pMBlk->Para.MHC_Le_Stk_Gen_Para.HCI_Conn_Hdl_L);

}

void MHC_Event_Le_Wait_Passkey_Confrim(MBLK *pMBlk)
{

    BleMHC_PostEvent(0x0C,(void *)&pMBlk->Para.MHC_Le_Stk_Gen_Para.HCI_Conn_Hdl_L);

}

void MHC_Evnet_Auth_Status(MBLK *pMBlk)
{

    BleMHC_PostEvent(0x0D,(void *)&pMBlk->Para.MHC_Le_Auth_status_Para.HCI_Conn_Hdl_L);

}

void MHC_LE_ACL_Data_Pkt_Extend(MBLK *pMBlk)
{
    __nop();
}

void (* const HCI_fsm[])(MBLK *) =
{
    MHC_Event_Null,
    MHC_Event_Cmd_Complete,
    MHC_Event_Cmd_Status,
    MHC_Event_Disconn_Complete,
    MHC_Event_Num_Of_Completed_Pckts,
    MHC_Event_Read_Remote_Ver_Info_Complete,
    MHC_Event_Encrypt_Change,
    MHC_Event_Encrypt_Key_Refresh_Complete,
    MHC_Event_Data_Buffer_Overflow,
    MHC_Event_Hardware_Error,
    MHC_Event_Le_Conn_Complete,
    MHC_Event_Le_Adv_Report,
    MHC_Event_Le_Conn_Update_Complete,
    MHC_Event_Le_Read_Remote_Used_Features_Complete,
    MHC_Event_Le_Long_Term_Key_Req,
    MHC_Event_Le_Remote_Conn_param_Req_Event,
    MHC_Event_Authen_Payload_Timeout_Expired,
    MHC_LE_ACL_Data_Pkt,
    MHC_Event_HCI_Raw,
    MHC_Event_Le_Data_Length_Change,
    MHC_Event_Le_PHY_Update_Complete,
    MHC_Event_STK_Gen_Method,
    MHC_Event_Le_Wait_Passkey_Confrim,
    MHC_Evnet_Auth_Status,
    MHC_LE_ACL_Data_Pkt_Extend,
};


void HCI_Root(Uint8 QueueId)
{
    Uint8 event;
    MBLK *pmblk;

#line 1521 "..\\..\\..\\source\\hci.c"
    pmblk = RcvMsgBlk(QueueId);
    event = pmblk->Primitive - 0x60;
    (*(HCI_fsm[event]))(pmblk);
    FreeMsgBlk(pmblk);
}


void initUart(void)
{
#line 1538 "..\\..\\..\\source\\hci.c"
}


void initHCI(void)
{
    Knl_CodeCpy(Event_Mask, EVENT_MASK_INI, 5);
    Event_Mask2[0] = 0x80;
    Knl_CodeCpy(LE_Event_Mask, LE_EVENT_MASK_INI, 3);
}

#line 1709 "..\\..\\..\\source\\hci.c"



#line 1762 "..\\..\\..\\source\\hci.c"




#pragma pop

