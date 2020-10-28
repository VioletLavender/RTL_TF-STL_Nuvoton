//----------------------------------------------------------------------------//
// This file implement MCU Timer peripherals for Link Layer
//
//----------------------------------------------------------------------------//
#include "porting_LLtimer.h"
#include "rf_phy.h"
#include "NuMicro.h"

//_TICK_BASE_:_TICK_BASE_125P00_  //smallest time resolution is 125us
#if ((_BOARD_SELECTION_==_BOARD_NUVOTON_M031SE_) || (_BOARD_SELECTION_==_BOARD_NUVOTON_M032SE3AE_))   //Timer clock is 32MHz(HXT)
#define BI_TIMER_PRESCALE       200U    //Nuvoton's 8bits limit, <256
#define BI                      20
#elif (_BOARD_SELECTION_==_BOARD_NUVOTON_M031TD2AE_QFN33_)                                            //Timer clock is 48MHz(HIRC, can't run connection if use MCU as LL timer)
#define BI_TIMER_PRESCALE       200U    //Nuvoton's 8bits limit, <256
#define BI                      30      //X =(48M/((1000000/(125))*BI_TIMER_PRESCALE))
#elif (_BOARD_SELECTION_==_BOARD_NUVOTON_M031_SIP_)                                                   //Timer clock is 48MHz(HIRC) or 12MHz(HXT), need to change __HXT define

#if (_USE_MCU_CLK_==MCU_CLK_SOURCE_HXT) //HXT 12M
#define BI_TIMER_PRESCALE       100U    //Nuvoton's 8bits limit, <256
#define BI                      15      //X =(12M/((1000000/(125))*BI_TIMER_PRESCALE))
#else                                   //HIRC 48M
#define BI_TIMER_PRESCALE       200U    //Nuvoton's 8bits limit, <256
#define BI                      30      //X =(48M/((1000000/(125))*BI_TIMER_PRESCALE))
#endif

#elif (_BOARD_SELECTION_==_BOARD_NUVOTON_M487JIDAE_B3_)                                               //Timer clock is 12MHz(HXT)
#define BI_TIMER_PRESCALE       100U    //Nuvoton's 8bits limit, <256
#define BI                      15      //X =(12M/((1000000/(125))*BI_TIMER_PRESCALE)) 
//  = TIMER_CLOCK_RATE/8000/BI_TIMER_PRESCALE
//for 32M, BI=20, BI_TIMER_PRESCALE=200
//for 12M, BI=15, BI_TIMER_PRESCALE=100
#endif  //_BOARD_SELECTION_


/******************************************************************************
 * Public Functions
 ******************************************************************************/

void Tiny_Delay(uint32_t u32Usec)
{
    CLK_SysTickDelay(u32Usec);
}

//Below can be removed at MP_A1 use internal timer

