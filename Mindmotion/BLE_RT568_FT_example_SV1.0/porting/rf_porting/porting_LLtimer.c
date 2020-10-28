//----------------------------------------------------------------------------//
// This file implement MCU Timer peripherals for Link Layer
//
//----------------------------------------------------------------------------//
#include "porting_LLtimer.h"
#include "rf_phy.h"
#include "NuMicro.h"

                                        //Timer clock is 48MHz(HIRC, can't run connection if use MCU as LL timer)
#define BI_TIMER_PRESCALE       200U    //Nuvoton's 8bits limit, <256
#define BI                      30      //X =(48M/((1000000/(125))*BI_TIMER_PRESCALE))
/******************************************************************************
 * Public Functions
 ******************************************************************************/

void delay_us(uint32_t u32Usec)
{
    CLK_SysTickDelay(u32Usec);
}

//Below can be removed at MP_A1 use internal timer

