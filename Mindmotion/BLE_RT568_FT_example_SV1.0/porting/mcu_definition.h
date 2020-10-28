#ifndef __MCU_DEFINITION_H__
#define __MCU_DEFINITION_H__

#include <stdint.h>
#include "NuMicro.h"

#define _MCU_ARM_CORTEX_   //no actual use, just for InterruptDisable
#define Tiny_Delay(x)    CLK_SysTickDelay(x)

/* Platform Select */
#define _BOARD_NUVOTON_M031TD2AE_QFN33_ 3   //M031 daughter board
#define _BOARD_SELECTION_              _BOARD_NUVOTON_M031TD2AE_QFN33_

//Select MCU clock source
#define MCU_CLK_SOURCE_HXT              0
#define MCU_CLK_SOURCE_HIRC             1
#define _USE_MCU_CLK_                   MCU_CLK_SOURCE_HIRC   //SIP SLT board use HIRC


#define CPU_CLOCK_RATE                 48000000u
#define PCLK_DIV                       2u           //PCLK=CPU_CLOCK_RATE/PCLK_DIV. it is better to choose PCLK=8*N (24MHz or 32MHz) for SPI_clock=8M
#define TIMER_CLOCK_RATE               24000000u    //PCLK 24MHz as source

/* InterruptEnable */
#ifdef _MCU_ARM_CORTEX_
#define InterruptDisable      __disable_irq
#define InterruptEnable       __enable_irq
#endif  //(#ifdef _MCU_ARM_CORTEX_)


#endif  //__MCU_DEFINITION_H__


