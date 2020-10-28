#ifndef __MCU_DEFINITION_H__
#define __MCU_DEFINITION_H__

#include <stdint.h>
#include "NuMicro.h"

#define _MCU_ARM_CORTEX_   //no actual use, just for InterruptDisable

/* Platform Select */
#define _BOARD_NUVOTON_M031SE_          0   //M031 EVK
#define _BOARD_NUVOTON_M032SE3AE_       1   //M032 EVK 
#define _BOARD_NUVOTON_M487JIDAE_B3_    2   //M487 new EVK
#define _BOARD_NUVOTON_M031TD2AE_QFN33_ 3   //M031 daughter board
#define _BOARD_NUVOTON_M031_SIP_        4   //M031+RF SIP 
#define _BOARD_SELECTION_              _BOARD_NUVOTON_M031TD2AE_QFN33_

//Select MCU clock source
#define MCU_CLK_SOURCE_HXT              0
#define MCU_CLK_SOURCE_HIRC             1
#define _USE_MCU_CLK_                   MCU_CLK_SOURCE_HIRC   //SIP SLT board use HIRC

#if (_BOARD_SELECTION_==_BOARD_NUVOTON_M487JIDAE_B3_)
#define CPU_CLOCK_RATE                 64000000u
#define PCLK_DIV                       2u           //PCLK=CPU_CLOCK_RATE/PCLK_DIV. it is better to choose PCLK=8*N (24MHz or 32MHz) for SPI_clock=8M
#define TIMER_CLOCK_RATE               32000000u    //PCLK 32MHz as source
#else
#define CPU_CLOCK_RATE                 48000000u
#define PCLK_DIV                       2u           //PCLK=CPU_CLOCK_RATE/PCLK_DIV. it is better to choose PCLK=8*N (24MHz or 32MHz) for SPI_clock=8M
#define TIMER_CLOCK_RATE               24000000u    //PCLK 24MHz as source
#endif

/* InterruptEnable */
#ifdef _MCU_ARM_CORTEX_
#define InterruptDisable      __disable_irq
#define InterruptEnable       __enable_irq
#endif  //(#ifdef _MCU_ARM_CORTEX_)


#endif  //__MCU_DEFINITION_H__


