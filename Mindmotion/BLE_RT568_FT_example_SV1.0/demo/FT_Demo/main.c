/**************************************************************************//**
 * @file     main.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate BLE operation.
 *           Includes the basic initialization and the loop for kernel(BLE) operations.
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "rf_phy.h"
#include "porting_spi.h"
#include "porting_misc.h"
#include "mcu_definition.h"

/*!
   \brief Initial necessary peripheral on MCU.
*/
void SYS_Init(void)
{
    int8_t irqno;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

#if (_BOARD_SELECTION_==_BOARD_NUVOTON_M031TD2AE_QFN33_)   //HIRC
    //GPIO_SetMode(PF, (BIT2|BIT3), GPIO_MODE_INPUT);
    // Enable HIRC
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    // Waiting for HIRC clock ready
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
#elif (_BOARD_SELECTION_==_BOARD_NUVOTON_M031_SIP_)
#if (_USE_MCU_CLK_==MCU_CLK_SOURCE_HXT) //HXT
    GPIO_SetMode(PF, (BIT2|BIT3), GPIO_MODE_INPUT);
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk|CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk|CLK_STATUS_HIRCSTB_Msk);
#else                                   //HIRC   
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
#endif
#else
    //GPIO_SetMode(PF, (BIT2|BIT3), GPIO_MODE_INPUT);
    // Enable HXT
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    // Waiting for HXT clock ready
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
#endif

    /* Set core clock as PLL_CLOCK from PLL */
    //CLK_SetCoreClock(CPU_CLOCK_RATE);  //48MHz for M0, 64MHz for M4
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

#if (_BOARD_SELECTION_==_BOARD_NUVOTON_M487JIDAE_B3_)  //M487
    /* Set both PCLK0 and PCLK1 as HCLK/PCLK_DIV */
#if (PCLK_DIV==4)
    CLK->PCLKDIV = CLK_PCLKDIV_PCLK0DIV4 | CLK_PCLKDIV_PCLK1DIV4;  //96/4=24MHz
#elif (PCLK_DIV==2)
    CLK->PCLKDIV = CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2;  //64/2=32MHz
#elif (PCLK_DIV==1)
    CLK->PCLKDIV = CLK_PCLKDIV_PCLK0DIV1 | CLK_PCLKDIV_PCLK1DIV1;
#endif //(PCLK_DIV==4)

#else //M031 series
    /* Set both PCLK0 and PCLK1 as HCLK/PCLK_DIV */
#if (PCLK_DIV==4)
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV4 | CLK_PCLKDIV_APB1DIV_DIV4;
#elif (PCLK_DIV==2)
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;  //48/2=24MHz
#elif (PCLK_DIV==1)
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;
#endif //(PCLK_DIV==4)

#endif //(_BOARD_SELECTION_==_BOARD_NUVOTON_M487JIDAE_B3_))


    //debug print use UART0
    /* Select HIRC as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select HIRC as the clock source of UART1 */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
#if (_BOARD_SELECTION_==_BOARD_NUVOTON_M031_SIP_)   //Nuvoton SIP EVK
    /* Set PA multi-function pins for UART0 RXD=PA.0 and TXD=PA.1 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk)) |
                    (SYS_GPA_MFPL_PA0MFP_UART0_RXD | SYS_GPA_MFPL_PA1MFP_UART0_TXD);
#else
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |     //It is also VCOM TX/RX port
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
#endif


#if (_BOARD_SELECTION_==_BOARD_NUVOTON_M031TD2AE_QFN33_)   //Nuvoton SIP EVK
    // UART1
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);
#endif

    /* Set only BLE interrupt with the highest priority to mkae sure RF can handle event in time */
    for(irqno = BOD_IRQn; irqno <= RTC_IRQn; irqno++)
    {
        NVIC_SetPriority((IRQn_Type)irqno, 1);
    }

#if ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_)||(_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_))
    NVIC_SetPriority(GPIO_PAPB_IRQn, 0);
    NVIC_SetPriority(GPIO_PAPBPGPH_IRQn, 0);
#else
    NVIC_SetPriority(GPIO_PCPDPEPF_IRQn, 0);
#endif

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
    /* Lock protected registers */
    SYS_LockReg();
}

void RF_Open()
{
#ifndef _TMR_USE_INTERNAL_
    MCU_Timer0Enable();    //new add, Init Timer0
#endif  //(#ifndef _TMR_USE_INTERNAL_)

    /* Wait RF PHY stable */
    CLK_SysTickDelay(25000);

    /* Initialize Gpio reset pin */
    MCU_GpioResetInit();

    /* Do Gpio Reset */
    MCU_GpioReset();
    CLK_SysTickDelay(50000);     //HW 32K clk count 15ms, but need to consider 32K deviation & MCU HIRC deviation

    /* SPI IO remapping */
    RF_SpiIoMapping();

    /* initial SPI PDMA */
    SPI_PDMA_Init();
}

void UART0_Init(void)
{
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0,(UART_INTEN_RDAIEN_Msk|UART_INTEN_RXTOIEN_Msk));
    NVIC_EnableIRQ(UART02_IRQn);
}


void UART1_Init(void)
{
    UART_Open(UART1, 115200);
    UART_EnableInt(UART1,(UART_INTEN_RDAIEN_Msk|UART_INTEN_RXTOIEN_Msk));
    NVIC_EnableIRQ(UART1_IRQn);
}


/*!
   \brief main loop for initialization and BLE kernel
*/
int main(void)
{
    extern BleStackStatus Ble_Kernel_Root(void);
    extern void BleApp_Main(void);
    extern void BleApp_Init(void);

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 - for control test item and debug message */
    UART0_Init();

    /* Init UART0 - for connecting to golden dongle */
    UART1_Init();

    /* Enable the BLE RF PHY */
    RF_Open();

    D_msg("-------------------\n");
    D_msg("  FT Test          \n");
    D_msg("-------------------\n");

    D_msg("Chip_ID=0x%x\n",ChipId_Get());

    if(WDT_GET_RESET_FLAG() == 1)
    {
        WDT_CLEAR_RESET_FLAG();
    }

    BleApp_Init();

    while(1)
    {
        /* Run BLE kernel, the task priority is LL > Host */
        if(Ble_Kernel_Root() == BLESTACK_STATUS_FREE)
        {
            BleApp_Main();
        }
    }
}
