/*----------------------------------------------------------------------------*/
/* This file implement MCU peripherals like: UART, GPIO, etc for BLE          */
/*----------------------------------------------------------------------------*/
#include "porting_misc.h"
#include "porting_spi.h"
#include "rf_phy.h"
#include "NuMicro.h"
#include "host.h"
#include "ble_cmd.h"
#include "ble_dtm.h"
#ifdef _HCI_HW_    //In normal mode, DO NOT define _HCI_HW_
#include "knl_pblc.h"
#include "hci.h"
#include "LL.h"
#endif


/******************************************************************************
 * Public Functions
 ******************************************************************************/
/*-------------------- GPIO peripheral ---------------------*/
void MCU_GpioResetInit(void)
{
    //Assign RF Reset pin
#if ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_)||(_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_))
    GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M487JIDAE_B3_
    GPIO_SetMode(PC, BIT10, GPIO_MODE_OUTPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031TD2AE_QFN33_
    GPIO_SetMode(PC, BIT1, GPIO_MODE_OUTPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031_SIP_
    GPIO_SetMode(PA, BIT12, GPIO_MODE_OUTPUT);
#endif

    RESET_RF = 1;  //RESET_RF defined in porting_misc.h
}


void MCU_SetGpioResetIdle(void)
{
    //Assign RF Reset pin
#if ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_)||(_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_))
    GPIO_SetMode(PA, BIT4, GPIO_MODE_INPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M487JIDAE_B3_
    GPIO_SetMode(PC, BIT10, GPIO_MODE_INPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031TD2AE_QFN33_
    GPIO_SetMode(PC, BIT1, GPIO_MODE_INPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031_SIP_
    GPIO_SetMode(PA, BIT12, GPIO_MODE_INPUT);
#endif
}

void MCU_GpioReset(void)
{
    //Do Reset: pulse low
    RESET_RF = 1;
    CLK_SysTickDelay(1000);       //1ms

    RESET_RF = 0;
    CLK_SysTickDelay(1000);       //1ms

    RESET_RF = 1;
}

//Assign GPIO INT pin, LED pin
void MCU_GpioPinInit(void)
{
    //Configure Interrupt pin as Input mode
#if ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_)||(_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_))
    GPIO_SetMode(PA, BIT5, GPIO_MODE_INPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M487JIDAE_B3_
    GPIO_SetMode(PC, BIT9, GPIO_MODE_INPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031TD2AE_QFN33_
    GPIO_SetMode(PF, BIT15, GPIO_MODE_INPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031_SIP_
    GPIO_SetMode(PC, BIT2, GPIO_MODE_INPUT);
#endif

    //LED pin assign
#if (_BOARD_SELECTION_ == _BOARD_NUVOTON_M031_SIP_)
    GPIO_SetMode(PF, BIT15, GPIO_MODE_OUTPUT); //LED
    PF15 = 1;                                  //initial off
#else
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT); //LED
    PB14 = 1;                                  //initial off
#endif

//    //Debug Pin
//    GPIO_SetMode(PA, BIT10, GPIO_MODE_OUTPUT);
//    PA10 = 0;

// detector DTM or HRS pin as Input mode
#ifdef BLE_DEMO
#if (BLE_DEMO==DEMO_HRS_DTM)
#if _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031TD2AE_QFN33_
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
#elif _BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031_SIP_
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
#endif
#endif
#endif
}

void MCU_GpioIntEnable(void)
{
#if ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_)||(_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_))
    // Configure PA.5 as Input mode and enable interrupt by rising edge trigger
    GPIO_SetMode(PA, BIT5, GPIO_MODE_INPUT);

    if(ChipId_Get()==MP_A1)
    {
        GPIO_EnableInt(PA, 5, GPIO_INT_RISING);
    }
    else
    {
#if 1
        GPIO_EnableInt(PA, 5, GPIO_INT_HIGH);
#else   //(0)
        GPIO_EnableInt(PA, 5, GPIO_INT_RISING);
#endif  //(0)
    }
    NVIC_EnableIRQ(GPIO_PAPB_IRQn);

#elif (_BOARD_SELECTION_ ==  _BOARD_NUVOTON_M487JIDAE_B3_)
    // Configure PC.9 as Input mode and enable interrupt by rising edge trigger
    GPIO_SetMode(PC, BIT9, GPIO_MODE_INPUT);

    if(ChipId_Get()==MP_A1)
    {
        GPIO_EnableInt(PC, 9, GPIO_INT_RISING);
    }
    else
    {
        GPIO_EnableInt(PC, 9, GPIO_INT_HIGH);
    }
    NVIC_EnableIRQ(GPC_IRQn);

#elif (_BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031TD2AE_QFN33_)
    // Configure PF.15 as Input mode and enable interrupt by rising edge trigger
    GPIO_SetMode(PF, BIT15, GPIO_MODE_INPUT);

    if(ChipId_Get()==MP_A1)
    {
        GPIO_EnableInt(PF, 15, GPIO_INT_RISING);
    }
    else
    {
        GPIO_EnableInt(PF, 15, GPIO_INT_HIGH);
    }
    NVIC_EnableIRQ(GPIO_PCPDPEPF_IRQn);

#elif (_BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031_SIP_)
    // Configure PC.2 as Input mode and enable interrupt by rising edge trigger
    GPIO_SetMode(PC, BIT2, GPIO_MODE_INPUT);

    if(ChipId_Get()== MP_A1)
    {
        GPIO_EnableInt(PC, 2, GPIO_INT_RISING);
    }
    else
    {
        GPIO_EnableInt(PC, 2, GPIO_INT_HIGH);
    }
    NVIC_EnableIRQ(GPIO_PCPDPEPF_IRQn);

#endif //(_BOARD_SELECTION_==_BOARD_NUVOTON_M031SE_)

}


void MCU_GpioIntDisable(void)
{
#if (_BOARD_SELECTION_   == _BOARD_NUVOTON_M031SE_)
    GPIO_DisableInt(PA, 5);
    NVIC_DisableIRQ(GPIO_PAPB_IRQn);    //disable irq in nvic
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_)
    GPIO_DisableInt(PA, 5);
    NVIC_DisableIRQ(GPIO_PAPB_IRQn);    //disable irq in nvic
#elif (_BOARD_SELECTION_ ==  _BOARD_NUVOTON_M487JIDAE_B3_)
    GPIO_DisableInt(PC, 9);
    NVIC_DisableIRQ(GPC_IRQn);          //disable irq in nvic
#elif (_BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031TD2AE_QFN33_)
    GPIO_DisableInt(PF, 15);
    NVIC_DisableIRQ(GPIO_PCPDPEPF_IRQn);          //disable irq in nvic
#elif (_BOARD_SELECTION_ ==  _BOARD_NUVOTON_M031_SIP_)
    GPIO_DisableInt(PC, 2);
    NVIC_DisableIRQ(GPIO_PCPDPEPF_IRQn);          //disable irq in nvic
#endif  //(_BOARD_SELECTION_)
}


/* GPIO Interrupt Handler */
#pragma push
#pragma Otime
#if ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_)||(_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_))
void GPAB_IRQHandler(void)
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M487JIDAE_B3_)
void GPC_IRQHandler(void)
#elif ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031TD2AE_QFN33_)||(_BOARD_SELECTION_ == _BOARD_NUVOTON_M031_SIP_))
void GPCDEF_IRQHandler(void)
#endif
{
    volatile uint32_t temp;
    extern void LL_GPIO_Isr(void);
#ifdef BLE_DEMO
#if ((BLE_DEMO==DEMO_DTM) ||(BLE_DEMO==DEMO_HRS_DTM))
    extern void DTM_Isr(void);
#endif //#if (BLE_DEMO==DEMO_DTM)
#endif //#ifdef BLE_DEMO
    //Clear MCU GPIO Int status
#if ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_)||(_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_))
    if(GPIO_GET_INT_FLAG(PA, BIT5))
    {
        GPIO_CLR_INT_FLAG(PA, BIT5);
    }
    else
    {
        // Un-expected interrupt. Just clear all PA interrupts
        temp = PA->INTSRC;
        PA->INTSRC = temp;
    }
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M487JIDAE_B3_)
    if(GPIO_GET_INT_FLAG(PC, BIT9))
    {
        GPIO_CLR_INT_FLAG(PC, BIT9);
    }
    else
    {
        // Un-expected interrupt. Just clear all PC interrupts
        temp = PC->INTSRC;
        PC->INTSRC = temp;
    }
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M031TD2AE_QFN33_)
    if(GPIO_GET_INT_FLAG(PF, BIT15))
    {
        GPIO_CLR_INT_FLAG(PF, BIT15);
    }
    else
    {
        // Un-expected interrupt. Just clear all PF interrupts
        temp = PF->INTSRC;
        PF->INTSRC = temp;
    }
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M031_SIP_)
    if(GPIO_GET_INT_FLAG(PC, BIT2))
    {
        GPIO_CLR_INT_FLAG(PC, BIT2);
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC interrupts */
        temp = PC->INTSRC;
        PC->INTSRC = temp;
    }
#endif // (_BOARD_SELECTION_ == _BOARD_NUVOTON_xxxx)

#ifdef BLE_DEMO
#if (BLE_DEMO==DEMO_DTM)
    DTM_Isr();
#elif (BLE_DEMO==DEMO_HRS_DTM)
    if(DEMO_GPI_SEL_PIN==1)//DTM mode
    {
        DTM_Isr();
    }
    else
    {
        LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
    }
#else
    LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
#endif
#else
    LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
#endif

}
#pragma pop

/*------------------- SPI GPIO for I/O mapping --------------*/
/**************************************************************************************
 * spiWriteReg
 *
 * Writes to an 8-bit register with the SPI port
 **************************************************************************************/
void spiGpioDelay(void)
{
    CLK_SysTickDelay(1);   //one 1M cycle=1us
}

void spiGpioWriteReg(const unsigned char regAddr, const unsigned char regData)
{

    unsigned char SPICount;                               // Counter used to clock out the data
    unsigned char SPIData;                                // Define a data structure for the SPI data.

    SPI_CS = 1;                                           // Make sure we start with /CS high
    SPI_CK = 0;                                           // and CK low

    SPI_CS = 0;                                           // Set /CS low to start the SPI cycle 25nS
    // Although SPIData could be implemented as an "int", resulting in one
    // loop, the routines run faster when two loops are implemented with
    // SPIData implemented as two "char"s.

    spiGpioDelay();

    //Address 1th byte
    SPIData = regAddr & 0x7F;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                                 // Check for a 1
            SPI_MOSI = 1;                                     // and set the MOSI line appropriately
        else
            SPI_MOSI = 0;

        spiGpioDelay();                                     // delay half clk cycle
        SPI_CK = 1;                                         // Toggle the clock line
        spiGpioDelay();
        SPI_CK = 0;
        SPIData <<= 1;                                      // Rotate to get the next bit
    }                                                     // and loop back to send the next bit
    // Repeat for the Data byte
    //Address 2nd byte
    SPIData = (regAddr & 0x80)>>7;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                                 // Check for a 1
            SPI_MOSI = 1;                                     // and set the MOSI line appropriately
        else
            SPI_MOSI = 0;

        spiGpioDelay();                                     // delay half clk cycle
        SPI_CK = 1;                                         // Toggle the clock line
        spiGpioDelay();
        SPI_CK = 0;
        SPIData <<= 1;                                      // Rotate to get the next bit
    }

    //Data
    SPIData = regData;                                    // Preload the data to be sent with Data
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Data
    {
        if (SPIData & 0x80)
            SPI_MOSI = 1;
        else
            SPI_MOSI = 0;

        spiGpioDelay();
        SPI_CK = 1;
        spiGpioDelay();
        SPI_CK = 0;
        SPIData <<= 1;
    }

    spiGpioDelay();

    SPI_CS = 1;
    SPI_MOSI = 0;
}


void SPI_GPIO_Init(void)
{
    //For SPI I/O remapping, set all GPIO pin as output
#if (_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_)
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT);  //PA5
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_)
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT); //PA5
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M487JIDAE_B3_)
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT9, GPIO_MODE_OUTPUT); //PC9, EVK3
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M031TD2AE_QFN33_)
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PF, BIT15, GPIO_MODE_OUTPUT);  //PF15
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M031_SIP_)
    GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT);  //PC2
#endif
}

//SPI IO mapping. Must do this after Power ON
void RF_SpiIoMapping(void)
{
    //(1) Set all SPI GPIO pin as output. Be careful not to set 5 pins as HIGH, it will trigger HW mapping SPI pin
    SPI_GPIO_Init();         //Here set all CLK,CS,MOSI,NISO,INT as output
    SPI_CS=1;
    SPI_CK=0;
    SPI_MOSI=0;
    SPI_MISO=0;
    DEFAULT_INT=0;

    //(2) Write R248, R249 GPIO select
#if (SPI_IO_ORDER==1)
    //Write R248=8'b10,001,000, R249=8'b0,100,011,0
    //GPIO0[2:0]=0 - CS
    //GPIO1[2:0]=1 - CLK
    //GPIO2[2:0]=2 - MOSI
    //GPIO3[2:0]=3 - MISO
    //GPIO4[2:0]=4 - INT
    CLK_SysTickDelay(50);
    spiGpioWriteReg(248, 0x88);
    CLK_SysTickDelay(50);
    spiGpioWriteReg(249, 0x46);
    CLK_SysTickDelay(50);
#elif (SPI_IO_ORDER==2)
    //Write R248=8'b01,000,100, R249=8'b0,011,010,0
    //GPIO0[2:0]=4 - INT
    //GPIO1[2:0]=0 - CS
    //GPIO2[2:0]=1 - CK
    //GPIO3[2:0]=2 - MOSI
    //GPIO4[2:0]=3 - MISO
    CLK_SysTickDelay(50);
    spiGpioWriteReg(248, 4 | (0<<3) | ((1&0x03)<<6));
    CLK_SysTickDelay(50);
    spiGpioWriteReg(249, ((1&0x04)>>2) | (2<<1) | (3<<4));
    CLK_SysTickDelay(50);
#elif (SPI_IO_ORDER==3)
    //Write R248=8'b00,100,011, R249=8'b0,010,001,0
    //GPIO0[2:0]=3 - MISO
    //GPIO1[2:0]=4 - INT
    //GPIO2[2:0]=0 - CS
    //GPIO3[2:0]=1 - CK
    //GPIO4[2:0]=2 - MOSI
    CLK_SysTickDelay(50);
    spiGpioWriteReg(248, 3 | (4<<3) | ((0&0x03)<<6));
    CLK_SysTickDelay(50);
    spiGpioWriteReg(249, ((0&0x04)>>2) | (1<<1) | (2<<4));
    CLK_SysTickDelay(50);
#elif (SPI_IO_ORDER==4)
    //Write R248=8'b00,100,011, R249=8'b0,010,001,0
    //GPIO0[2:0]=2 - MOSI
    //GPIO1[2:0]=3 - MISO
    //GPIO2[2:0]=4 - INT
    //GPIO3[2:0]=0 - CS
    //GPIO4[2:0]=1 - CK
    CLK_SysTickDelay(50);
    spiGpioWriteReg(248, 2 | (3<<3) | ((4&0x03)<<6));
    CLK_SysTickDelay(50);
    spiGpioWriteReg(249, ((4&0x04)>>2) | (0<<1) | (1<<4));
    CLK_SysTickDelay(50);
#elif (SPI_IO_ORDER==5)
    //Write R248=8'b00,100,011, R249=8'b0,010,001,0
    //GPIO0[2:0]=1 - CK
    //GPIO1[2:0]=2 - MOSI
    //GPIO2[2:0]=3 - MISO
    //GPIO3[2:0]=4 - INT
    //GPIO4[2:0]=0 - CS
    CLK_SysTickDelay(50);
    spiGpioWriteReg(248, 1 | (2<<3) | ((3&0x03)<<6));
    CLK_SysTickDelay(50);
    spiGpioWriteReg(249, ((3&0x04)>>2) | (4<<1) | (0<<4));
    CLK_SysTickDelay(50);
#elif (SPI_IO_ORDER==6)   //for SIP 
    //Write R248=8'b01,000,100, R249=8'b0,010,011,0
    //GPIO0[2:0]=4 - INT
    //GPIO1[2:0]=0 - CS
    //GPIO2[2:0]=1 - CK
    //GPIO3[2:0]=3 - MISO
    //GPIO4[2:0]=2 - MOSI
    CLK_SysTickDelay(50);
    spiGpioWriteReg(248, 4 | (0<<3) | ((1&0x03)<<6));
    CLK_SysTickDelay(50);
    spiGpioWriteReg(249, ((1&0x04)>>2) | (3<<1) | (2<<4));
    CLK_SysTickDelay(50);
#elif (SPI_IO_ORDER==7)   //for daughter board
    //Write R248=8'b10,001,000, R249=8'b0,100,011,0
    //GPIO0[2:0]=0 - CS
    //GPIO1[2:0]=1 - CLK
    //GPIO2[2:0]=3 - MISO
    //GPIO3[2:0]=2 - MOSI
    //GPIO4[2:0]=4 - INT
    CLK_SysTickDelay(50);
    spiGpioWriteReg(248, 0 | (1<<3) | ((3&0x03)<<6));
    CLK_SysTickDelay(50);
    spiGpioWriteReg(249, ((3&0x04)>>2) | (2<<1) | (4<<4));
    CLK_SysTickDelay(50);
#endif
    //(3) Output all pin as HIGH, last 10ms(>1ms). trigger HW take effect
    SPI_GPIO_Init();          //Set all as output
    SPI_CS=1;
    SPI_CK=1;
    SPI_MOSI=1;
    SPI_MISO=1;
    DEFAULT_INT=1;
    CLK_SysTickDelay(10000);        //Delay 10ms
    //CLK_SysTickDelay(25000); //RF POR_Reset, delay 25ms, wait for 16M stable, then start RF_Init. Add the line when using HIRC

    //(4) Init GPIO & SPI
    //MCU_GpioIntEnable(); //configure ext_int GPIO pin. Not enable INT!
    MCU_GpioPinInit();        //Set GPIO interrupt pin as input
    MCU_SpiInit();            //Intial SPI pin, change MISO direction as INPUT
    //CLK_SysTickDelay(100);
    //SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 1000000); //1M clk

#if (SPI_IO_ORDER==1)
    SPI_1BYT_SetTx(249, (0x46 | 0x80));  //set RF MISO, INT as output
#elif (SPI_IO_ORDER==2)
    SPI_1BYT_SetTx(249, ((1&0x04)>>2) | (2<<1) | (3<<4) | (1<<7));  //set RF MISO, INT as output
#elif (SPI_IO_ORDER==3)
    SPI_1BYT_SetTx(249, ((0&0x04)>>2) | (1<<1) | (2<<4) | (1<<7));  //set RF MISO, INT as output
#elif (SPI_IO_ORDER==4)
    SPI_1BYT_SetTx(249, ((4&0x04)>>2) | (0<<1) | (1<<4) | (1<<7));  //set RF MISO, INT as output
#elif (SPI_IO_ORDER==5)
    SPI_1BYT_SetTx(249, ((3&0x04)>>2) | (4<<1) | (0<<4) | (1<<7));  //set RF MISO, INT as output
#elif (SPI_IO_ORDER==6)  //for SIP
    SPI_1BYT_SetTx(249, ((1&0x04)>>2) | (3<<1) | (2<<4) | (1<<7));  //set RF MISO, INT as output
#elif (SPI_IO_ORDER==7)  //for daughter board
    SPI_1BYT_SetTx(249, ((3&0x04)>>2) | (2<<1) | (4<<4) | (1<<7));  //set RF MISO, INT as output
#endif

    //manual control
    SPI_1BYT_SetTx(53, 0xC0); //To gurantee DC/DC power on when set R40=0x90

    //enable LDO
    SPI_1BYT_SetTx(40, 0xC0);
    CLK_SysTickDelay(25000);        //Put delay after LDO_enable, or set register may have strange behavior!

    //enable chip
    SPI_1BYT_SetTx(53, 0x80); //Enable chip
    CLK_SysTickDelay(10000);        //Put delay after chip_enable, or set register may have strange behavior!
}



#ifdef _DEBUG_PINS_
void Debug_Pins_Init(void)
{
    // Configure PF15 as Output mode
    GPIO_SetMode(PF, BIT15, GPIO_MODE_OUTPUT);
    // Configure PA15, PA14 as Output mode
    GPIO_SetMode(PA, (BIT15|BIT14), GPIO_MODE_OUTPUT);
    PF15 = 1;   //initialization, used as debug pin
    PA15 = 1;   //initialization, used as debug pin
    PA14 = 1;   //initialization, used as debug pin
}
#endif  //#ifdef _DEBUG_PINS_

//------------------- CLK function -----------------------//
void _CLK_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

    /* Set chip in idle mode because of WFI command */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for diasble internal analog POR circuit                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Disable_AnalogPORCircuit(void)
{
    SYS->PORDISAN = 0x5AA5;
}

/*************************************************************************
 *
 * System_PowerDown -
 * Description:
 *
 ************************************************************************/
void System_PowerDown(void)
{
    if(BleRFMode_Get() == BLERFMODE_SLEEP)  //RF_sleep
    {
        /* Check if all the debug messages are finished */
        UART_WAIT_TX_EMPTY(UART0);

        /* Unlock protected registers before entering Power-down mode */
        SYS_UnlockReg();

        /* LVR must be enabled and the POR will be enabled automatically */
        SYS_ENABLE_LVR();

        /* Turn off internal analog POR circuit */
        SYS_Disable_AnalogPORCircuit();

        /* Disable Power-on Reset */
        SYS_DISABLE_POR();

        /* Enter to Power-down mode */
        CLK_PowerDown();

        /* Lock protected registers */
        SYS_LockReg();
    }
    else
    {
        CLK_Idle();
    }
}

void UART1_SendData(Uint8* data, Uint8 len)
{
    UART_Write(UART1,data,len);
    UART_WAIT_TX_EMPTY(UART1);
}


#ifdef _HW_PRG_RESET_
void MCU_WDTmr_En(void)
{
    extern void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

    InterruptDisable();
    SYS_UnlockReg();

    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);
    CLK_EnableModuleClock(WDT_MODULE);
    SystemCoreClockUpdate();

    WDT_Open(WDT_TIMEOUT_2POW4, WDT_RESET_DELAY_18CLK, TRUE, FALSE);
    NVIC_EnableIRQ(WDT_IRQn);
    WDT_EnableInt();

    SYS_LockReg();
    InterruptEnable();
}
#endif  //(#ifdef _HW_PRG_RESET_)

//--------------------- UART peripheral --------------------//
#ifdef _HCI_HW_    //In normal mode, DO NOT define _HCI_HW_
#ifdef _HCI_VIRTUAL_

#else
extern uint8_t UARTBufferT[];  //UART buffer for Tx
extern uint8_t UARTBufIdxR;           //UART buffer index for Rx
extern uint8_t UARTBufIdxT;           //UART buffer index for Tx
extern uint8_t UART_Tx_size;
extern uint8_t UART_Rx_size;
extern uint8_t UARTBufValidT;

/* Init UART */
void UART_Init(void)
{

    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 38400);

    /* Enable UART RDA and THRE interrupt */
    //UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk));

    /* Enable UART RDA interrupt, THRE interrupt should enable until the UART Tx start */
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);

    /* Set RX Trigger Level = 1 */
    UART0->FIFO = (UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;

    UARTBufIdxR = 0;                    //initail
    UARTBufIdxT = 0;                    //initail
    UARTBufValidT = 0;                  //initial
}


/* UART_TX */
void setUART_Tx(Uint8 * bufAddr, Uint8 length)
{
    UART_Tx_size = length;
    Knl_MemCpy(UARTBufferT, bufAddr, length);
    //SBUF = UARTBufferT[0];
    UART_WRITE(UART0, UARTBufferT[0]);
    UARTBufValidT = 1;
    UARTBufIdxT = 1;
    UART_ENABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
}

/* UART Interrupt Handler: HCI command decoder */
void UART02_IRQHandler(void)
{

    uint8_t i, ogf, size;
    uint8_t j, idx;
    uint8_t const *ogf_tbl;

    static MBLK *mblk;
    MBLK *mblk_extend;
    static uint8_t *ParaData;

    if (UART_GET_INT_FLAG(UART0,UART_INTSTS_RDAINT_Msk))             //Received data
    {
        //RDAINT Flag Cleared By Read UART_DAT
        if(UARTBufIdxR == 0)
        {
            InterruptDisable();
            mblk = GetMsgBlk_Isr();
            InterruptEnable();
            ParaData = &(mblk->Para.Data[0]);
        }

        if(UARTBufIdxR < (4+1))
        {
            ParaData[UARTBufIdxR] = UART_READ(UART0);
            switch(UARTBufIdxR)                                                         //HCI__005
            {
            case 4:
                if(ParaData[0] == HCI_PKT_COMMAND)
                {
                    UART_Rx_size = ParaData[FLD_HCI_PKT_LTH_COMMAND] + 3;           //2byte OpCode + 1Byte Parameter Length
                }
                else if(ParaData[0] == HCI_PKT_ACL_DATA)
                {
                    UART_Rx_size = ParaData[FLD_HCI_PKT_LTH_ACL_DATA] + 4;          //2byte Handle + 2Byte Data Length
                    if(UARTBufIdxR == 4)
                    {
                        if(ParaData[FLD_HCI_PKT_LTH_ACL_DATA] > SIZE_MBLK_ACL_DATA_UNIT)
                        {
                            mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = SIZE_MBLK_ACL_DATA_UNIT;
                            mblk_extend = GetMsgBlk_L2_wSize(ParaData[FLD_HCI_PKT_LTH_ACL_DATA] - SIZE_MBLK_ACL_DATA_UNIT);
                            mblk->Next = mblk_extend;
                        }
                        else
                        {
                            mblk->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = ParaData[FLD_HCI_PKT_LTH_ACL_DATA];
                        }
                    }
                }
                else                                                                //if(mblk->Para.Data[0] == HCI_PKT_SYNCHRONOUS_DATA)
                {
                    UART_Rx_size = ParaData[FLD_HCI_PKT_LTH_SYNCHRONOUS_DATA] + 3;  //2byte Handle + 1Byte Parameter Length
                }
                break;

            case 3:
                if(ParaData[0] == HCI_PKT_EVENT)
                {
                    UART_Rx_size = ParaData[FLD_HCI_PKT_LTH_EVENT] + 2;              //1byte EventCode + 1Byte Parameter Length
                }
                break;

            default:
                UART_Rx_size = 4;
                break;
            }
        }
        else
        {
            if(ParaData[0] == HCI_PKT_ACL_DATA)
            {
                if(SIZE_MBLK_ACL_DATA_UNIT >= (UARTBufIdxR-4))
                {
                    ParaData[UARTBufIdxR] = UART_READ(UART0);
                }
                else
                {
                    idx = (UARTBufIdxR-4) - SIZE_MBLK_ACL_DATA_UNIT-1;

                    mblk_extend = mblk->Next;
                    while(idx >= SIZE_MBLK_ACL_DATA_UNIT)
                    {
                        if(mblk_extend->Next != (MBLK *)0)
                        {
                            mblk_extend = mblk_extend->Next;
                        }
                        idx -= SIZE_MBLK_ACL_DATA_UNIT;
                    }
                    if(idx == 0)
                    {
                        mblk_extend->Primitive = HCLL_LE_ACL_DATA_PKT_EXTEND;
                        mblk_extend->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 = 0;
                        mblk_extend->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx1 = 0;
                        mblk_extend->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL = 0;
                        mblk_extend->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthH = 0;
                    }
                    mblk_extend->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_LthL += 1;
                    mblk_extend->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_Data_Idx0 += 1;
                    mblk_extend->Para.HCLL_LE_ACL_Data_Pkt_Extend_Para.HCI_DataPkt_Data[idx] = UART_READ(UART0);
                }
            }
            else
            {
                ParaData[UARTBufIdxR] = UART_READ(UART0);
            }
        }

        UARTBufIdxR++;

        if(UARTBufIdxR> UART_Rx_size)
        {
            switch(ParaData[0])
            {
            case HCI_PKT_COMMAND:
                ogf = (ParaData[2]>>2);
                if(ogf <= OGF_HCI_CMD_LE_CONTROLLER)
                {
                    j = ParaData[1];
                    size = NUM_OGF_TABLE[ogf];
                    ogf_tbl = &HCI_OGF_TABLE[ogf][0];

                    for(i=0; i<size; i++)
                    {
                        if((ogf_tbl[i] == j))
                        {
                            if(HCI_LENGTH_CMD_TABLE[ogf][i] != ParaData[3])
                            {
                                mblk->Primitive = LLHC_ERR_CODE_INVALID_LMP_PARAMETERS;
                            }
                            else
                            {
                                mblk->Primitive = HCI_CMD_TABLE[ogf][i];
                            }
                            break;
                        }
                    }
                    if(i == size)
                    {
                        mblk->Primitive = LLHC_ERR_CODE_UNKNOWN_HCI_COMMAND;
                    }
                }
                else
                {
                    mblk->Primitive = LLHC_ERR_CODE_UNKNOWN_HCI_COMMAND;
                }
                break;

            case HCI_PKT_ACL_DATA:
                mblk->Primitive = HCLL_LE_ACL_DATA_PKT;
                break;

            //case HCI_PKT_EVENT:
            //case HCI_PKT_SYNCHRONOUS_DATA:
            default:
                mblk->Primitive = MLL_HCI_NULL;
                break;
            }
            InterruptDisable();
            SndMsgBlks_Isr(mblk, Q_2LL);
            InterruptEnable();
            UARTBufIdxR = 0;    //release for next packet receive
        }
    }
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_THREINT_Msk))
    {
        //THERINT Flag Cleared By Write UART_DAT
        if(UARTBufValidT == 1)                  //UART buffer in use
        {
            if(UARTBufIdxT == UART_Tx_size)
            {
                /* No more data, just stop Tx (Stop work) */
                UART_DISABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
                UARTBufIdxT= 0;         //initail
                UARTBufValidT = 0;      //initial
            }
            else
            {
                UART_WRITE(UART0, UARTBufferT[UARTBufIdxT]);
                UARTBufIdxT++;
            }
        }
    }
}  //end of UART02_IRQHandler()
#endif
#endif //_HCI_HW_

#ifdef _PROFILE_OTA_
void System_Reboot(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    FMC_SET_LDROM_BOOT();
    NVIC_SystemReset();
}

uint32_t  Crc32_Checksum(uint32_t flash_addr, uint32_t size)
{
    uint32_t ChkSum;

    /* Enable CRC clock */
    CLK_EnableModuleClock(CRC_MODULE);

    SYS_UnlockReg();

    FMC_Open();

    ChkSum = FMC_GetChkSum(flash_addr, size);
    FMC_Close();
    SYS_LockReg();

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    return ChkSum;
}
#endif



