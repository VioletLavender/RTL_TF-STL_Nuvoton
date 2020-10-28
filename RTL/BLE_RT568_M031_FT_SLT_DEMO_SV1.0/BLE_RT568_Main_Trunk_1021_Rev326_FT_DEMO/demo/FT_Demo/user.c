/**************************************************************************//**
 * @file     user.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate how to use LIB pre-build function to start and stop a BLE
 *           connection.
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "BleAppSetting.h"
#include "ft_func.h"
#include "porting_misc.h"
#include "porting_spi.h"
#include "rf_phy.h"

/*******************************************************************************
*   CONSTANT AND DEFINE
*******************************************************************************/
#define FT_RSSITEST_BASE_VALUE               (-32)   // -32dBm,                        can be modified,
#define FT_RSSITEST_RANGE                    20      // RSSITEST_BASE_VALUE +/- 20dBm, can be modified,

// Request
#define APPREQUEST_IDLE                   0x01
#define APPREQUEST_FT_TEST                0x02

// Test Cases
typedef enum bleFT_TestCase
{
    TX_2402_LDO_TEST = 1,
    TX_2402_DCDC_TEST,
    RX_LDO_TEST,
    RX_DCDC_TEST,
    SLEEP_TEST,
    DEEPSLEEP_TEST,
    RSSI_DUTRX_TEST_WITH_SG,
    CLK_16M_TEST
} BleFT_TestCase;


/*******************************************************************************
*   VARIABLES
*******************************************************************************/
uint8_t           appSystemRequest  = APPREQUEST_IDLE;
BleFT_TestCase    bleFtTestCase;

/*******************************************************************************
*   FUNCTIONS
*******************************************************************************/
RT568FT_TestStatus RT568_FTTestSelection(BleFT_TestCase testCase);

void BLEDemo_InitMessage(void)
{
    D_msg("+====================================================================+\n");
    D_msg("Press the number to start related testing.\n");
    D_msg("+====================================================================+\n");
    D_msg("1. TX @2402MHz(LDO) Test\n");
    D_msg("2. TX @2402MHz(DCDC) Test\n");
    D_msg("3. RX(LDO) Test\n");
    D_msg("4. RX(DCDC) Test\n");
    D_msg("5. Sleep Test\n");
    D_msg("6. Deep Sleep Test\n");
    D_msg("7. RSSI Test (DUT RX) with Signal Generator\n");
    D_msg("8. 16MHz CLK Test\n");
    D_msg("+====================================================================+\n");
}


void RT568_Reset(void)
{
    // Disable MCU GPIO interrupt
    MCU_GpioIntDisable();

    // Disable PDMA and SPI module
    PDMA_Close(PDMA);
    SPI_Close(SPI0);

    //Reset pins to GPIO
    // MCU SPI pin initialization
#if ((_BOARD_SELECTION_ == _BOARD_NUVOTON_M031SE_) || (_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_) || (_BOARD_SELECTION_ == _BOARD_NUVOTON_M487JIDAE_B3_) || (_BOARD_SELECTION_ == _BOARD_NUVOTON_M031TD2AE_QFN33_))
    // Setup SPI0 multi-function pins //
    // PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
    // PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA3MFP_Msk |
                                       SYS_GPA_MFPL_PA2MFP_Msk |
                                       SYS_GPA_MFPL_PA1MFP_Msk |
                                       SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA3MFP_GPIO |
                     SYS_GPA_MFPL_PA2MFP_GPIO |
                     SYS_GPA_MFPL_PA1MFP_GPIO |
                     SYS_GPA_MFPL_PA0MFP_GPIO);
#elif (_BOARD_SELECTION_ == _BOARD_NUVOTON_M031_SIP_)    //SIP

    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD3MFP_Msk |
                                       SYS_GPD_MFPL_PD2MFP_Msk |
                                       SYS_GPD_MFPL_PD1MFP_Msk |
                                       SYS_GPD_MFPL_PD0MFP_Msk)) |
                    (SYS_GPD_MFPL_PD3MFP_GPIO   |
                     SYS_GPD_MFPL_PD2MFP_GPIO  |
                     SYS_GPD_MFPL_PD1MFP_GPIO |
                     SYS_GPD_MFPL_PD0MFP_GPIO);

#endif  //_BOARD_SELECTION_      
    CLK_SysTickDelay(10000);

    // Initialize Gpio reset pin
    MCU_GpioResetInit();

    // Do Gpio Reset
    MCU_GpioReset();
    CLK_SysTickDelay(50000);     //HW 32K clk count 15ms, but need to consider 32K deviation & MCU HIRC deviation

    // SPI IO remapping
    RF_SpiIoMapping();

    // Enable MCU GPIO interrupt
    MCU_GpioIntEnable();

    // initial SPI PDMA
    SPI_PDMA_Init();
}


void BleApp_Init(void)
{
    BLEDemo_InitMessage();
}


void BleApp_Main(void)
{
    if((appSystemRequest & APPREQUEST_FT_TEST ) != 0 )
    {
        RT568FT_TestStatus status;

        appSystemRequest &= ~APPREQUEST_FT_TEST;

        status = RT568_FTTestSelection(bleFtTestCase);

        if(status == RT568_TEST_OK)
        {
            D_msg("Result:PASS\n");
        }
        else
        {
            D_msg("Result:FAIL Error Code: 0x%02x\n",status);
        }

        // show test cases init message
        BLEDemo_InitMessage();
    }
}




RT568FT_TestStatus RT568_FTTestSelection(BleFT_TestCase testCase)
{
    RT568FT_TestStatus status;
    uint8_t chipId;

    /* ======================================================== */
    /* Reset RT568 and re-init RF to do continuous FT Test in this demo code
     * If FT test would power-off and power-on before each test case then these commands can be removed.
    */
    MCU_GpioResetInit(); // reset pin will be set to input when in sleep and deep sleep mode.
    RT568_Reset();
    RF_Init();
  
    chipId = ChipId_Get();
    D_msg("Chip_ID=0x%x\n",chipId);
    
    if((chipId != MP_A1) && (chipId != MP_A2))
    {
      return RT568_TEST_ERROR_ID;
    }  
    /* ======================================================== */

    // Start test
    switch(testCase)
    {
    case TX_2402_LDO_TEST:
        status = RT568_TxTest(LDO_MODE);
        D_msg("[RT568_FT TX @2402MHz LDO Test]\n");
        break;

    case TX_2402_DCDC_TEST:
        status = RT568_TxTest(DCDC_MODE);
        D_msg("[RT568_FT TX @2402MHz DCDC Test]\n");
        break;

    case RX_LDO_TEST:
        status = RT568_RxTest(LDO_MODE);
        D_msg("[RT568_FT RX LDO Test]\n");
        break;

    case RX_DCDC_TEST:
        status = RT568_RxTest(DCDC_MODE);
        D_msg("[RT568_FT RX DCDC Test]\n");
        break;

    case SLEEP_TEST:
        status = RT568_SleepTest();
        D_msg("[RT568_FT Sleep Test]\n");
        break;

    case DEEPSLEEP_TEST:
        status = RT568_DeepSleepTest();
        D_msg("[RT568_FT Deep Sleep Test]\n");
        break;

    case RSSI_DUTRX_TEST_WITH_SG:
    {
        signed char rssiResult;
        status = RT568_RssiDUT_RxTest(FT_RSSITEST_BASE_VALUE,FT_RSSITEST_RANGE, &rssiResult);
        D_msg("[RT568_FT RSSI DUT RX Test] RSSI: %d\n",rssiResult);  
    }
      break;

    case CLK_16M_TEST:
        status = RT568_16MCLK_Test();
        D_msg("[RT568_FT 16MHz CLK Test]\n");
        break;

    default:
        return RT568_TEST_ERROR_CMD;
    }

    if(status != RT568_TEST_OK)
    {
        return status;
    }

    return RT568_TEST_OK;
}

/*******************************************************************************
*   UART Handlers
*******************************************************************************/

int BLEDemo_UartRxDataHandler(uint8_t *data, uint8_t dataLen)
{

    if(data[dataLen] == '\r' || data[dataLen] == '\n')
    {
        int testCase = 0;

        sscanf((char *)data,"%d",&testCase);
        bleFtTestCase = (BleFT_TestCase)testCase;

        appSystemRequest |= APPREQUEST_FT_TEST;

        return 0;
    }

    return -1;
}



void UART02_IRQHandler(void) __irq
{
    static uint8_t uartBuffer[218];
    static uint8_t index = 0u;

    uint8_t volatile uartReceiveByte;

    if (UART_GET_INT_FLAG(UART0,UART_INTSTS_RDAINT_Msk))
    {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0))
        {
            /* Get the character from UART Buffer */
            uartReceiveByte = UART_READ(UART0);
            uartBuffer[index] = uartReceiveByte;

            if(BLEDemo_UartRxDataHandler(uartBuffer,index) == 0)
            {
                index = 0;
            }
            else
            {
                index++;
            }
        }
    }
}

