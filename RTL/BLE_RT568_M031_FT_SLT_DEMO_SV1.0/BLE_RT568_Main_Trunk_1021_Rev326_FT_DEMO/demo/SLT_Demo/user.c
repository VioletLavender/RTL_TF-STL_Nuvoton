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
#include <string.h>
#include "BleAppSetting.h"
#include "porting_misc.h"
#include "porting_spi.h"
#include "porting_LLtimer.h"
#include "rf_phy.h"
#include "ble_cmd.h"
#include "ble_event.h"

/*******************************************************************************
*   CONSTANT AND DEFINE
*******************************************************************************/
#define SLT_ADDRESS_SELECTED_INDEX           1  //              << can be modified;
#define SLT_TEST_TIMEOUT_S                   2  // 2s           << can be modified;
#define SLT_RSSITEST_RANGE                  20  // +/- 20 dBm   << can be modified;
#define SLT_DUT_READ_RSSI_BASE_VALUE      (-32) //              << can be modified;
#define SLT_DONGLE_READ_RSSI_BASE_VALUE   (-32) //              << can be modified; 


// SLT Test Command String Send to Dongle via UART1
#define TEST_COMMAND_BLEADDR_LEN      (SIZE_BLE_ADDR+7) // length of TEST_COMMAND_BLEADDR = 6, '\n' length = 1
#define TEST_COMMAND_DUMMY            "DUMMY\n"
#define TEST_COMMAND_BLEADDR          "BTADDR"          // will be added ble address after this string, '\n' no needed.
#define TEST_COMMAND_SCANENABLE       "SCANENABLE\n"
#define TEST_COMMAND_SCANDISABLE      "SCANDISABLE\n"
#define TEST_COMMAND_TXTEST           "TXTEST\n"
#define TEST_COMMAND_ENDTEST          "ENDTEST\n"

// SLT Test Command String via BLE
#define TEST_BLERXSTR                 "SLT-TX-TEST"
#define TEST_BLETXSTR                 "SLT-RX-TEST"
#define TEST_BLETXPOWERSTR            "SLT-TXP-TEST"


// Request
#define APPREQUEST_IDLE                   0x01
#define APPREQUEST_SLT_STARTTEST          0x02
#define APPREQUEST_SLT_ADV                0x04
#define APPREQUEST_SLT_CONNECTED          0x08
#define APPREQUEST_SLT_RX_DATA            0x10
#define APPREQUEST_SLT_TX_DATA            0x20
#define APPREQUEST_SLT_RSSI_READ          0x40
#define APPREQUEST_SLT_DISCONNECTED       0x80

//Advertising parameters
#define APP_ADV_INTERVAL_MIN              32U    //value*0.625ms = 32*0.625 = 20ms
#define APP_ADV_INTERVAL_MAX              32U    //value*0.625ms = 32*0.625 = 20ms


//SLT Test Results
typedef uint8_t RT568SLT_TestStatus;
#define RT568_TEST_OK                     0x00   //Command completed. 
#define RT568_TEST_ERROR_CONN             0x01   //Connection failed. 
#define RT568_TEST_ERROR_TX               0x02   //DUT TX test failed. 
#define RT568_TEST_ERROR_RX               0x03   //DUT RX test failed. 
#define RT568_TEST_ERROR_RSSI             0x04   //RSSI value out of range or fail to get RSSI value. 
#define RT568_TEST_ERROR_DISCONN          0x05   //Disconnection failed. 
#define RT568_TEST_ERROR_OTHERS           0x06   //Others. 
#define RT568_TEST_WAITING                0x10   //Waiting.

//SLT Test BLE Address Arrays
// 01:22:33:44:55:11
#define BLEADDR_DEFAULT_TABLE  {  {0x11, 0x55,0x44,0x33, 0x22,  0x01}, \
                                  {0x22, 0x55,0x44,0x33, 0x22,  0x02}, \
                                  {0x33, 0x55,0x44,0x33, 0x22,  0x03}, \
                                  {0x44, 0x55,0x44,0x33, 0x22,  0x04}, \
                                  {0x55, 0x55,0x44,0x33, 0x22,  0x05}, \
                                  {0x66, 0x55,0x44,0x33, 0x22,  0x06}, \
                                  {0x77, 0x55,0x44,0x33, 0x22,  0x07}, \
                                  {0x88, 0x55,0x44,0x33, 0x22,  0x08}, \
                                  {0x99, 0x55,0x44,0x33, 0x22,  0x09}, \
                                  {0xAA, 0x55,0x44,0x33, 0x22,  0x0A}, \
                                  {0xBB, 0x55,0x44,0x33, 0x22,  0x0B}, \
                                  {0xCC, 0x55,0x44,0x33, 0x22,  0x0C}, \
                                  {0xDD, 0x55,0x44,0x33, 0x22,  0x0D}, \
                                  {0xEE, 0x55,0x44,0x33, 0x22,  0x0E}, \
                                  {0xFF, 0x55,0x44,0x33, 0x22,  0x0F}, \
                                  {0x01, 0x55,0x44,0x33, 0x22,  0x10}}

const Uint8 bleDefaultAddrTable[16][SIZE_BLE_ADDR]  = BLEADDR_DEFAULT_TABLE;


// SLT Test Cases
typedef enum bleSLT_TestCase
{
    SLT_IDLE = 1,
    SLT_ADV,
    SLT_CONN,
    SLT_TX,
    SLT_RX,
    SLT_DONGLE_RSSI,
    SLT_DUT_RSSI,
    SLT_DISCONN
} BleSLT_TestCase;


/*******************************************************************************
*   VARIABLES
*******************************************************************************/
uint8_t           appSystemRequest  = APPREQUEST_IDLE;
BleSLT_TestCase   sltTestCase       = SLT_IDLE;
uint8_t           connId            = 0;
Sint8             dongleRssiValue   = 0;
Sint8             dutRssiValue      = 0;

/*******************************************************************************
*   FUNCTIONS Declaration
*******************************************************************************/
static void App_SetBleAdvAddress(uint8_t arg);
static void RT568_SystemTestInit(void);
static RT568SLT_TestStatus RT568_SystemTest(void);
static RT568SLT_TestStatus RT568_SystemTestErrorCode(BleSLT_TestCase testCase);
static uint8_t RT568_RssiTest(int8_t dutReadRssi, int8_t dongleReadRssi);
static void BleEvent_Callback(BleCmdEvent event, void* param);
static BleStackStatus BleDataExcahngeService_SendData(Uint8 length, Uint8 *data);

/*******************************************************************************
*   FUNCTIONS
*******************************************************************************/
void BLEDemo_InitMessage(void)
{
    D_msg("+====================================================================+\n");
    D_msg("Press 1 to start SLT test.\n");
    D_msg("+====================================================================+\n");
}

void Ble_Slave_AdvInit(void)
{
    BLE_Adv_Param advParam;

    uint8_t SET_ADV_DATA[] =
    {
        12, //Uint8 HCI_Adv_Data_Length;
        GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, BLE_GAP_FLAGS_GENERAL_DISCOVERABLE_MODE,      //LE General Discoverable Mode, Bluetooth Spec. Ver4.0 [Vol 3] page 401 of 656
        GAP_AD_TYPE_LENGTH_8, GAP_AD_TYPE_LOCAL_NAME_COMPLETE,
        'S', 'L', 'T', 'D', 'E', 'M', 'O'
    };

    advParam.Adv_Type = ADV_TYPE_ADV_IND;
    advParam.Adv_Interval_Min = APP_ADV_INTERVAL_MIN;
    advParam.Adv_Interval_Max = APP_ADV_INTERVAL_MAX;
    advParam.Adv_Channel_Map = ADV_CHANNEL_ALL;
    advParam.Adv_Filter_Policy = ADV_FILTER_POLICY_ACCEPT_ALL;

    setBLE_AdvParam(advParam);
    setBLE_AdvData((uint8_t *)SET_ADV_DATA, sizeof(SET_ADV_DATA));
}


void Ble_Slave_StartADV(void)
{
    Ble_Slave_AdvInit();
    setBLE_AdvEnable();
}

void BleApp_Init(void)
{
    BLEDemo_InitMessage();

    // register command event callback function
    setBLE_RegisterBleEvent(BleEvent_Callback);

    // wait for UART command or enable the following code to start SLT test when power-on.
//  /* ========================
//   * Start SLT test
//   * ======================== */
//  appSystemRequest |= APPREQUEST_SLT_STARTTEST;
}


void BleApp_Main(void)
{
    RT568SLT_TestStatus status;

    status = RT568_SystemTest();

    if(status == RT568_TEST_WAITING)
    {
        // do nothing
    }
    else
    {
        appSystemRequest  = APPREQUEST_IDLE;
        sltTestCase = SLT_IDLE;

        if(status == RT568_TEST_OK)
        {
            D_msg("Result:PASS\n");
        }
        else
        {
            D_msg("Result:FAIL  Error Code: 0x%02x\n",status);
        }

        BLEDemo_InitMessage();
    }
}



void RT568_SystemTestInit(void)
{
    D_msg("[RT568_SLT Test]\n");

    //Start Timer0 counting 
    TIMER_Start(TIMER0);

    // BLE already initialized

    // init flag
    appSystemRequest  = APPREQUEST_IDLE;
    sltTestCase = SLT_IDLE;

    // wait for dongle and flush garbage data -------------------------
    Tiny_Delay(10000);
    UART1_SendData((Uint8 *)TEST_COMMAND_DUMMY,strlen((char *)TEST_COMMAND_DUMMY));
    // ----------------------------------------------------------------

    /* 1. Set Local BLE Address and transmit BLE address to Golden dongle via UART */
    App_SetBleAdvAddress(SLT_ADDRESS_SELECTED_INDEX);

    /* 2. Send address to golden dongle to start scanning */
    UART1_SendData((Uint8 *)TEST_COMMAND_SCANENABLE,strlen((char *)TEST_COMMAND_SCANENABLE));
}



static RT568SLT_TestStatus RT568_SystemTest(void)
{
    // 1. Test init: set BLE Address
    if((appSystemRequest & APPREQUEST_SLT_STARTTEST ) != 0)
    {
        appSystemRequest &= ~APPREQUEST_SLT_STARTTEST;

        RT568_SystemTestInit();
        appSystemRequest |= APPREQUEST_SLT_ADV; // start advertising
    }

    // 2. Advertising Test
    if((appSystemRequest & APPREQUEST_SLT_ADV ) != 0)
    {
        appSystemRequest &= ~APPREQUEST_SLT_ADV;

        Ble_Slave_StartADV(); // start advertising

        sltTestCase = SLT_CONN;
        //================================================================================================
        // wait for connected with dongle       (BleEvent_Callback)
        //================================================================================================
    }

    // 3. Connection Test
    if((appSystemRequest & APPREQUEST_SLT_CONNECTED ) != 0)
    {
        appSystemRequest &= ~APPREQUEST_SLT_CONNECTED;

        sltTestCase = SLT_RX;
        //================================================================================================
        // wait for RX data from dongle dongle  (BleDataExcahngeService_Callback)
        //================================================================================================
    }

    // 4. DUT RX/TX Test
    if((appSystemRequest & APPREQUEST_SLT_TX_DATA ) != 0)
    {
        appSystemRequest &= ~APPREQUEST_SLT_TX_DATA;

        sltTestCase = SLT_TX;
        if(BleDataExcahngeService_SendData(strlen((char *)TEST_BLETXSTR),(Uint8 *)TEST_BLETXSTR) != BLESTACK_STATUS_SUCCESS)
        {
            return RT568_TEST_ERROR_TX;
        }

        sltTestCase = SLT_DONGLE_RSSI;
    }

    // 5. Dongle RSSI Test
    if((appSystemRequest & APPREQUEST_SLT_RX_DATA ) != 0)
    {
        appSystemRequest &= ~APPREQUEST_SLT_RX_DATA;
        D_msg("dongleRssiValue=%d\n",dongleRssiValue);

        sltTestCase = SLT_DUT_RSSI;

        // get RSSI
        if(getBLE_RssiValue(connId) != BLESTACK_STATUS_SUCCESS)
        {
            return RT568_TEST_ERROR_RSSI;
        }
    }

    // 6. DUT RSSI Test
    if((appSystemRequest & APPREQUEST_SLT_RSSI_READ ) != 0)
    {
        appSystemRequest &= ~APPREQUEST_SLT_RSSI_READ;
        D_msg("dutRssiValue=%d\n",dutRssiValue);

        // check RSSI values
        if(RT568_RssiTest(dutRssiValue,dongleRssiValue) == 0)
        {
            return RT568_TEST_ERROR_RSSI;
        }

        sltTestCase = SLT_DISCONN;
        // disconnect the link
        if(setBLE_Disconnect(connId) != BLESTACK_STATUS_SUCCESS)
        {
            return RT568_TEST_ERROR_DISCONN;
        }
    }

    // 7. Disconnect Test (Final test)
    if((appSystemRequest & APPREQUEST_SLT_DISCONNECTED ) != 0)
    {
        appSystemRequest &= ~APPREQUEST_SLT_DISCONNECTED;

        // stop TIMER0 counter
        TIMER_Stop(TIMER0);

        if(sltTestCase == SLT_DISCONN)
        {
            return RT568_TEST_OK;
        }
        else
        {
            // due to timeout
            return RT568_SystemTestErrorCode(sltTestCase);
        }
    }

    return RT568_TEST_WAITING;
}

static RT568SLT_TestStatus RT568_SystemTestErrorCode(BleSLT_TestCase testCase)
{
    switch(testCase)
    {
    case SLT_CONN:
        return RT568_TEST_ERROR_CONN;
    case SLT_TX:
        return RT568_TEST_ERROR_TX;
    case SLT_RX:
        return RT568_TEST_ERROR_RX;
    case SLT_DONGLE_RSSI:
    case SLT_DUT_RSSI:
        return RT568_TEST_ERROR_RSSI;
    case SLT_DISCONN:
        return RT568_TEST_ERROR_DISCONN;

    default:
        break;
    }

    return RT568_TEST_ERROR_OTHERS;
}

static void App_SetBleAdvAddress(uint8_t arg)
{
    Uint8 selectedAddrIndex;
    Uint8 bleAddrSetCmd[TEST_COMMAND_BLEADDR_LEN];
    BLE_Addr_Param addrParam;

    // 2. Set BLE Address, transmit BLE address to Golden dongle via UART
    selectedAddrIndex = (arg & 0x0F);
    addrParam.addrType = PUBLIC_ADDR;
    memcpy(addrParam.addr,bleDefaultAddrTable[selectedAddrIndex],SIZE_BLE_ADDR);
    setBLE_BleDeviceAddr(&addrParam);
    D_msg("selectedAddrIndex = %d \n",selectedAddrIndex);

    // Send slected peer BLE address to golden
    // CMD = "BTADDRXXXXXXXXXXXX"
    memcpy(bleAddrSetCmd,(Uint8 *)TEST_COMMAND_BLEADDR,strlen((char *)TEST_COMMAND_BLEADDR));
    memcpy(bleAddrSetCmd+strlen((char *)TEST_COMMAND_BLEADDR),addrParam.addr,SIZE_BLE_ADDR);
    bleAddrSetCmd[TEST_COMMAND_BLEADDR_LEN - 1] = '\n';
    UART1_SendData(bleAddrSetCmd,TEST_COMMAND_BLEADDR_LEN);
}


static uint8_t RT568_RssiTest(int8_t dutReadRssi, int8_t dongleReadRssi)
{
    // compare
    if((dutReadRssi >= (SLT_DUT_READ_RSSI_BASE_VALUE + SLT_RSSITEST_RANGE)) || (dutReadRssi <= (SLT_DUT_READ_RSSI_BASE_VALUE - SLT_RSSITEST_RANGE)) ||
      ((dongleReadRssi >= (SLT_DONGLE_READ_RSSI_BASE_VALUE + SLT_RSSITEST_RANGE)) || (dongleReadRssi <= (SLT_DONGLE_READ_RSSI_BASE_VALUE - SLT_RSSITEST_RANGE))))
    {
        // Error rssi value
        return 0;
    }

    return 1;
}


/*******************************************************************************
*   Data Exchange and GAP Handlers
*******************************************************************************/

/* DataExchange Service Callback Function */
static BleStackStatus BleDataExcahngeService_SendData(Uint8 length, Uint8 *data)
{
    BleStackStatus status;

    memcpy(att_HDL_DATAEXCHANGES_READINIT,data,length);

    if(att_HDL_DATAEXCHANGES_READINIT_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION[0] == GATT_DESCRIPTORS_CLIENT_CHARACTERISTIC_CONFIGURATION_INDICATION)
    {
        status = ATT_HDL_Indicate(connId,(Uint8 *)ATT_HDL_DATAEXCHANGES_READINIT, att_HDL_DATAEXCHANGES_READINIT_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION, att_HDL_DATAEXCHANGES_READINIT, length);
    }
    else
    {
        status = ATT_HDL_Notify(connId,(Uint8 *)ATT_HDL_DATAEXCHANGES_READINIT, att_HDL_DATAEXCHANGES_READINIT_MEASUREMENT_CLIENT_CHARACTERISTIC_CONFIGURATION, att_HDL_DATAEXCHANGES_READINIT, length);
    }

    return status;
}

void BleDataExcahngeService_Callback(Uint8 length, Uint8 *data)
{
    uint8_t txpowerStrLen = strlen((char *)TEST_BLETXPOWERSTR);

    if(memcmp((Uint8 *)TEST_BLERXSTR,data,length) == 0) // 1st data recieved from dongle
    {
        appSystemRequest |= APPREQUEST_SLT_TX_DATA;
    }

    if((length == (txpowerStrLen+1)) && ((memcmp((Uint8 *)TEST_BLETXPOWERSTR,data,txpowerStrLen)) == 0)) // "SLT-TXP-TEST" + 1 Byte (positive RSSI Value)
    {
        appSystemRequest |= APPREQUEST_SLT_RX_DATA;
        dongleRssiValue = ((Sint8)data[txpowerStrLen]);
    }
}

/* GAP Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void* param)
{
    //struct MHC_Disconn_Complete_Para *disParam = param;
    switch(event)
    {
    case BLECMD_EVENT_ADV_COMPLETE:
    {
    }
    break;

    case BLECMD_EVENT_CONN_COMPLETE:
    {
        BLE_Event_ConnParam *connparam = (BLE_Event_ConnParam *)param;
        connId = connparam->connId;
        appSystemRequest |= APPREQUEST_SLT_CONNECTED;

        D_msg("Connected\n");
    }
    break;
    case BLECMD_EVENT_DISCONN_COMPLETE:
    {
        appSystemRequest |= APPREQUEST_SLT_DISCONNECTED;

        D_msg("Disconnected\n");
    }
    break;


    case BLECMD_EVENT_READ_RSSI_COMPLETE:
    {
        BLE_Event_Rssi_Param *rssiParam = (BLE_Event_Rssi_Param *)param;
        dutRssiValue = rssiParam->rssi;
        appSystemRequest |= APPREQUEST_SLT_RSSI_READ;
    }
    break;


    default:
        break;
    }
}


/*******************************************************************************
*   TIMER0 Handlers
*******************************************************************************/
void TMR0_IRQHandler(void)
{
    static int waitCount = 0;

    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        waitCount++;
        if(waitCount > SLT_TEST_TIMEOUT_S)
        {
            // timeout
            waitCount = 0;

            setBLE_Disconnect(connId);
        }
    }
}



/*******************************************************************************
*   UART Handlers
*******************************************************************************/

int BLEDemo_UartRxDataHandler(uint8_t *data, uint8_t dataLen)
{

    if(data[dataLen] == '\r' || data[dataLen] == '\n')
    {
        int cmd = 0;

        sscanf((char *)data,"%d",&cmd);

        if((cmd == 1) && (sltTestCase == SLT_IDLE))
        {
            /* ========================
             * Start SLT test
             * ======================== */
            appSystemRequest |= APPREQUEST_SLT_STARTTEST;
        }
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
