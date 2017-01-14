#include <string.h>
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf51_bitfields.h"
#include "nrf_soc.h"
#include "comreloaded.h"
#include "hmi.h"
#include "app_error.h"
#include "power.h"

///////////////////// defines /////////////////////////////////////////////////
#define TIMESLOTDELAY 250000

///////////////////// type definitions ////////////////////////////////////////
typedef struct
{
    uint8_t StuffCounter;
    uint8_t BitCounter;
    uint16_t DataByte;
    uint8_t LastBitSize;
}
com_TransmissionStruct;

typedef struct
{
    uint8_t Buffer[COM_BUFFER];
    uint8_t Read;
    uint8_t Write;
}
com_TransmitFIFOStruct;

///////////////////// internal function prototypes ////////////////////////////
static inline uint8_t _crc_ibutton_update(uint8_t, uint8_t);
static inline void com_Enable(void);
static void com_Sync(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void com_StartTransmission(void);
static void com_StopTransmission(void);
static void com_Disable(void);
static void com_Receive(void);
static void com_Transmit(void);

///////////////////// local variables /////////////////////////////////////////
static volatile uint8_t com_RxDoneFlag = 0;
static uint8_t com_RXBuffer = 0;
static uint8_t com_ReceiveBuffer[2][18];
static com_TransmitFIFOStruct com_TransmitBuffer;
static com_TransmissionStruct com_TX = {4,144,0,0};
static com_TransmissionStruct com_RX = {4,0,0,1};
static com_MessageStruct com_InputMessage;
static const nrf_drv_timer_t timerInst = NRF_DRV_TIMER_INSTANCE(1);
static uint16_t timeslotlength;

///////////////////// private functions ///////////////////////////////////////
static inline uint8_t _crc_ibutton_update(uint8_t crc, uint8_t data)
{
	uint8_t i;

	crc = crc ^ data;
	for (i = 0; i < 8; i++)
	{
		if (crc & 0x01)
			crc = (crc >> 1) ^ 0x8C;
		else
			crc >>= 1;
	}

	return crc;
}

//---------------------------------------------------------------------------//
// Com Enable function                                                       //
// powers up and start timer used by com interface                           //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
static inline void com_Enable()
{
    nrf_timer_cc_write(timerInst.p_reg, NRF_TIMER_CC_CHANNEL2, 0);
    nrf_drv_timer_enable(&timerInst);
}

//---------------------------------------------------------------------------//
// Start Transmission funktion                                               //
// starts a new transmission                                                 //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
static void com_StartTransmission()
{
    com_TX.BitCounter = 0;
    com_TX.StuffCounter = 0;
    com_TX.LastBitSize = (3 + (com_TransmitBuffer.Buffer[(com_TransmitBuffer.Read+1) & (COM_BUFFER-1)] & 0x0F)) << 3;
    com_TX.DataByte = com_TransmitBuffer.Buffer[com_TransmitBuffer.Read];
    if (com_Off())
        com_Enable();
    else
        nrf_drv_timer_clear(&timerInst);
    com_TXEnable();
    com_OutputLow();
}

//---------------------------------------------------------------------------//
// Stop Transmission function                                                //
// stops an ongouing Transmission and resets FIFO read pointer               //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
static void com_StopTransmission()
{
    com_OutputOD();
    com_TXDisable();
    com_TX.BitCounter = 144;
}

//---------------------------------------------------------------------------//
// Com disable function                                                      //
// disables com interface and power downs timer                              //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
static void com_Disable()
{
    nrf_drv_timer_disable(&timerInst);
    com_OutputOD();
    nrf_drv_timer_clear(&timerInst);
    nrf_timer_cc_write(timerInst.p_reg, NRF_TIMER_CC_CHANNEL1, 95);
    nrf_timer_cc_write(timerInst.p_reg, NRF_TIMER_CC_CHANNEL2, 1);
    nrf_timer_event_clear(timerInst.p_reg, NRF_TIMER_EVENT_COMPARE0);
    nrf_timer_event_clear(timerInst.p_reg, NRF_TIMER_EVENT_COMPARE1);
}

//---------------------------------------------------------------------------//
// Com receive function                                                      //
// receives incomming messages                                               //
// no input or return values                                                 //
//                                                                           //
// call from variable output compare timer isr                               //
//---------------------------------------------------------------------------//
static void com_Receive()
{
    uint8_t com_Input;

    com_Input = com_InputState();

    if ((com_TX.BitCounter != 144) && (com_Input != com_OutputState()))
        com_StopTransmission();

    if (com_RX.StuffCounter < 4)
    {
        if (com_Input)
        {
            com_RX.DataByte <<= 1;
            com_RX.DataByte++;
        }
        else
            com_RX.DataByte <<= 1;
        if ((com_RX.BitCounter & 0x07) == 0x07)
            com_ReceiveBuffer[com_RXBuffer][com_RX.BitCounter>>3] = com_RX.DataByte;
        com_RX.BitCounter++;
    }
    else if (com_Input && (com_RX.StuffCounter == 5))
    {
        if (com_RX.BitCounter >= ((3 + (com_ReceiveBuffer[com_RXBuffer][1] & 0x0F)) << 3))
        {
            com_RxDoneFlag = 1;
            com_RXBuffer ^= 1;
            pwr_SetActiveFlag(1<<pwr_ACTIVECOM);
        }
        com_RX.BitCounter = 0;
    }
    else if (com_Input && (com_RX.StuffCounter == 8))
    {
        com_RX.BitCounter = 0;
        //if (com_TransmitBuffer.Write != com_TransmitBuffer.Read)
        //    com_StartTransmission();
    }

    if (com_Input == com_RX.LastBitSize)
    {
        if (com_RX.StuffCounter < 32)
            com_RX.StuffCounter++;
        else
        {
            com_Disable();
            com_RX.BitCounter = 0;
            com_RX.StuffCounter = 8;
        }
    }
    else
    {
        com_RX.StuffCounter = 0;
        com_RX.LastBitSize = com_Input;
    }
}

//---------------------------------------------------------------------------//
// Com transmit functioon                                                    //
// transmits outgoing messages                                               //
// no input or return values                                                 //
//                                                                           //
// call from staticle output compare / overflow timer isr                    //
//---------------------------------------------------------------------------//
static void com_Transmit()
{
    if (com_TX.BitCounter < com_TX.LastBitSize)
    {
        if (com_TX.StuffCounter < 4)
        {
            com_TX.DataByte <<= 1;
            if (com_TX.DataByte & (1<<8))
            {
                if (com_OutputState())
                    com_TX.StuffCounter++;
                else
                    com_TX.StuffCounter = 0;
                com_OutputOD();
            }
            else
            {
                if (com_OutputState())
                    com_TX.StuffCounter = 0;
                else
                    com_TX.StuffCounter++;
                com_OutputLow();
            }
            if ((com_TX.BitCounter++ & 0x07) == 0x07)
                com_TX.DataByte = com_TransmitBuffer.Buffer[(com_TransmitBuffer.Read + (com_TX.BitCounter>>3)) & (COM_BUFFER-1)];
        }
        else
        {
            com_TX.StuffCounter = 0;
            com_OutputToggle();
        }
    }
    else
    {
        com_TransmitBuffer.Read = (com_TransmitBuffer.Read + (com_TX.LastBitSize>>3)) & (COM_BUFFER-1);
        com_StopTransmission();
    }
}

static void timerHandler(nrf_timer_event_t eventType, void *pContext)
{
    (void)pContext;

    if (eventType == NRF_TIMER_EVENT_COMPARE0)
        com_Transmit();
    if (eventType == NRF_TIMER_EVENT_COMPARE1)
        com_Receive();
}

static void com_Sync(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (com_Off())
        com_Enable();
    else
    {
        //NRF_TIMER1->TASKS_CAPTURE[3] = 1;
        uint32_t newCapt = nrf_timer_cc_read(timerInst.p_reg, NRF_TIMER_CC_CHANNEL3);
        newCapt += 95;
        newCapt &= 0xFF;
        nrf_timer_cc_write(timerInst.p_reg, NRF_TIMER_CC_CHANNEL1, newCapt);
    }
}

static void com_StartTimeslot()
{
    uint32_t errCode;

    timeslotlength = (4 + (com_TransmitBuffer.Buffer[(com_TransmitBuffer.Read+1) & (COM_BUFFER-1)] & 0x0F)); // number of bytes to transmit including crc and padding byte at end
    timeslotlength *= 1280;     // maximum length of one byte including stuff bits
    timeslotlength += 500;      // additional buffer

    nrf_radio_request_t timeslot =
    {
        NRF_RADIO_REQ_TYPE_EARLIEST,
        .params.earliest =
        {
            NRF_RADIO_HFCLK_CFG_DEFAULT,
            NRF_RADIO_PRIORITY_HIGH,
            timeslotlength,
            TIMESLOTDELAY
        }
    };

    errCode = sd_radio_request(&timeslot);
    if (errCode != NRF_ERROR_FORBIDDEN)
    {
        APP_ERROR_CHECK(errCode);
    }
}

static nrf_radio_signal_callback_return_param_t *timeslotCallback (uint8_t sig)
{
    static nrf_radio_signal_callback_return_param_t retVal;

    retVal.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
    switch (sig)
    {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        // timeslot was granted, set timer0 timeout and start measurement
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
        NRF_TIMER0->CC[0] = timeslotlength - 500;
        NVIC_ClearPendingIRQ(TIMER0_IRQn);
        NVIC_EnableIRQ(TIMER0_IRQn);
        com_StartTransmission();
        break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        if (com_RX.BitCounter != 0)     // communication still ongoing, try to extend timeslot
        {
            retVal.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
            retVal.params.extend.length_us = 1280;
            timeslotlength = 1280;
        }
        else if (com_TransmitBuffer.Write != com_TransmitBuffer.Read)   // another message is waiting to be sent
        {
            retVal.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
            retVal.params.extend.length_us = 1280;
            timeslotlength = (4 + (com_TransmitBuffer.Buffer[(com_TransmitBuffer.Read+1) & (COM_BUFFER-1)] & 0x0F));
            timeslotlength *= 1280;
        }
        else
        {
            NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
            timeslotlength = 0;
            retVal.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
        }
        break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
        NRF_TIMER0->CC[0] += timeslotlength;
        break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
        NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
        timeslotlength = 0;
        retVal.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
        break;
    default:
        break;
    }
    return &retVal;
}

///////////////////// external function ///////////////////////////////////////

//---------------------------------------------------------------------------//
// Init function                                                             //
// prepares Pin and Timer of Com Interface                                   //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
void com_Init()
{
    uint32_t errCode;
    nrf_drv_timer_config_t timerConfig =
    {
        .frequency =          NRF_TIMER_FREQ_2MHz,
        .mode =               NRF_TIMER_MODE_TIMER,
        .bit_width =          NRF_TIMER_BIT_WIDTH_8,
        .interrupt_priority = NRF_APP_PRIORITY_HIGH,
        .p_context =          NULL
    };

    // request XTAL
    sd_clock_hfclk_request();

    // timer configuration
    errCode = nrf_drv_timer_init(&timerInst, &timerConfig, &timerHandler);
    APP_ERROR_CHECK(errCode);
    nrf_drv_timer_clear(&timerInst);
    nrf_drv_timer_compare(&timerInst, NRF_TIMER_CC_CHANNEL0, 255, false);
    nrf_drv_timer_compare(&timerInst, NRF_TIMER_CC_CHANNEL1, 95, true);
    nrf_drv_timer_compare(&timerInst, NRF_TIMER_CC_CHANNEL2, 1, false);

    // TX pin configuration
    nrf_gpio_cfg(PCOMTX, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(PCOMTX);

    // RX pin configuration
    if (!nrf_drv_gpiote_is_init())
    {
        errCode = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(errCode);
    }
    nrf_drv_gpiote_in_config_t rxConfig = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    errCode = nrf_drv_gpiote_in_init(PCOMRX, &rxConfig, &com_Sync);
    APP_ERROR_CHECK(errCode);

    nrf_drv_gpiote_in_event_enable(PCOMRX, true);

    // synchronization capture configuration
    /* synchronization only works with GPIOTE with high accuracy, but this will
       lead to higher off state current consumption, for now com has to work
       without synchronization.
    nrf_ppi_channel_t ppiCapture;
    errCode = nrf_drv_ppi_channel_alloc(&ppiCapture);
    APP_ERROR_CHECK(errCode);
    errCode = nrf_drv_ppi_channel_assign(ppiCapture,
                                         nrf_drv_gpiote_in_event_addr_get(PCOMRX),
                                         nrf_drv_timer_task_address_get(&timerInst, NRF_TIMER_TASK_CAPTURE3));
    APP_ERROR_CHECK(errCode);
    errCode = nrf_drv_ppi_channel_enable(ppiCapture);
    APP_ERROR_CHECK(errCode);*/

    // timeslot configuration
    errCode = sd_radio_session_open(&timeslotCallback);
    APP_ERROR_CHECK(errCode);
}

//---------------------------------------------------------------------------//
// Com Check function                                                        //
// checks incomming messageg for correct crc value                           //
// no input values                                                           //
// returns value: pointer to received message                                //
//                zero pointer if no message or crc wrong                    //
//---------------------------------------------------------------------------//
const com_MessageStruct *com_Check()
{
    uint8_t crctemp, i;
    com_MessageStruct *pMessage;

    pMessage = NULL;

    if (com_RxDoneFlag)
    {
        com_RxDoneFlag = 0;
        pwr_ClearActiveFlag(1<<pwr_ACTIVECOM);
        crctemp = 0xFF;
        for (i = 0; i < 3 + (com_ReceiveBuffer[com_RXBuffer ^ 1][1] & 0x0F); i++)
            crctemp = _crc_ibutton_update(crctemp, com_ReceiveBuffer[com_RXBuffer ^ 1][i]);
        if (!crctemp)
        {
            pMessage = &com_InputMessage;
            memcpy(pMessage, com_ReceiveBuffer[com_RXBuffer ^ 1], 2 + (com_ReceiveBuffer[com_RXBuffer ^ 1][1] & 0x0F));
        }
    }

    if ( timeslotlength == 0 && (com_Off()) && (com_InputState()) && (com_TransmitBuffer.Write != com_TransmitBuffer.Read))
        com_StartTimeslot();

    return pMessage;
}

void com_OnSysEvt(uint32_t sysEvent)
{
    switch (sysEvent)
    {
    case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
        APP_ERROR_HANDLER(0);
        break;
    case NRF_EVT_RADIO_SESSION_IDLE:
        timeslotlength = 0;
        break;
    case NRF_EVT_RADIO_BLOCKED:
    case NRF_EVT_RADIO_CANCELED:
        timeslotlength = 0;
        APP_ERROR_HANDLER(0);
        break;
    case NRF_EVT_RADIO_SESSION_CLOSED:
        break;
    default:
        break;
    }
}

//---------------------------------------------------------------------------//
// Com Put function                                                          //
// puts outgoing messages into FIFO buffer and calculates crc                //
// input value: pointer to MessageStruct to be sent                          //
// returns value: ErrorEnum (FIFOFULL or NOERROR)                            //
//---------------------------------------------------------------------------//
com_ErrorEnum com_Put(const com_MessageStruct *pMessage)
{
    com_ErrorEnum error;
    uint8_t size, space, crctemp, i;

    //if (pMessage->Identifier != 0x65)
    //    return COM_NOERROR;

    size = 3 + (pMessage->Control & 0x0F);
    if (com_TransmitBuffer.Write >= com_TransmitBuffer.Read)
        space = com_TransmitBuffer.Read + COM_BUFFER - com_TransmitBuffer.Write;
    else
        space = com_TransmitBuffer.Read - com_TransmitBuffer.Write;
    if (space < size)
        error =  COM_FIFOFULL;
    else {
        crctemp = 0xFF;
        for (i = 0; i < size-1; i++) {
            com_TransmitBuffer.Buffer[com_TransmitBuffer.Write] = *(uint8_t *)pMessage;
            pMessage = (com_MessageStruct *)((uint8_t *)pMessage + 1);
            crctemp = _crc_ibutton_update(crctemp, com_TransmitBuffer.Buffer[com_TransmitBuffer.Write++]);
            com_TransmitBuffer.Write &= (COM_BUFFER-1);
        }
        com_TransmitBuffer.Buffer[com_TransmitBuffer.Write++] = crctemp;
        com_TransmitBuffer.Write &= (COM_BUFFER-1);
        error = COM_NOERROR;
    }
    //if ( (com_Off()) && (com_InputState()) && (com_TransmitBuffer.Write != com_TransmitBuffer.Read))
    //    com_StartTransmission();
    return error;
}

//---------------------------------------------------------------------------//
// Com FlushFIFO function                                                    //
// flushs fifo output buffer (usefull if error on bus)                       //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
void com_FlushFIFO()
{
    com_TransmitBuffer.Read = 0;
    com_TransmitBuffer.Write = 0;
}

///////////////////// internal function ///////////////////////////////////////
