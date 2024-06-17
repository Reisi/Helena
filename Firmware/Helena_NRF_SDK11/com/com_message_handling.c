/**
  ******************************************************************************
  * @file    com_message_handling.c
  * @author  Thomas Reisnecker
  * @brief   module to handle the com messages related to the helen project
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define COM_LOG_ENABLED

#ifdef COM_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // MAIN_LOG_ENABLED



/* Includes ------------------------------------------------------------------*/
#include "app_timer.h"
#include "main.h"
#include "debug.h"
#include "com_message_handling.h"
#include "mode_management.h"

/* External variables --------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define ID_MASTER_MAIN  0x00
#define ID_MASTER_AUX   0x02
#define ID_FRONT_HLMT   0x64
#define ID_TAIL_PRI     0x6A
#define ID_TAIL_SEC     0x6C
#define ID_BAT_PRI      0x24
#define ID_BAT_SEC      0x26

#define MSG_LEN_MASTER  3
#define MSG_LEN_HLMT    5
#define MSG_LEN_TAIL    5
#define MSG_LEN_BAT     7

#define ID_OWN          ID_FRONT_HLMT

#define VALID_TIMEOUT   MAIN_TIMER_TICKS(30000)

/* Private typedef -----------------------------------------------------------*/
/// TODO: how to handle primary and secondary taillights/batteries?
///       at the moment I'm just expecting, that there will never be primary and
///       secondary on the same bus and just requesting secondary because main
///       application is indented to be the helmet light

typedef struct
{
    q3_13_t power;
    bool requested;     // indicating if remote transfer was requested
    uint32_t timestamp; // timestamp of last reception
} taillight_t;

typedef struct
{
    q7_9_t soc;
    bool requested;     // indicating if remote transfer was requested
    uint32_t timestamp; // timestamp of last reception
} battery_t;

/* Private macros ------------------------------------------------------------*/

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static cmh_eventHandler_t evtHandler;           // the handler to report events

static bool               isActive;             // indicator if sending is enabled
static bool               ignoreNextAuxMaster;  // indicator to ignore self sent master messages

static taillight_t        taillight;            // buffer of taillight power
static battery_t          battery;              // buffer of battery soc

/* Private functions ---------------------------------------------------------*/
/** @brief handler for status messages */
static void onStatusReceived(com_message_t const* pMsg)
{
    uint32_t i; // helper variable to convert from com units to project units

    // taillight status message, ignore if length is wrong
    if ((pMsg->id == ID_TAIL_PRI || pMsg->id == ID_TAIL_SEC) &&
        pMsg->len == MSG_LEN_TAIL)
    {
        i = (1 + (pMsg->data[1] >> 4)) * 2; // led voltage
        i *= pMsg->data[2];                 // power in com units

        i <<= 13;                           // convert to q3_13_t
        i /= 2500;

        taillight.power = i;
        taillight.requested = false;
         (void)app_timer_cnt_get(&taillight.timestamp);
    }
    // battery status message, ignore if length is wrong
    else if ((pMsg->id == ID_BAT_PRI || pMsg->id == ID_BAT_SEC) &&
        pMsg->len == MSG_LEN_BAT)
    {
        i = pMsg->data[6];                  // soc in com units

        i <<= 9;                            // convert to q7_9_t
        i *= 100;
        i /= 255;

        battery.soc = i;
        battery.requested = false;
        (void)app_timer_cnt_get(&battery.timestamp);
    }
}

/** @brief handler for reset request messages */
static void onResetReq(uint8_t id)
{
    if (id == ID_OWN)
    {
        cmh_event_t evt = {.type = CMH_EVENT_RESET_REQUEST};
        evtHandler(&evt);
    }
}

/** @brief helper function to decode master messages to desired mode */
static uint8_t decodeMode(uint8_t const* pData)
{
    if (pData[0] == 0x02 && (pData[1] & 0x0F) == 0)     // main beam low
        return 0;

    if (pData[0] == 0x33 && (pData[1] & 0x0F) == 0)     // main and high beam high
        return 1;

    if (pData[0] == 0x02 && (pData[1] & 0x0F) == 0x02)  // main beam and helmet low
        return 2;

    if (pData[0] == 0x30 && (pData[1] & 0x0F) == 0x03)  // high beam and helmet high
        return 3;

    return MM_MODE_OFF;
}

/** @brief helper function to encode the mode to master message */
static void encodeMode(uint8_t mode, uint8_t* pData)
{
    switch (mode)
    {
    case 0:
        pData[0] = 0x02;
        pData[1] = 0;
        pData[2] = 0x02;
        break;
    case 1:
        pData[0] = 0x33;
        pData[1] = 0;
        pData[2] = 0x02;
        break;
    case 2:
        pData[0] = 0x02;
        pData[1] = 0x02;
        pData[2] = 0x02;
        break;
    case 3:
        pData[0] = 0x30;
        pData[1] = 0x03;
        pData[2] = 0x02;
        break;
    default:
        pData[0] = 0;
        pData[1] = 0;
        pData[2] = 0;
        break;
    }
}

/** @brief handler for status return requests */
static void onStatusReturn(com_message_t const* pMsg)
{
    // just messages from masters are evaluated
    if ((pMsg->id == ID_MASTER_MAIN || pMsg->id == ID_MASTER_AUX) &&
        pMsg->len == MSG_LEN_MASTER)
    {
        // ignore our own requests
        if (ignoreNextAuxMaster && pMsg->id == ID_MASTER_AUX)
        {
            //NRF_LOG_WARNING("ignore");
            ignoreNextAuxMaster = false;
            return;
        }

        cmh_event_t evt;
        evt.type = CMH_EVENT_MODE_REQUEST;
        evt.mode = decodeMode(pMsg->data);
        evtHandler(&evt);
    }
}

/** @brief handler for remote transfer requests */
static void onRemoteTransferReq(uint8_t id)
{
    if (id == ID_OWN && isActive)   // no need to send event if we can't send messages
    {
        cmh_event_t evt = {.type = CMH_EVENT_STATUS_REQUEST};
        evtHandler(&evt);
    }
}

/** @brief handler for incoming messages */
static void comHandler(com_event_t const* pEvt)
{
    if (evtHandler == NULL)
        return;

    if (pEvt->type == COM_EVT_MESSAGE_RECEIVED)
    {
        switch (pEvt->pMessage->type)
        {
        case COM_MSG_STATUS:
            onStatusReceived(pEvt->pMessage);
            break;
        case COM_MSG_RESETREQ:
            onResetReq(pEvt->pMessage->id);
            break;
        case COM_MSG_STATUSRETREQ:
            onStatusReturn(pEvt->pMessage);
            break;
        case COM_MSG_REMTRANSREQ:
            onRemoteTransferReq(pEvt->pMessage->id);
            break;
        default:
            break;
        }
    }
}

/** @brief function to send a remote transfer request message
 *
 * @param[in] id  the id of the device
 */
static ret_code_t transferRequest(uint8_t id)
{
    com_message_t msg;

    if (!isActive)
        return NRF_ERROR_NOT_SUPPORTED;

    msg.id = id;
    msg.type = COM_MSG_REMTRANSREQ;
    msg.len = 0;

    return com_Put(&msg);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t cmh_Init(cmh_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->handler == NULL)
        return NRF_ERROR_NULL;

    if (pInit->rxPin == COM_PIN_NOT_USED && pInit->txPin == COM_PIN_NOT_USED)
        return NRF_SUCCESS;

    if (pInit->rxPin == COM_PIN_NOT_USED)
        return NRF_ERROR_INVALID_PARAM;

    com_init_t init;

    init.rxPin = pInit->rxPin;
    init.txPin = pInit->txPin;
    init.pHandler = comHandler;

    errCode = com_Init(&init);
    if (errCode == NRF_SUCCESS)
    {
        evtHandler = pInit->handler;
        isActive   = pInit->txPin != COM_PIN_NOT_USED;
    }

    //LOG_ERROR_CHECK("com init error %d", errCode);

    return errCode;
}

ret_code_t cmh_RelayMode(uint8_t mode)
{
    if (evtHandler == NULL)
        return NRF_ERROR_INVALID_STATE;

    if (!isActive)
        return NRF_ERROR_NOT_SUPPORTED;

    ret_code_t errCode;
    com_message_t msg;

    msg.id = ID_MASTER_AUX;
    msg.type = COM_MSG_STATUSRETREQ;
    msg.len = 3;
    encodeMode(mode, msg.data);

    errCode = com_Put(&msg);
    if (errCode == NRF_SUCCESS)
        ignoreNextAuxMaster = true;

    return errCode;
}

ret_code_t cmh_GetTaillightPower(q3_13_t* pPower)
{
    if (pPower == NULL)
        return NRF_ERROR_NULL;

    ret_code_t retVal;
    uint32_t time;
    (void)app_timer_cnt_get(&time);
    *pPower = taillight.power;

    if (time == 0)
        retVal = NRF_ERROR_INVALID_STATE;   // timer inactive
    else
    {
        (void)app_timer_cnt_diff_compute(time, taillight.timestamp, &time);
        if (time < VALID_TIMEOUT && taillight.timestamp != 0)
            retVal = NRF_SUCCESS;               // data valid
        else
        {
            if (!taillight.requested)
            {
                (void)transferRequest(ID_TAIL_SEC); /// TODO: also request primary
                taillight.requested = true;
            }
            if (taillight.timestamp == 0)
                retVal = NRF_ERROR_NOT_FOUND;   // nothing heard yet
            else
                retVal = NRF_ERROR_TIMEOUT;     // data too old
        }
    }
    return retVal;
}

ret_code_t cmh_GetBatterySOC(q7_9_t* pSOC)
{
    if (pSOC == NULL)
        return NRF_ERROR_NULL;

    ret_code_t retVal;
    uint32_t time;
    (void)app_timer_cnt_get(&time);
    *pSOC = battery.soc;

    if (time == 0)
        retVal = NRF_ERROR_INVALID_STATE;   // timer inactive
    else
    {
        (void)app_timer_cnt_diff_compute(time, taillight.timestamp, &time);
        if (time < VALID_TIMEOUT && battery.timestamp != 0)
            retVal = NRF_SUCCESS;               // data valid
        else
        {
            if (!battery.requested)
            {
                (void)transferRequest(ID_BAT_SEC); /// TODO: request primary, too
                battery.requested = true;
            }
            if (battery.timestamp == 0)
                retVal = NRF_ERROR_NOT_FOUND;   // nothing heard yet
            else
                retVal = NRF_ERROR_TIMEOUT;     // data too old
        }
    }
    return retVal;
}

/**END OF FILE*****************************************************************/
