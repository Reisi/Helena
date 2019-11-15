/**
  ******************************************************************************
  * @file    com_msg_handling.c
  * @author  Thomas Reisnecker
  * @brief   com messages handling module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "com_msg_handling.h"
#include "app_timer.h"
#include "main.h"
#include "app_error.h"
#include "nrf.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint8_t errorFlags;
    uint8_t mode;
    uint8_t current;
    uint8_t temperature;
    uint8_t voltage;
} lightMessageData_t;

typedef struct
{
    uint32_t lastTimeSent;                  // timestamp of last message sent
    lightMessageData_t  lastMessage;        // last message data
    lightMessageData_t  pendingMessage;     // pending message data
    bool isPending;                         // indicator if message is pending
} lightData_t;

typedef struct
{
    uint32_t lastTimeSent;                  // timestamp of last message sent
    bool isPending;                         // indicator if message is pending
} brakeIndicatorData_t;

typedef struct
{
    bool enable;                            // requested state of taillight
    bool isPending;                         // indicator if message is pending
} tailLightData_t;

typedef lightData_t helmetLightDataStruct;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define MASTERID                0x00
#define AUXMASTERID             0x02
#define HELMETLIGHTID           0x64

#define LIGHTERROR_OVERCURRENT  (1<<0)
#define LIGHTERROR_VOLTAGE      (1<<1)
#define LIGHTERROR_TEMPERATURE  (1<<2)
#define LIGHTERROR_DIRECTDRIVE  (1<<3)

#define LIGHT_MASTER_TIMEOUT    (APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER))   // timeout for sending taillight messages
#define LIGHT_MASTER_INVALID    0xFFFFFFFFul

#define HELMETLIGHT_TIMEBASE    (APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER))
#define BRAKEINDICATOR_TIMEBASE (APP_TIMER_TICKS(250, APP_TIMER_PRESCALER))

/* Private variables ---------------------------------------------------------*/
static helmetLightDataStruct helmetLight;           // data structure holding helmet light information
static brakeIndicatorData_t brakeIndicator;         // data for brake indicator
static tailLightData_t tailLight;                   // data for taillight

static cmh_LightMasterHandler_t pLightMasterHandler;// handler to be called for light master messages
static uint8_t ignoreAuxMaster;                     // counter indicating if auxiliary master message should be ignored
static uint32_t lastLightMasterMessage;             // timestamp of last received light master message

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/** @brief function to send a light status message
 */
uint32_t sendLightMessage(lightMessageData_t* pData, uint8_t id)
{
    com_MessageStruct MessageOut;

    MessageOut.Identifier = id;
    MessageOut.Control = 0x05;
    MessageOut.Data[0] = pData->errorFlags;
    MessageOut.Data[1] = pData->mode;
    MessageOut.Data[2] = pData->current;
    MessageOut.Data[3] = pData->temperature;
    MessageOut.Data[4] = pData->voltage;

    if (com_Put(&MessageOut) == COM_FIFOFULL)
        return NRF_ERROR_NO_MEM;
    else
        return NRF_SUCCESS;
}

/** @brief function to send a brake indicator message
 *
 * @note there is no brake indicator message yet, so an acceleration sensor message is used instead
 */
uint32_t sendBrakeMessage()
{
    com_MessageStruct MessageOut;

    MessageOut.Identifier = 0xB8;
    MessageOut.Control = 0x03;
    MessageOut.Data[0] = 128;   // represents 0g in x-axis
    MessageOut.Data[1] = 124;   // represents -0.25g in y-axis
    MessageOut.Data[2] = 128;   // represents 0g in z-axis

    if (com_Put(&MessageOut) == COM_FIFOFULL)
        return NRF_ERROR_NO_MEM;
    else
        return NRF_SUCCESS;
}

/** @brief function to send an auxiliary master message to enable/disable a taillight
 */
uint32_t sendTaillightMessage(bool enable)
{
    com_MessageStruct messageOut;

    messageOut.Identifier = AUXMASTERID;
    messageOut.Control = 0x43;
    messageOut.Data[0] = 0;
    messageOut.Data[1] = 0;
    messageOut.Data[2] = enable ? 2 : 0;

    if (com_Put(&messageOut) == COM_FIFOFULL)
        return NRF_ERROR_NO_MEM;
    else
        return NRF_SUCCESS;
}

/* Public functions ----------------------------------------------------------*/
uint32_t cmh_Init(cmh_LightMasterHandler_t lightMasterHandler)
{
    if (pLightMasterHandler != NULL)
        pLightMasterHandler = pLightMasterHandler;

    lastLightMasterMessage = LIGHT_MASTER_INVALID;

    return NRF_SUCCESS;
}

void cmh_Execute()
{
    uint32_t timestamp, timediff;

    (void)app_timer_cnt_get(&timestamp);

    (void)app_timer_cnt_diff_compute(timestamp, lastLightMasterMessage, &timediff);
    if (timediff >= LIGHT_MASTER_TIMEOUT)
        lastLightMasterMessage = LIGHT_MASTER_INVALID;

    (void)app_timer_cnt_diff_compute(timestamp, helmetLight.lastTimeSent, &timediff);
    if (timediff >= HELMETLIGHT_TIMEBASE && helmetLight.isPending == true)
    {
        if (sendLightMessage(&helmetLight.pendingMessage, 0x64) == NRF_SUCCESS)
        {
            helmetLight.lastMessage = helmetLight.pendingMessage;
            helmetLight.isPending = false;
            helmetLight.lastTimeSent = timestamp;
        }
        else
        {
            helmetLight.isPending = true;
        }
    }

    (void)app_timer_cnt_diff_compute(timestamp, brakeIndicator.lastTimeSent, &timediff);
    if (timediff >= BRAKEINDICATOR_TIMEBASE && brakeIndicator.isPending == true)
    {
        if (sendBrakeMessage() == NRF_SUCCESS)
        {
            brakeIndicator.isPending = false;
            brakeIndicator.lastTimeSent = timestamp;
        }
        else
        {
            brakeIndicator.isPending = true;
        }
    }

    if (tailLight.isPending)
    {
        if (sendTaillightMessage(tailLight.enable) == NRF_SUCCESS)
        {
            tailLight.isPending = false;
            ignoreAuxMaster++;
        }
    }
}

void cmh_ComMessageCheck(const com_MessageStruct * pMessageIn)
{
    if (pMessageIn == NULL)
        return;

    // handle remote requests
    if (pMessageIn->Identifier == HELMETLIGHTID && pMessageIn->Control == 0xF0)
    {
        if (sendLightMessage(&helmetLight.pendingMessage, 0x64) == NRF_SUCCESS)
        {
            helmetLight.lastMessage = helmetLight.pendingMessage;
            helmetLight.isPending = false;
            (void)app_timer_cnt_get(&helmetLight.lastTimeSent);
        }
        else
        {
            helmetLight.isPending = true;
        }
    }

    // handle remote reset request
    if (pMessageIn->Identifier == HELMETLIGHTID && pMessageIn->Control == 0x10)
        NVIC_SystemReset();

    // handle master messages
    if ((pMessageIn->Identifier == MASTERID || pMessageIn->Identifier == AUXMASTERID) &&
        pMessageIn->Control == 0x43)
    {
        if (pMessageIn->Identifier == MASTERID)
            (void)app_timer_cnt_get(&lastLightMasterMessage);   // save timestamp of valid master message

        if (ignoreAuxMaster == 0)                               // ignore auxiliary master messages if we sent them
        {
            if (pLightMasterHandler != NULL)
            {
                cmh_lightMasterData_t masterData;

                masterData.mainBeam = pMessageIn->Data[0] & 0x0F;
                masterData.highBeam = pMessageIn->Data[0] >> 4;
                masterData.helmetBeam = pMessageIn->Data[1] & 0x0F;
                masterData.taillight = pMessageIn->Data[2] & 0x0F;
                (*pLightMasterHandler)(&masterData);
            }
        }
        else
            ignoreAuxMaster--;
    }
}

uint32_t cmh_UpdateHelmetLight(cmh_helmetLight_t* pLight)
{
    uint32_t timestamp;

    helmetLight.pendingMessage.errorFlags = 0;
    if (pLight->overcurrentError)
        helmetLight.pendingMessage.errorFlags |= LIGHTERROR_OVERCURRENT;
    if (pLight->voltageError)
        helmetLight.pendingMessage.errorFlags |= LIGHTERROR_VOLTAGE;
    if (pLight->temperatureError)
        helmetLight.pendingMessage.errorFlags |= LIGHTERROR_TEMPERATURE;
    if (pLight->directdriveError)
        helmetLight.pendingMessage.errorFlags |= LIGHTERROR_DIRECTDRIVE;
    helmetLight.pendingMessage.mode = pLight->mode;
    helmetLight.pendingMessage.current = pLight->current / 20;
    helmetLight.pendingMessage.temperature = (pLight->temperature - (2730 - 400)) / 5;
    helmetLight.pendingMessage.voltage = (pLight->voltage - 5500) / 50;

    (void)app_timer_cnt_get(&timestamp);
    (void)app_timer_cnt_diff_compute(timestamp, helmetLight.lastTimeSent, &timestamp);

    if (/*helmetLight.pendingMessage.errorFlags != helmetLight.lastMessage.errorFlags ||*/
        helmetLight.pendingMessage.mode != helmetLight.lastMessage.mode ||
        (timestamp >= HELMETLIGHT_TIMEBASE &&
         memcmp(&helmetLight.pendingMessage, &helmetLight.lastMessage, sizeof(lightMessageData_t)) != 0))
    {
        if (sendLightMessage(&helmetLight.pendingMessage, HELMETLIGHTID) == NRF_SUCCESS)
        {
            helmetLight.lastMessage = helmetLight.pendingMessage;
            helmetLight.isPending = false;
            (void)app_timer_cnt_get(&helmetLight.lastTimeSent);
        }
        else
        {
            helmetLight.isPending = true;
        }
    }

    return NRF_SUCCESS;
}

uint32_t cmh_UpdateBrakeIndicator(bool braking)
{
    uint32_t timestamp;

    (void)app_timer_cnt_get(&timestamp);
    (void)app_timer_cnt_diff_compute(timestamp, brakeIndicator.lastTimeSent, &timestamp);

    if (timestamp >= HELMETLIGHT_TIMEBASE && braking)
    {
        if (sendBrakeMessage() == NRF_SUCCESS)
        {
            brakeIndicator.isPending = false;
            (void)app_timer_cnt_get(&brakeIndicator.lastTimeSent);
        }
        else
        {
            brakeIndicator.isPending = true;
        }
    }

    return NRF_SUCCESS;
}

uint32_t cmh_EnableTaillight(bool enable)
{
    if (lastLightMasterMessage != LIGHT_MASTER_INVALID)
        return NRF_ERROR_INVALID_STATE;

    if (sendTaillightMessage(enable) == NRF_SUCCESS)
    {
        tailLight.isPending = false;
        ignoreAuxMaster++;
    }
    else
    {
        tailLight.enable = enable;
        tailLight.isPending = true;
    }

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/



