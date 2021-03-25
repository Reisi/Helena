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
    uint8_t id;                             // com id for this light type
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

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define MASTERID                0x00
#define AUXMASTERID             0x02
#define MAINBEAMID              0x60
#define HIGHBEAMID              0x62
#define HELMETLIGHTID           0x64

#define LIGHTERROR_OVERCURRENT  (1<<0)
#define LIGHTERROR_VOLTAGE      (1<<1)
#define LIGHTERROR_TEMPERATURE  (1<<2)
#define LIGHTERROR_DIRECTDRIVE  (1<<3)

#define LIGHT_MASTER_TIMEOUT    (APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER))   // timeout for sending taillight messages
#define LIGHT_MASTER_INVALID    0xFFFFFFFFul

#define LIGHT_TIMEBASE          (APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER))
#define BRAKEINDICATOR_TIMEBASE (APP_TIMER_TICKS(250, APP_TIMER_PRESCALER))

/* Private variables ---------------------------------------------------------*/
#ifdef HELENA
static lightData_t lightData[1];                    // data structure holding helmet light information
#elif defined BILLY
static lightData_t lightData[2];                    // data structures holding main and high beam information
#endif // defined
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
    if (lightMasterHandler != NULL)
        pLightMasterHandler = lightMasterHandler;

    lastLightMasterMessage = LIGHT_MASTER_INVALID;

#ifdef BILLY
    lightData[0].id = MAINBEAMID;
    lightData[1].id = HIGHBEAMID;
#elif defined HELENA
    lightData[0].id = HELMETLIGHTID;
#endif // defined

    return NRF_SUCCESS;
}

void cmh_Execute()
{
    uint32_t timestamp, timediff;

    (void)app_timer_cnt_get(&timestamp);

    (void)app_timer_cnt_diff_compute(timestamp, lastLightMasterMessage, &timediff);
    if (timediff >= LIGHT_MASTER_TIMEOUT)
        lastLightMasterMessage = LIGHT_MASTER_INVALID;

    for (uint_fast8_t i = 0; i < sizeof(lightData)/sizeof(lightData[0]); i++)
    {
        (void)app_timer_cnt_diff_compute(timestamp, lightData[i].lastTimeSent, &timediff);
        if (timediff >= LIGHT_TIMEBASE && lightData[i].isPending == true)
        {
            if (sendLightMessage(&lightData[i].pendingMessage, lightData[i].id) == NRF_SUCCESS)
            {
                lightData[i].lastMessage = lightData[i].pendingMessage;
                lightData[i].isPending = false;
                lightData[i].lastTimeSent = timestamp;
            }
            else
            {
                lightData[i].isPending = true;
            }
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
    if (pMessageIn->Control == 0xF0)
    {
        for (uint_fast8_t i = 0; i < sizeof(lightData)/sizeof(lightData[0]); i++)
        {
            if (pMessageIn->Identifier == lightData[i].id)
            {
                if (sendLightMessage(&lightData[i].pendingMessage, lightData[i].id) == NRF_SUCCESS)
                {
                    lightData[i].lastMessage = lightData[i].pendingMessage;
                    lightData[i].isPending = false;
                    (void)app_timer_cnt_get(&lightData[i].lastTimeSent);
                }
                else
                {
                    lightData[i].isPending = true;
                }
            }
        }
    }

    // handle remote reset request
    if (pMessageIn->Control == 0x10)
    {
        for (uint_fast8_t i = 0; i < sizeof(lightData)/sizeof(lightData[0]); i++)
        {
            if (pMessageIn->Identifier == lightData[i].id)
                NVIC_SystemReset();
        }
    }

    // handle master messages
    if ((pMessageIn->Identifier == MASTERID || pMessageIn->Identifier == AUXMASTERID) &&
        pMessageIn->Control == 0x43)
    {
        if (pMessageIn->Identifier == MASTERID)
            (void)app_timer_cnt_get(&lastLightMasterMessage);   // save timestamp of valid master message

        if (pMessageIn->Identifier == AUXMASTERID && ignoreAuxMaster != 0)
        {
            ignoreAuxMaster--;
            return;                 // ignore auxiliary master messages, if we sent them
        }

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
}

static uint32_t updateLightData(cmh_light_t* pLight, uint8_t id)
{
    uint32_t timestamp;
    lightData_t* pLightData = NULL;

    for (uint_fast8_t i = 0; i < sizeof(lightData)/sizeof(lightData[0]); i++)
    {
        if (id == lightData[i].id)
            pLightData = &lightData[i];
    }

    if (pLightData == NULL)
        return NRF_ERROR_INVALID_PARAM;

    pLightData->pendingMessage.errorFlags = 0;
    if (pLight->overcurrentError)
        pLightData->pendingMessage.errorFlags |= LIGHTERROR_OVERCURRENT;
    if (pLight->voltageError)
        pLightData->pendingMessage.errorFlags |= LIGHTERROR_VOLTAGE;
    if (pLight->temperatureError)
        pLightData->pendingMessage.errorFlags |= LIGHTERROR_TEMPERATURE;
    if (pLight->directdriveError)
        pLightData->pendingMessage.errorFlags |= LIGHTERROR_DIRECTDRIVE;
    pLightData->pendingMessage.mode = pLight->mode;
    pLightData->pendingMessage.current = pLight->current / 20;
    pLightData->pendingMessage.temperature = (pLight->temperature - (2730 - 400)) / 5;
    pLightData->pendingMessage.voltage = (pLight->voltage - 5500) / 50;

    (void)app_timer_cnt_get(&timestamp);
    (void)app_timer_cnt_diff_compute(timestamp, pLightData->lastTimeSent, &timestamp);

    if (/*pLightData->pendingMessage.errorFlags != pLightData->lastMessage.errorFlags ||*/
        pLightData->pendingMessage.mode != pLightData->lastMessage.mode ||
        (timestamp >= LIGHT_TIMEBASE &&
         memcmp(&pLightData->pendingMessage, &pLightData->lastMessage, sizeof(lightMessageData_t)) != 0))
    {
        if (sendLightMessage(&(pLightData->pendingMessage), id) == NRF_SUCCESS)
        {
            pLightData->lastMessage = pLightData->pendingMessage;
            pLightData->isPending = false;
            (void)app_timer_cnt_get(&pLightData->lastTimeSent);
        }
        else
        {
            pLightData->isPending = true;
        }
    }

    return NRF_SUCCESS;
}

#ifdef BILLY

uint32_t cmh_UpdateMainBeam(cmh_light_t* pLight)
{
    return updateLightData(pLight, MAINBEAMID);
}

uint32_t cmh_UpdateHighBeam(cmh_light_t* pLight)
{
    return updateLightData(pLight, HIGHBEAMID);
}

#elif defined HELENA

uint32_t cmh_UpdateHelmetLight(cmh_light_t* pLight)
{
    return updateLightData(pLight, HELMETLIGHTID);
}

#endif // define

uint32_t cmh_UpdateBrakeIndicator(bool braking)
{
    uint32_t timestamp;

    (void)app_timer_cnt_get(&timestamp);
    (void)app_timer_cnt_diff_compute(timestamp, brakeIndicator.lastTimeSent, &timestamp);

    if (timestamp >= BRAKEINDICATOR_TIMEBASE && braking)
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



