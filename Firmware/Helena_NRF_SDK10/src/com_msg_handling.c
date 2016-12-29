/**
  ******************************************************************************
  * @file    com_msg_handling.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/06
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
} lightMessageDataStruct;

typedef struct
{
    uint32_t lastTimeSent;                  /**< timestamp of last message sent */
    lightMessageDataStruct  lastMessage;    /**< last message data */
    lightMessageDataStruct  pendingMessage; /**< pending message data */
    bool isPending;                         /**< indicator if message is pending */
} lightDataStruct;

typedef lightDataStruct helmetLightDataStruct;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define MASTERID                0x00
#define AUXMASTERID             0x02
#define HELMETLIGHTID           0x64

#define LIGHTERROR_OVERCURRENT  (1<<0)
#define LIGHTERROR_VOLTAGE      (1<<1)
#define LIGHTERROR_TEMPERATURE  (1<<2)
#define LIGHTERROR_DIRECTDRIVE  (1<<3)

#define HELMETLIGHT_TIMEBASE    (APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER))

/* Private variables ---------------------------------------------------------*/
APP_TIMER_DEF(msgTimerId);  /**< timer for message generation */
static uint8_t TimeoutCnt;
static helmetLightDataStruct helmetLight;
static cmh_LightMasterHandler pLightMaster;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void msgTimeoutHandler(void* pContext)
{
    (void)pContext;

    TimeoutCnt++;
}

uint32_t sendMessage(lightMessageDataStruct* pData, uint8_t id)
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

/* Public functions ----------------------------------------------------------*/
uint32_t cmh_Init(cmh_LightMasterHandler pLightMasterHandler)
{
    if (pLightMasterHandler != NULL)
        pLightMaster = pLightMasterHandler;

    return NRF_SUCCESS;
}

uint32_t cmh_UpdateHelmetLight(cmh_HelmetLightStruct* pLight)
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
         memcmp(&helmetLight.pendingMessage, &helmetLight.lastMessage, sizeof(lightMessageDataStruct)) != 0))
    {
        if (sendMessage(&helmetLight.pendingMessage, HELMETLIGHTID) == NRF_SUCCESS)
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

void cmh_Execute()
{
    uint32_t timestamp;

    (void)app_timer_cnt_get(&timestamp);
    (void)app_timer_cnt_diff_compute(timestamp, helmetLight.lastTimeSent, &timestamp);

    if (timestamp >= HELMETLIGHT_TIMEBASE && helmetLight.isPending == true)
    {
        if (sendMessage(&helmetLight.pendingMessage, 0x64) == NRF_SUCCESS)
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
}

void cmh_ComMessageCheck(const com_MessageStruct * pMessageIn)
{
    if (pMessageIn == NULL)
        return;

    // handle remote requests
    if (pMessageIn->Identifier == HELMETLIGHTID && pMessageIn->Control == 0xF0)
    {
        if (sendMessage(&helmetLight.pendingMessage, 0x64) == NRF_SUCCESS)
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
    if (pLightMaster != NULL &&
        (pMessageIn->Identifier == MASTERID || pMessageIn->Identifier == AUXMASTERID) &&
        pMessageIn->Control == 0x43)
    {
        cmh_LightMasterDataStruct masterData;

        masterData.mainBeam = pMessageIn->Data[0] & 0x0F;
        masterData.highBeam = pMessageIn->Data[0] >> 4;
        masterData.helmetBeam = pMessageIn->Data[1] & 0x0F;
        masterData.taillight = pMessageIn->Data[2] & 0x0F;
        (*pLightMaster)(&masterData);
    }
}

/**END OF FILE*****************************************************************/



