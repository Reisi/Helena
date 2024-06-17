/**
  ******************************************************************************
  * @file    hmi.c
  * @author  Thomas Reisnecker
  * @brief   human machine interface module
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
//#define HMI_LOG_ENABLED

#ifdef HMI_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // HMI_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "app_timer.h"
#include "hmi.h"
#include "board.h"
#include "main.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define TIMEBASE                MAIN_TIMER_TICKS(500)

#define DELETE_TIMEOUT          (MAIN_TIMER_TICKS(3000)/TIMEBASE)
#define DELETE_INDICATOR        HMI_LT_CNT

/* Private variables ---------------------------------------------------------*/
APP_TIMER_DEF(hmi_Timer);                                   // timer for debouncing, evaluation and led management
static hmi_ledState_t ledState[HMI_LT_CNT + 1];             // state of status leds, the additional state is used as delete indicator
static hmi_eventHandler_t pEventHandler;                    // the hmi event handler

static uint8_t deleteIndicatorTimeout;

/* Private functions ---------------------------------------------------------*/
/** @brief helper function to determine if any LED is in use
 *
 * @return true any of the LEDs is in use, otherwise false
 */
static bool isLedActive()
{
    for (uint_fast8_t i = HMI_LT_RED; i < HMI_LT_CNT + 1; i++)
    {
        if (ledState[i] != HMI_LS_OFF)
            return true;
    }
    return false;
}

/** @brief function to request the use of the timer
 *
 * this function will automatically start and stop the timer
 *
 * @param request true to request, false to release
 */
static void timerRequest(bool request)
{
    static uint8_t cnt; // request counter to manage more than one user

    if (!request && --cnt == 0)
    {
        APP_ERROR_CHECK(app_timer_stop(hmi_Timer));
        LOG_INFO("[hmi]: Timer stopped.");
    }
    else if (request && cnt++ == 0)
    {
        APP_ERROR_CHECK(app_timer_start(hmi_Timer, TIMEBASE, NULL));
        LOG_INFO("[hmi]: Timer started.");
    }
}

static void ledHandling()
{
    static uint8_t prescale;

    prescale++;

    if (deleteIndicatorTimeout && --deleteIndicatorTimeout == 0)
        ledState[DELETE_INDICATOR] = HMI_LS_OFF;

    for (hmi_ledType_t i = HMI_LT_RED; i < HMI_LT_CNT; i++)
    {
        hmi_ledState_t state;
        if (i == HMI_LT_RED && ledState[DELETE_INDICATOR] != HMI_LS_OFF)
            state = ledState[DELETE_INDICATOR];
        else
            state = ledState[i];

        if (state == HMI_LS_ON ||
            (state == HMI_LS_BLINKSLOW && (prescale & 0x03) == 0x03) ||
            (state == HMI_LS_BLINKFAST && (prescale & 0x01) == 0x01))
            brd_EnableLed(i, true);
        else
            brd_EnableLed(i, false);
    }
}

static void hmi_TimerCallback(void *p_context)
{
    (void)p_context;

    ledHandling();
}

/* Public functions ----------------------------------------------------------*/
uint32_t hmi_Init(hmi_init_t const* pInit)
{
    if (pInit == NULL || pInit->eventHandler == NULL)
        return NRF_ERROR_NULL;

    pEventHandler = pInit->eventHandler;

    uint32_t errCode = app_timer_create(&hmi_Timer, APP_TIMER_MODE_REPEATED, &hmi_TimerCallback);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[hmi]: Timer creation failed.");

    return errCode;
}

void hmi_Execute()
{
    // led handling
    //ledHandling();
}

void hmi_SetLed(hmi_ledType_t led, hmi_ledState_t state)
{
    bool timerNeeded, isAlreadyActive = isLedActive();

    if (led > DELETE_INDICATOR || ledState[led] == state) // nothing to do
        return;

    ledState[led] = state;

    timerNeeded = isLedActive();

    if (timerNeeded != isAlreadyActive)
    {
        timerRequest(timerNeeded);

        if (!timerNeeded)
        {
            LOG_INFO("[hmi]: LED interface disabled.");
        }
        else
        {
            LOG_INFO("[hmi]: LED interface enabled.");
        }
    }

    ledHandling();
}

static void deleteResultHandler(ret_code_t errCode)
{
    if (errCode == NRF_SUCCESS)
        hmi_SetLed(DELETE_INDICATOR, HMI_LS_ON);
    else
        hmi_SetLed(DELETE_INDICATOR, HMI_LS_BLINKFAST);
    deleteIndicatorTimeout = DELETE_TIMEOUT;
}

uint32_t hmi_RequestEvent(hmi_evtRequest_t request, hmi_evtParams_t* pParams)
{
    if (pEventHandler == NULL)
        return NRF_ERROR_NULL;

    hmi_evt_t event;
    event.type = request;
    if (pParams != NULL)
        event.params = *pParams;

    if ((request == HMI_EVT_DELETEBONDS || request == HMI_EVT_FACTORYRESET) &&
        (pParams == NULL || pParams->resultHandler == NULL))
        event.params.resultHandler = deleteResultHandler;

    return (*pEventHandler)(&event);
}

#undef  NRF_LOG_LEVEL

/**END OF FILE*****************************************************************/
