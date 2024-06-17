/**
  ******************************************************************************
  * @file    button.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
//#define BUT_LOG_ENABLED

#ifdef BUT_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BUT_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "app_timer.h"

#include "button.h"
//#include "debug.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define FREE_PIN                0xFFFFFFFF

#define CNT_UNTIL_BUTTON_STABLE 4

#define TIMEBASE                MAIN_TIMER_TICKS(10) // debouncing and evaluating timebase

#define ARRAY_SIZE(x)           (sizeof(x)/sizeof(x[0]))

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    STATE_WAITFORPINCHANGE,
    STATE_DEBOUNCEANDEVALUATE
} buttonState_t;

typedef struct
{
    uint32_t             pin;       // the pin to debounce
    but_debounceReport_t pReportHandler;// the handler to call when stable state is reached
    but_polarity_t       polarity;  // Active low or active high
    uint8_t              evalId;    // the id for evaluation, BUTTON_EVALUATE_CNT if not used
    uint8_t              cnt;       // number of times, new button state has already been different from last known
    uint8_t              lastState; // last known stable state
} debounce_t;

typedef struct
{
    but_evaluateReport_t reportHandler; // the handler to call for evaluated events
    uint16_t longClickCnt;          // the number of counts needed to recognize as long click
    uint16_t StillPressedPeriod;    // the period BUTTON_IS_STILL_PRESSED events are reported with
    uint16_t abortCnt;              // the number of counts when evaluating should be aborted
    uint16_t cnt;                   // the current counter state, is also used as indicator (0 = not pressed, !0 = pressed)
} evaluate_t;

/* Private variables ---------------------------------------------------------*/
static bool          isInit;            // indicator if module has been initialized
static buttonState_t state;             // current state (polling or waiting for interrupt)
APP_TIMER_DEF(butTimer);                // board module timer
#if BUTTON_DEBOUNCE_CNT > 0
static debounce_t    debounce[BUTTON_DEBOUNCE_CNT];
#endif
#if BUTTON_EVALUATE_CNT > 0
static evaluate_t    evaluate[BUTTON_EVALUATE_CNT];
#endif

/* Private functions ---------------------------------------------------------*/
/** @brief function to get the current button state
 */
static uint8_t getButtonState(debounce_t const* pDeb)
{
    if (pDeb == NULL)
        return BUTTON_RELEASED; // just failsafe

    uint32_t pinState = nrf_gpio_pin_read(pDeb->pin);
    if (pDeb->polarity == BUTTON_ACTIVE_LOW)
        return pinState ? BUTTON_RELEASED : BUTTON_PRESSED;
    else
        return pinState ? BUTTON_PRESSED : BUTTON_RELEASED;
}

/** @brief function to change state between polling (activ) and waiting for interrupt (inactive)
 */
static void setState(buttonState_t newState)
{
    if (state == newState)
    {
        LOG_ERROR("[but]: already in requested button state");
        return;
    }

    if (newState == STATE_DEBOUNCEANDEVALUATE)
    {
        // disable pin change interrupts
        for (uint_fast8_t i = 0; i < ARRAY_SIZE(debounce); i++)
        {
            if (debounce[i].pin != FREE_PIN)
                nrf_drv_gpiote_in_event_disable(debounce[i].pin);
        }
        // start timer
        ret_code_t errCode = app_timer_start(butTimer, TIMEBASE, NULL);
        APP_ERROR_CHECK(errCode);

        LOG_INFO("[but]: debouncing and evaluating");
    }
    else
    {
        // stop timer
        ret_code_t errCode = app_timer_stop(butTimer);
        APP_ERROR_CHECK(errCode);

        // enable pin change interrupts
        for (uint_fast8_t i = 0; i < ARRAY_SIZE(debounce); i++)
        {
            if (debounce[i].pin != FREE_PIN)
                nrf_drv_gpiote_in_event_enable(debounce[i].pin, true);
        }
        LOG_INFO("[but]: waiting for pin change");
    }

    state = newState;
}

/** @brief debouncing function
 * @return true if at least one button is still bouncing
 */
static bool debouncing()
{
    bool debouncing = false;

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(debounce); i++)
    {
        if (debounce[i].pin == FREE_PIN)
            continue;   // ignore unused array

        uint8_t newState = getButtonState(&debounce[i]);
        if (newState == debounce[i].lastState)
            continue;   // ignore stable pins

        if (debounce[i].cnt++ >= CNT_UNTIL_BUTTON_STABLE)
        {
            debounce[i].lastState = newState;
            debounce[i].cnt = 0;
            if (debounce[i].pReportHandler)
                debounce[i].pReportHandler(debounce[i].pin, newState);
            if (debounce[i].evalId <= BUTTON_EVALUATE_CNT)
                (void)but_EvaluateButtonChange(debounce[i].evalId, newState);
        }
        else
            debouncing = true;
    }

    return debouncing;
}

/** @brief evaluation function
 * @return true if at least one button is still being evaluated
 */
static bool evaluating()
{
    bool evaluating = false;

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(evaluate); i++)
    {
        if (evaluate[i].reportHandler == NULL || evaluate[i].cnt == 0)
            continue;   // ignore unused and those not evaluating

        if (evaluate[i].cnt == evaluate[i].abortCnt)
        {   // send abort event and stop evaluating
            evaluate[i].reportHandler(i, BUTTON_ABORT, evaluate[i].abortCnt);
            evaluate[i].cnt = 0;
        }
        else if(evaluate[i].cnt % evaluate[i].StillPressedPeriod == 0)
            evaluate[i].reportHandler(i, BUTTON_IS_STILL_PRESSED, evaluate[i].cnt);

        if (evaluate[i].cnt)
        {
            evaluating = true;
            evaluate[i].cnt++;
        }
    }

    return evaluating;
}

/** @brief polling timer interrupt handler
 */
static void timerCallback(void *p_context)
{
    (void)p_context;

    bool isDebouncing = debouncing();
    bool isEvaluating = evaluating();

    if (!isDebouncing && !isEvaluating)
        setState(STATE_WAITFORPINCHANGE);
}

/** @brief function to get the debounce structure for a specific pin (or a empty one)
 */
static debounce_t* getDebounce(uint32_t pin)
{
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(debounce); i++)
    {
        if (debounce[i].pin == pin)
            return &debounce[i];
    }
    return NULL;
}

/** @brief function to get the id for a free evaluation structure
 */
static uint8_t getFreeEvaluate()
{
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(evaluate); i++)
    {
        if (evaluate[i].reportHandler == NULL)
            return i;
    }
    return ARRAY_SIZE(evaluate);
}

/** @brief the pin change interrupt handler
 */
static void pinChangeHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)action;

    debounce_t* pDebounce = getDebounce(pin);
    if (pDebounce == NULL) // this should not happen under normal conditions
    {
        LOG_ERROR("[but]: pin change for unknown pin received");
        return;
    }

    // if pin state has changed, switch to polling mode
    uint8_t buttonState = getButtonState(pDebounce);
    if (pDebounce->lastState == buttonState)
        return; // no change, should normally not happen, but maybe during debugging

    setState(STATE_DEBOUNCEANDEVALUATE);
}

/** @brief function ti initialize button module
 */
static ret_code_t init()
{
    ret_code_t errCode = NRF_SUCCESS;

#if BUTTON_DEBOUNCE_CNT > 0
    // the gpiote module might already be initialized by another module, so just check
    if (!nrf_drv_gpiote_is_init())
    {
        errCode = nrf_drv_gpiote_init();
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[but]: GPIOTE init failed with error %d.", errCode);
            return errCode;
        }
    }
    // set default values for all debounce structures
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(debounce); i++)
    {
        debounce[i].pin = FREE_PIN;
        debounce[i].evalId = BUTTON_EVALUATE_CNT;
    }
#endif // BUT_DEBOUNCE_CNT

#if (BUTTON_DEBOUNCE_CNT > 0) || (BUTTON_EVALUATE_CNT > 0)
    errCode = app_timer_create(&butTimer, APP_TIMER_MODE_REPEATED, &timerCallback);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[but]: timer creation error %d.", errCode);
        return errCode;
    }
#endif

    isInit = true;

    return errCode;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t but_DebounceInit(but_debounceInit_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL)
        return NRF_ERROR_NULL;

    if (!isInit)
        init();

    debounce_t* pDebounce = getDebounce(FREE_PIN);
    if (pDebounce == NULL)
        return NRF_ERROR_NO_MEM;

    pDebounce->pin = pInit->pinNo;
    pDebounce->polarity = pInit->polarity;
    pDebounce->pReportHandler = pInit->pReportHandler;
    pDebounce->evalId = pInit->evalId;

    // configure button pin
    nrf_drv_gpiote_in_config_t buttonConfig = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    buttonConfig.pull = pInit->polarity == BUTTON_ACTIVE_LOW ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN;
    errCode = nrf_drv_gpiote_in_init(pInit->pinNo, &buttonConfig, &pinChangeHandler);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[but]: GPIOTE input pin init failed with error %d.", errCode);
        return errCode;
    }
    nrf_drv_gpiote_in_event_enable(pInit->pinNo, true);

    return errCode;
}

uint8_t but_EvaluateInit(but_evaluateInit_t const* pInit)
{
    if (pInit == NULL || pInit->reportHandler == NULL)
        return ARRAY_SIZE(evaluate);

    if (!isInit)
        init();

    uint8_t evalId = getFreeEvaluate();
    if (evalId < ARRAY_SIZE(evaluate))
    {
        evaluate[evalId].reportHandler = pInit->reportHandler;
        evaluate[evalId].longClickCnt = pInit->longClickCnt;
        evaluate[evalId].StillPressedPeriod = pInit->StillPressedPeriod;
        evaluate[evalId].abortCnt = pInit->abortCnt;
    }

    return evalId;
}

ret_code_t but_EvaluateButtonChange(uint8_t evalId, uint8_t newState)
{
    if (evalId >= ARRAY_SIZE(evaluate) || evaluate[evalId].reportHandler == NULL)
        return NRF_ERROR_INVALID_PARAM;

    // button has been pressed, send event and start evaluating
    if (evaluate[evalId].cnt == 0 && newState == BUTTON_PRESSED)
    {
        evaluate[evalId].reportHandler(evalId, BUTTON_IS_PRESSED, evaluate[evalId].cnt);

        evaluate[evalId].cnt = 1;
        if (state != STATE_DEBOUNCEANDEVALUATE)
            setState(STATE_DEBOUNCEANDEVALUATE);
    }
    // button has been released, send event
    else if (evaluate[evalId].cnt != 0 && newState == BUTTON_RELEASED)
    {
        if (evaluate[evalId].cnt > evaluate[evalId].StillPressedPeriod)
            evaluate[evalId].reportHandler(evalId, BUTTON_IS_RELEASED, evaluate[evalId].cnt);
        else if (evaluate[evalId].cnt > evaluate[evalId].longClickCnt)
            evaluate[evalId].reportHandler(evalId, BUTTON_CLICKLONG, evaluate[evalId].cnt);
        else
            evaluate[evalId].reportHandler(evalId, BUTTON_CLICKSHORT, evaluate[evalId].cnt);

        evaluate[evalId].cnt = 0;
    }

    return NRF_SUCCESS;
}


/**END OF FILE*****************************************************************/
