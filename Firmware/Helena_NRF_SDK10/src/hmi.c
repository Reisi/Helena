/**
  ******************************************************************************
  * @file    hmi.c
  * @author  RT
  * @version V1.0
  * @date    15/02/08
  * @brief   human machine interface module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "hmi.h"
#include "power.h"
#include "custom_board.h"
#include "main.h"
#include "app_util_platform.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/**< structure to hold debounce status for internal button(s) */
typedef struct
{
    uint8_t cnt;        // number of times, new button state has already been different from last known
    uint8_t lastState;  // last known stable state
} buttonDebounce_t;

/**< buttons for short, long click, hold evaluation */
typedef enum
{
    BUTEV_INTERNAL = 0,
    BUTEV_XIA_BIG,
    BUTEV_XIA_SEC,
    BUTEV_CNT
} buttonEvaluated_t;

/**< the already reported state for a button */
typedef enum
{
    BUTREP_HOLDNONE = 0,
    BUTREP_HOLD2SEC,
    BUTREP_HOLD10SEC
} buttonHoldReported_t;

/**< structure to hold data for evaluation buttons */
typedef struct
{
    uint8_t              state;             // last known state
    buttonHoldReported_t reported;          // the already reported hold state
    uint32_t             timestampPressed;  // timestamp of last pressed event
} buttonEvaluation_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define BUTTON                  (pBoardConfig->button)

#define TIMEBASE                APP_TIMER_TICKS(10,0)
#define BUTTONPRESS_LONG        APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)
#define BUTTONPRESS_HOLD2SEC    APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
#define BUTTONPRESS_HOLD10SEC   APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)
#define BUTTONPRESS_HOLDABORT   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)

#define BUTTON_PRESSED          1
#define BUTTON_RELEASED         0
#define CNT_UNTIL_BUTTON_STABLE 4

#define LOG(string) //SEGGER_RTT_WriteString(0, string)

/* Private variables ---------------------------------------------------------*/
APP_TIMER_DEF(hmi_Timer);                           // timer for debouncing, evaluation and led management
static volatile uint8_t timebaseCnt;                // counter increased in timer handler

static buttonDebounce_t buttonDebounce;             // debounce data for internal button
static buttonEvaluation_t buttonState[BUTEV_CNT];   // button structure for evaluating hold, short and long clicks
static btle_hidEventType_t lastHidEvent = BTLE_EVT_HID_CNT; // last received hid event related to R51 remote

static hmi_ledState_t ledState[HMI_LEDCNT];         // state of status leds

static hmi_eventHandler_t eventHandler;             // event handler to report button related events

/* Private functions ---------------------------------------------------------*/
static bool isLedActive()
{
    return ledState[HMI_LEDRED] != HMI_LEDOFF || ledState[HMI_LEDBLUE] != HMI_LEDOFF;
}

static bool isDebouncingActive()
{
    return buttonDebounce.cnt != 0;
}

static void timerRequest(bool request)
{
    static uint8_t cnt; // request counter

    if (!request && --cnt == 0)
    {
        APP_ERROR_CHECK(app_timer_stop(hmi_Timer));
        LOG("[HMI]: Timer stopped.\r\n");
        nrf_drv_gpiote_in_event_enable(BUTTON, true);
    }
    else if (request && cnt++ == 0)
    {
        APP_ERROR_CHECK(app_timer_start(hmi_Timer, TIMEBASE, NULL));
        LOG("[HMI]: Timer started.\r\n");
        nrf_drv_gpiote_in_event_disable(BUTTON);
    }
}

static void hmi_TimerCallback(void *p_context)
{
    (void)p_context;

    timebaseCnt++;
    pwr_SetActiveFlag(pwr_ACTIVEHMI);   // set flag to ensure hmi_execute is called before entering standby
}

static void hmi_DebounceStart(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin;
    (void)action;

    // there might be an pending event when activating, ignore it if state has not changed
    uint8_t inputState = nrf_gpio_pin_read(BUTTON) ? BUTTON_RELEASED : BUTTON_PRESSED;
    if (inputState == buttonDebounce.lastState)
        return;

    LOG("[HMI]: Debouncing started from interrupt.\r\n");

    timerRequest(true);
    buttonDebounce.cnt = 1;
    pwr_SetActiveFlag(pwr_ACTIVEHMI);   // no standby to ensure execution of hmi_execute
}

static void internalButtonDebounce(uint8_t newState)
{
    if (newState != buttonDebounce.lastState)
    {
        if (buttonDebounce.cnt++ == 0)                      // changed from a stable state
        {
            LOG("[HMI]: Debouncing started from timer.\r\n");
            timerRequest(true);                             // timer is necessary
        }
        else if (buttonDebounce.cnt >= CNT_UNTIL_BUTTON_STABLE)
        {                                                   // button has reached stable state
            buttonDebounce.lastState = newState;            // update new debounce state
            buttonState[BUTEV_INTERNAL].state = newState;   // update new evaluation state
            buttonDebounce.cnt = 0;                         // reset counter, to indicate that debouncing is finished
            timerRequest(false);                            // timer can be released
            LOG("[HMI]: Debouncing finished.\r\n");
        }
    }
}

static void evaluateButtons()
{
    static bool timerRequested;
    bool evaluating = false;

    for (uint_fast8_t i = BUTEV_INTERNAL; i < BUTEV_CNT; i++)
    {
        hmi_eventType_t evt = i * HMI_EVT_XIAOMI_PRI_SHORT;
        uint32_t timePressed;
        (void)app_timer_cnt_get(&timePressed);
        (void)app_timer_cnt_diff_compute(timePressed, buttonState[i].timestampPressed, &timePressed);

        // button is still pressed, check for ultralong
        if (buttonState[i].state == BUTTON_PRESSED && buttonState[i].timestampPressed != 0)
        {
            if (buttonState[i].reported == BUTREP_HOLDNONE && timePressed >= BUTTONPRESS_HOLD2SEC)
            {
                LOG("[HMI]: button hold for 2sec.\r\n");
                eventHandler(evt + HMI_EVT_INTERNAL_HOLD2SEC);
                buttonState[i].reported = BUTREP_HOLD2SEC;
            }
            if (buttonState[i].reported == BUTREP_HOLD2SEC && timePressed >= BUTTONPRESS_HOLD10SEC)
            {
                LOG("[HMI]: button hold for 10sec.\r\n");
                eventHandler(evt + HMI_EVT_INTERNAL_HOLD10SEC);
                buttonState[i].reported = BUTREP_HOLD10SEC;
            }
            if (buttonState[i].reported == BUTREP_HOLD10SEC && timePressed >= BUTTONPRESS_HOLDABORT)
            {
                LOG("[HMI]: button hold for 30sec, abort evaluation\r\n");
                buttonState[i].state = BUTTON_RELEASED;
                buttonState[i].timestampPressed = 0;
            }
        }
        // button has just been released, evaluate long or short
        if (buttonState[i].state == BUTTON_RELEASED && buttonState[i].timestampPressed != 0)
        {
            if (timePressed < BUTTONPRESS_LONG)
            {
                LOG("[HMI]: button clicked (short).\r\n");
                eventHandler(evt + HMI_EVT_INTERNAL_SHORT);
            }
            else if (timePressed < BUTTONPRESS_HOLD2SEC)
            {
                LOG("[HMI]: button clicked (long).\r\n");
                eventHandler(evt + HMI_EVT_INTERNAL_LONG);
            }
            else //if (timePressed >= BUTTONPRESS_HOLD2SEC)
            {
                LOG("[HMI]: button released.\r\n");
                eventHandler(evt + HMI_EVT_INTERNAL_HOLDRELEASED);
            }
            buttonState[i].timestampPressed = 0;
        }
        // button has just been pressed, save timestamp
        if (buttonState[i].state == BUTTON_PRESSED && buttonState[i].timestampPressed == 0)
        {
            (void)app_timer_cnt_get(&buttonState[i].timestampPressed);
            buttonState[i].reported = BUTREP_HOLDNONE;
        }

        if (buttonState[i].state == BUTTON_PRESSED)
        {
            evaluating = true;
        }
    }

    if (evaluating != timerRequested)
    {
        timerRequested = evaluating;
        if (evaluating)
        {
            LOG("[HMI]: Button evaluating enabled.\r\n");
            timerRequest(evaluating);
        }
        else
        {
            timerRequest(evaluating);
            LOG("[HMI]: Button evaluating disabled.\r\n");
        }
    }
}

static void ledHandling()
{
    uint8_t prescale = timebaseCnt;

    if (ledState[HMI_LEDBLUE] == HMI_LEDON ||
    (ledState[HMI_LEDBLUE] == HMI_LEDBLINKSLOW && (prescale & 0x60) == 0x60) || /// TODO: replace magic numbers
    (ledState[HMI_LEDBLUE] == HMI_LEDBLINKFAST && (prescale & 0x20) == 0x20))
        board_EnableLed(BOARD_LED_BLUE, true);
    else
        board_EnableLed(BOARD_LED_BLUE, false);

    if (ledState[HMI_LEDRED] == HMI_LEDON ||
    (ledState[HMI_LEDRED] == HMI_LEDBLINKSLOW && (prescale & 0x60) == 0x60) ||
    (ledState[HMI_LEDRED] == HMI_LEDBLINKFAST && (prescale & 0x20) == 0x20))
        board_EnableLed(HMI_LEDRED, true);
    else
        board_EnableLed(HMI_LEDRED, false);
}

/* Public functions ----------------------------------------------------------*/
uint32_t hmi_Init(hmi_init_t const* pInit)
{
    uint32_t errCode;

    if (pInit == NULL)
        return NRF_ERROR_NULL;

    eventHandler = pInit->eventHandler;

    board_EnableLed(BOARD_LED_BLUE, false);
    board_EnableLed(BOARD_LED_RED, false);

    if (!nrf_drv_gpiote_is_init())
    {
        errCode = nrf_drv_gpiote_init();
        if (errCode != NRF_SUCCESS)
            return errCode;
    }
    nrf_drv_gpiote_in_config_t buttonConfig = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    buttonConfig.pull = NRF_GPIO_PIN_PULLUP;
    errCode = nrf_drv_gpiote_in_init(BUTTON, &buttonConfig, &hmi_DebounceStart);
    if (errCode != NRF_SUCCESS)
        return errCode;

    nrf_drv_gpiote_in_event_enable(BUTTON, true);

    return app_timer_create(&hmi_Timer, APP_TIMER_MODE_REPEATED, &hmi_TimerCallback);
}

void hmi_Execute()
{
    // check if R51 events are pending
    if (lastHidEvent != BTLE_EVT_HID_CNT)
    {
        hmi_eventType_t evt = HMI_EVT_R51_PLAYPAUSE;
        evt += lastHidEvent - BTLE_EVT_HID_R51_PLAYPAUSE;
        eventHandler(evt);
        lastHidEvent = BTLE_EVT_HID_CNT;
    }

    static uint8_t lastCnt;

    if (timebaseCnt == lastCnt)
        return;

    lastCnt = timebaseCnt;
    pwr_ClearActiveFlag(pwr_ACTIVEHMI);

    // led handling
    ledHandling();

    // debouncing for internal button
    uint8_t inputState = nrf_gpio_pin_read(BUTTON) ? BUTTON_RELEASED : BUTTON_PRESSED;
    internalButtonDebounce(inputState);

    // evaluation button states
    evaluateButtons();
}

uint32_t hmi_ReportHidEvent(btle_hidEventType_t hidEvent)
{
    if (hidEvent >= BTLE_EVT_HID_CNT)
        return NRF_ERROR_INVALID_PARAM;

    switch (hidEvent)
    {
    case BTLE_EVT_HID_XIA_MAIN_PRESSED:
        buttonState[BUTEV_XIA_BIG].state = BUTTON_PRESSED;
        break;
    case BTLE_EVT_HID_XIA_MAIN_RELEASED:
        buttonState[BUTEV_XIA_BIG].state = BUTTON_RELEASED;
        break;
    case BTLE_EVT_HID_XIA_SEC_PRESSED:
        buttonState[BUTEV_XIA_SEC].state = BUTTON_PRESSED;
        break;
    case BTLE_EVT_HID_XIA_SEC_RELEASED:
        buttonState[BUTEV_XIA_SEC].state = BUTTON_RELEASED;
        break;
    default:
        lastHidEvent = hidEvent;    // R51 events are directly reported in execute
        return NRF_SUCCESS;
    }

    evaluateButtons();

    return NRF_SUCCESS;
}

void hmi_SetLed(hmi_ledType_t led, hmi_ledState_t state)
{
    bool timerNeeded, isAlreadyActive = isLedActive();

    ledState[led] = state;

    timerNeeded = isLedActive();

    if (timerNeeded != isAlreadyActive)
    {
        if (timerNeeded)
        {
            LOG("[HMI]: LED interface enabled.\r\n");
            timerRequest(timerNeeded);
        }
        else
        {
            board_EnableLed(BOARD_LED_BLUE, false); // disable manually, polling only works when timer is active
            board_EnableLed(BOARD_LED_RED, false);
            timerRequest(timerNeeded);
            LOG("[HMI]: LED interface disabled.\r\n");
        }
    }
}

/**END OF FILE*****************************************************************/
