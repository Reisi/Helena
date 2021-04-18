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

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    BUTEV_INTERNAL = 0,
    BUTEV_XIA_BIG,
    BUTEV_XIA_SEC,
    BUTEV_CNT
} buttonEvaluated_t;

typedef struct
{
    uint8_t  state;             // last known state
    uint32_t timestampPressed;  // timestamp of last pressed event
} buttonEvaluation_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define LEDRED              (pBoardConfig->ledRed)//25
#define LEDBLUE             (pBoardConfig->ledBlue)//24
#define BUTTON              (pBoardConfig->button)

#define TIMEBASE            APP_TIMER_TICKS(10,0)
#define BUTTONPRESS_LONG    APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)
#define BUTTONPRESS_HOLD    APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)

#define BUTTON_PRESSED      1
#define BUTTON_RELEASED     0

/* Private variables ---------------------------------------------------------*/
APP_TIMER_DEF(hmi_Timer);
static volatile uint8_t timebaseFlag;               // flag indicating that a timer interrupt occured
static volatile bool isDebouncingActive;
static buttonEvaluation_t buttonState[BUTEV_CNT];   // button structure for evaluating hold, short and long clicks
static btle_hidEventType_t lastHidEvent = BTLE_EVT_HID_CNT;

static hmi_eventHandler_t eventHandler;

/* Private functions ---------------------------------------------------------*/
static void hmi_DebounceStart(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin;
    (void)action;

    nrf_drv_gpiote_in_event_disable(BUTTON);                    // disable event, from now on everything is done in the timer handler
    APP_ERROR_CHECK(app_timer_start(hmi_Timer, TIMEBASE, NULL));
    isDebouncingActive = true;
    pwr_SetActiveFlag(pwr_ACTIVEHMI);
}

static void hmi_TimerCallback(void *p_context)
{
    (void)p_context;

    timebaseFlag++;// = 1;
    pwr_SetActiveFlag(pwr_ACTIVEHMI);
}

static void internalButtonDebounce(uint8_t* pOldState, uint8_t newState)
{
    static uint8_t intCnt;

    if (newState != *pOldState)
    {
        if (++intCnt >= 4)
        {
            *pOldState = newState;
            buttonState[BUTEV_INTERNAL].state = newState;
            intCnt = 0;
        }
    }
    else
        intCnt = 0;
}

static void evaluateButtons()
{
    for (uint_fast8_t i = BUTEV_INTERNAL; i < BUTEV_CNT; i++)
    {
        hmi_eventType_t evt = i * HMI_EVT_XIAOMI_PRI_SHORT;
        uint32_t timePressed;
        (void)app_timer_cnt_get(&timePressed);
        (void)app_timer_cnt_diff_compute(timePressed, buttonState[i].timestampPressed, &timePressed);

        // button is still pressed, check for ultralong
        if (buttonState[i].state == BUTTON_PRESSED && buttonState[i].timestampPressed != 0)
        {
            if (timePressed >= BUTTONPRESS_HOLD)
            {
                eventHandler(evt + HMI_EVT_INTERNAL_HOLD);
                buttonState[i].state = BUTTON_RELEASED;
                buttonState[i].timestampPressed = 0;
            }
        }
        // button has just been released, evaluate long or short
        if (buttonState[i].state == BUTTON_RELEASED && buttonState[i].timestampPressed != 0)
        {
            if (timePressed < BUTTONPRESS_LONG)
                eventHandler(evt + HMI_EVT_INTERNAL_SHORT);
            else if (timePressed < BUTTONPRESS_HOLD)
                eventHandler(evt + HMI_EVT_INTERNAL_LONG);
            buttonState[i].timestampPressed = 0;
        }
        // button has just been pressed, save timestamp
        if (buttonState[i].state == BUTTON_PRESSED && buttonState[i].timestampPressed == 0)
            (void)app_timer_cnt_get(&buttonState[i].timestampPressed);

    }
}

/* Public functions ----------------------------------------------------------*/
uint32_t hmi_Init(hmi_init_t const* pInit)
{
    uint32_t errCode;

    if (pInit == NULL)
        return NRF_ERROR_NULL;

    eventHandler = pInit->eventHandler;

    nrf_gpio_cfg_default(LEDRED);
    nrf_gpio_cfg_default(LEDBLUE);

    if (!nrf_drv_gpiote_is_init())
    {
        errCode = nrf_drv_gpiote_init();
        if (errCode != NRF_SUCCESS)
            return errCode;
    }
    nrf_drv_gpiote_in_config_t buttonConfig = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
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

    // the other stuff is timebased
    if (!timebaseFlag)
        return; // nothing to do

    timebaseFlag = 0;
    pwr_ClearActiveFlag(pwr_ACTIVEHMI);

    // debouncing for internal button
    static uint8_t lastState;
    uint8_t inputState = nrf_gpio_pin_read(BUTTON) ? BUTTON_RELEASED : BUTTON_PRESSED;
    internalButtonDebounce(&lastState, inputState);

    // no handler, evaluation senseless
    if (eventHandler == NULL)
        return;

    // evaluation button states
    evaluateButtons();

    // check if timer is still necessary
    bool evaluationOngoing = (bool)lastState;   // us last state from debounce routine for internal button,
                                                // otherwise clicks after a hold event will be missed
    for (uint_fast8_t i = BUTEV_INTERNAL; i < BUTEV_CNT; i++)
    {
        if (buttonState[i].state == BUTTON_PRESSED)
            evaluationOngoing = true;
    }
    if (!evaluationOngoing)
    {
        APP_ERROR_CHECK(app_timer_stop(hmi_Timer));
        nrf_drv_gpiote_in_event_enable(BUTTON, true);
        isDebouncingActive = false;
    }
}

uint32_t hmi_ReportHidEvent(btle_hidEventType_t hidEvent)
{
    if (hidEvent >= BTLE_EVT_HID_CNT)
        return NRF_ERROR_INVALID_PARAM;

    switch (hidEvent)
    {
    case BTLE_EVT_HID_XIA_MAIN_PRESSED:
        if (!isDebouncingActive)
            hmi_DebounceStart(0, 0);    // input values are ignored, so don't care
        buttonState[BUTEV_XIA_BIG].state = BUTTON_PRESSED;
        break;
    case BTLE_EVT_HID_XIA_MAIN_RELEASED:
        buttonState[BUTEV_XIA_BIG].state = BUTTON_RELEASED;
        break;
    case BTLE_EVT_HID_XIA_SEC_PRESSED:
        if (!isDebouncingActive)
            hmi_DebounceStart(0, 0);    // input values are ignored, so don't care
        buttonState[BUTEV_XIA_SEC].state = BUTTON_PRESSED;
        break;
    case BTLE_EVT_HID_XIA_SEC_RELEASED:
        buttonState[BUTEV_XIA_SEC].state = BUTTON_RELEASED;
        break;
    default:
        lastHidEvent = hidEvent;
    }

    return NRF_SUCCESS;
}

/*uint32_t hmi_Debounce(hmi_buttonState_t * buttonReport, uint16_t countOf)
{
    static uint8_t ct0 = 0xFF, ct1 = 0xFF, rpt, but_state, but_repeat, but_press, but_long;
    uint8_t i;

    if (countOf < HMI_BT_CNT)
        return NRF_ERROR_NO_MEM;

    //hmi_buttonState_t presstype = HMI_BS_NOPRESS;

    if (timebaseFlag)
    {
        timebaseFlag = 0;
        pwr_ClearActiveFlag(pwr_ACTIVEHMI);
        i = nrf_gpio_pin_read(BUTTON);
        i = but_state ^ ~i;
        ct0 = ~(ct0 & i);
        ct1 = ct0 ^(ct1 & i);
        i &= ct0 & ct1;
        but_state ^= (i & 1);
        but_press |= but_state & i;
        if ((but_state & 1) == 0)
            rpt = REPEATSTART;
        if (rpt)
        {
            if (--rpt == 0)
            {
                if (but_long)
                {
                    but_repeat |= but_state & 1;
                }
                else
                {
                    rpt = REPEATNEXT;
                    but_long |= but_state & 1;
                }
            }
        }

        if ((ct0 & ct1) == 0xFF && (but_state & 1) == 0)
        {
            APP_ERROR_CHECK(app_timer_stop(hmi_Timer));
            nrf_drv_gpiote_in_event_enable(BUTTON, true);
        }
    }

    // check if button was long pressed
    i = ~but_state & but_long;
    if (i)
    {
        buttonStates[HMI_BT_INTERNAL] = HMI_BS_LONG;
        but_long = 0;
        but_press = 0;
    }
    // check if button was short pressed
    i = ~but_state & but_press;
    if (i)
    {
        if (but_repeat)
            but_repeat = 0;
        else
            buttonStates[HMI_BT_INTERNAL] = HMI_BS_SHORT;
        but_press = 0;
    }
    // check if button is pressed right now
    if (but_state)
        buttonStates[HMI_BT_INTERNAL] = HMI_BS_PRESS;

    // check for ultra long press
    i = but_state & but_repeat & but_long;
    if (i)
    {
        buttonStates[HMI_BT_INTERNAL] = HMI_BS_ULTRALONG;
        but_long = 0;
    }

    for (hmi_buttonType_t i = 0; i < HMI_BT_CNT; i++)
    {
        buttonReport[i] = buttonStates[i];
        buttonStates[i] = HMI_BS_NOPRESS;
    }

    return NRF_SUCCESS;
}

uint32_t hmi_ReportButton(hmi_buttonType_t type, hmi_buttonState_t state)
{
    if (type == HMI_BT_INTERNAL || type >= HMI_BT_CNT)
        return NRF_ERROR_INVALID_PARAM;

    buttonStates[type] = state;

    return NRF_SUCCESS;
}*/

void hmi_SetLed(hmi_ledType_t led, hmi_ledState_t state)
{
    static uint8_t ledState;

    switch (state)
    {
    case HMI_LEDOFF:
        ledState &= ~(1<<led);
        break;
    case HMI_LEDON:
        ledState |= (1<<led);
        break;
    case HMI_LEDTOGGLE:
        ledState ^= (1<<led);
        break;
    }

    switch (ledState & ((1<<HMI_LEDBLUE)|(1<<HMI_LEDRED)))
    {
    case 0:
        nrf_gpio_cfg_default(LEDRED);
        nrf_gpio_cfg_default(LEDBLUE);
        break;
    case (1<<HMI_LEDRED):
        nrf_gpio_cfg_default(LEDBLUE);
        nrf_gpio_cfg_output(LEDRED);
        if (LEDRED == LEDBLUE)
            nrf_gpio_pin_clear(LEDRED);
        else
            nrf_gpio_pin_set(LEDRED);
        break;
    case (1<<HMI_LEDBLUE):
        nrf_gpio_cfg_default(LEDRED);
        nrf_gpio_cfg_output(LEDBLUE);
        nrf_gpio_pin_set(LEDBLUE);
        break;
    case (1<<HMI_LEDRED)|(1<<HMI_LEDBLUE):
        nrf_gpio_cfg_output(LEDBLUE);
        nrf_gpio_pin_set(LEDRED);
        nrf_gpio_cfg_output(LEDRED);
        if (LEDRED == LEDBLUE)
            nrf_gpio_pin_clear(LEDRED);
        else
            nrf_gpio_pin_set(LEDRED);
        break;
    }
}

/**END OF FILE*****************************************************************/
