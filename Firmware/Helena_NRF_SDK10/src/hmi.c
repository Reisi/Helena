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

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define ERRORPAGE ERRORPAGEHMI

#define LEDRED    (pBoardConfig->ledRed)//25
#define LEDBLUE   (pBoardConfig->ledBlue)//24
#define BUTTON    (pBoardConfig->button)

#define TIMEBASE        APP_TIMER_TICKS(10,0)
#define REPEATSTART     50
#define REPEATNEXT      150

/* Private variables ---------------------------------------------------------*/
APP_TIMER_DEF(hmi_Timer);
static uint8_t hmi_TimebaseFlag;//, hmi_TimeoutDelay;

/* Private functions ---------------------------------------------------------*/
void hmi_DebounceStart(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_in_event_disable(BUTTON);
    APP_ERROR_CHECK(app_timer_start(hmi_Timer, TIMEBASE, NULL));
    pwr_SetActiveFlag(pwr_ACTIVEHMI);
}

void hmi_TimerCallback(void *p_context)
{
    (void)p_context;

    hmi_TimebaseFlag = 1;
    pwr_SetActiveFlag(pwr_ACTIVEHMI);
}

/* Public functions ----------------------------------------------------------*/
void hmi_Init()
{
    uint32_t errCode;

    nrf_gpio_cfg_default(LEDRED);
    nrf_gpio_cfg_default(LEDBLUE);

    if (!nrf_drv_gpiote_is_init())
    {
        errCode = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(errCode);
    }
    nrf_drv_gpiote_in_config_t buttonConfig = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    buttonConfig.pull = NRF_GPIO_PIN_PULLUP;
    errCode = nrf_drv_gpiote_in_init(BUTTON, &buttonConfig, &hmi_DebounceStart);
    APP_ERROR_CHECK(errCode);
    nrf_drv_gpiote_in_event_enable(BUTTON, true);

    APP_ERROR_CHECK(app_timer_create(&hmi_Timer, APP_TIMER_MODE_REPEATED, &hmi_TimerCallback));
}

hmi_buttonState_t hmi_Debounce()
{
    static uint8_t ct0 = 0xFF, ct1 = 0xFF, rpt, but_state, but_repeat, but_press, but_long;
    uint8_t i;
    hmi_buttonState_t presstype = HMI_BUTTONNOPRESS;

    if (hmi_TimebaseFlag)
    {
        hmi_TimebaseFlag = 0;
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
        presstype = HMI_BUTTONLONG;
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
            presstype = HMI_BUTTONSHORT;
        but_press = 0;
    }
    // check if button is pressed right now
    if (but_state)
        presstype = HMI_BUTTONPRESS;

    // check for ultra long press
    i = but_state & but_repeat & but_long;
    if (i)
    {
        presstype = HMI_BUTTONULTRALONG;
        but_long = 0;
    }

    return presstype;
}

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

    /*uint32_t pinnumber = 1;
    if (Led == hmi_LEDRED)
        pinnumber = LEDRED;
    else if (Led == hmi_LEDBLUE)
        pinnumber = LEDBLUE;
    if (state == hmi_LEDTOGGLE)
        nrf_gpio_pin_toggle(pinnumber);
    else if (state == hmi_LEDON)
        nrf_gpio_pin_set(pinnumber);
    else
        nrf_gpio_pin_clear(pinnumber);*/
}

/**END OF FILE*****************************************************************/
