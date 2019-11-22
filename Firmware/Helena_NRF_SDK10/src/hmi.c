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
static hmi_buttonState_t buttonStates[HMI_BT_CNT];

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

uint32_t hmi_Debounce(hmi_buttonState_t * buttonReport, uint16_t countOf)
{
    static uint8_t ct0 = 0xFF, ct1 = 0xFF, rpt, but_state, but_repeat, but_press, but_long;
    uint8_t i;

    if (countOf < HMI_BT_CNT)
        return NRF_ERROR_NO_MEM;

    //hmi_buttonState_t presstype = HMI_BS_NOPRESS;

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
}

/**END OF FILE*****************************************************************/
