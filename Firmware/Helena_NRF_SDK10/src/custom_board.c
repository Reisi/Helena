/**
  ******************************************************************************
  * @file    custom_board.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/12/02
  * @brief   helena board module to distinguish between different board variants
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "custom_board.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "low_power_pwm.h"
#include "app_error.h"
#include "app_timer.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    REV20 = 0,
    REV21,
    REV22_REV23,
    REVCNT
} boardRev_t;

/* Private macros ------------------------------------------------------------*/
#define LOG(string) //SEGGER_RTT_WriteString(0, string)

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const board_config_t boardConfig[REVCNT] =
{
    {
        .button    = 27,
        .ledBlue   = 24,
        .ledRed    = 25,
        .bdepRX    = 30,
        .bdepTX    = 0,
        .analogVin = 26,
        .i2cSCL    = 29,
        .i2cSDA    = 28,
        .mpuInt    = 0xFFFFFFFF,
        .gyroOrientation = {-1,  0,  0,
                             0,  0,  1,
                             0,  1,  0},
        .hwRevStr  = "2.0"
    },
    {
        .button    = 11,
        .ledBlue   = 27,
        .ledRed    = 10,
        .bdepRX    = 15,
        .bdepTX    = 16,
        .analogVin = 6,
        .i2cSCL    = 14,
        .i2cSDA    = 13,
        .mpuInt    = 12,
        .gyroOrientation = { 1,  0,  0,
                             0,  0,  1,
                             0, -1,  0},
        .hwRevStr  = "2.1"
    },
    {
        .button    = 9,
        .ledBlue   = 13,
        .ledRed    = 13,
        .bdepRX    = 10,
        .bdepTX    = 10,
        .analogVin = 1,
        .i2cSCL    = 3,
        .i2cSDA    = 2,
        .mpuInt    = 4,
        .gyroOrientation = {-1,  0,  0,
                             0,  0,  1,
                             0,  1,  0},
        .hwRevStr  = "2.2/2.3"
    },
};
const board_config_t * pBoardConfig;

static boardRev_t boardRev = REVCNT;
APP_TIMER_DEF(pwmTimerId);
low_power_pwm_t pwm;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void findBoard()
{
    // all board variants have the SCL pin on a different pin, this can be used
    // to identify the board
    for (uint_fast8_t i = REV20; i < REVCNT; i++)
    {
        uint32_t pinState;

        nrf_gpio_cfg_input(boardConfig[i].i2cSCL, NRF_GPIO_PIN_PULLDOWN);   // activate pull down
        nrf_gpio_cfg_input(boardConfig[i].i2cSDA, NRF_GPIO_PIN_PULLDOWN);

        nrf_delay_us(50);

        pinState = nrf_gpio_pin_read(boardConfig[i].i2cSCL);

        nrf_gpio_cfg_default(boardConfig[i].i2cSCL);
        nrf_gpio_cfg_default(boardConfig[i].i2cSDA);

        if (pinState) // if the pin is still high, than the external I2C pull up has won
        {
            pBoardConfig = &boardConfig[i];
            boardRev = i;
            break;
        }
    }
}

static void dummyHandler(void* pContext)
{
    (void)pContext;
}

/* Public functions ----------------------------------------------------------*/
uint32_t board_Init()
{
    findBoard();

    if (pBoardConfig == NULL)
        return NRF_ERROR_NOT_FOUND;

    // board rev 2.2 and 2.3 use one pin for both leds. to be still able to
    // activate both leds a timer, gpiote and a ppi channel is used to toggle
    // the pin at a frequency of 200Hz
    if (boardRev == REV22_REV23)
    {
        uint32_t errCode;
        low_power_pwm_config_t pwmConfig;
        pwmConfig.active_high = false;
        pwmConfig.period = 255;
        pwmConfig.bit_mask = 1ul << pBoardConfig->ledRed;
        pwmConfig.p_timer_id = &pwmTimerId;
        errCode = low_power_pwm_init(&pwm, &pwmConfig, &dummyHandler);
        APP_ERROR_CHECK(errCode);
        nrf_gpio_cfg_default(pBoardConfig->ledRed);
    }
    // the other boards all using separate outputs for each led (active high)
    else
    {
        nrf_gpio_cfg(pBoardConfig->ledRed,
                     NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_DISCONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_D0H1,
                     NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_cfg(pBoardConfig->ledBlue,
                     NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_DISCONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_D0H1,
                     NRF_GPIO_PIN_NOSENSE);
    }

    return NRF_SUCCESS;
}

static uint32_t rev2223Handling(board_led_t led, bool enable)
{
    uint32_t errCode = NRF_SUCCESS;
    static bool ledEnabled[BOARD_LED_CNT] = {false, false};
    bool isPWMNeeded, isPWMEnabled = ledEnabled[BOARD_LED_RED] || ledEnabled[BOARD_LED_BLUE];

    ledEnabled[led] = enable;

    isPWMNeeded = ledEnabled[BOARD_LED_RED] || ledEnabled[BOARD_LED_BLUE];

    if (isPWMEnabled != isPWMNeeded)
    {
        if (isPWMNeeded)
        {
            errCode = low_power_pwm_start(&pwm, 1ul << pBoardConfig->ledRed);
            APP_ERROR_CHECK(errCode);
            errCode = low_power_pwm_duty_set(&pwm, 128);
            APP_ERROR_CHECK(errCode);
            LOG("[BOARD]: PWM enabled.\r\n");
        }
        else
        {
            errCode = low_power_pwm_stop(&pwm);
            APP_ERROR_CHECK(errCode);
            LOG("[BOARD]: PWM disabled.\r\n");
            nrf_gpio_cfg_default(pBoardConfig->ledRed);
        }
    }

    if (isPWMNeeded)
    {
        nrf_gpio_pin_drive_t pinDrive;

        if (ledEnabled[BOARD_LED_RED] && ledEnabled[BOARD_LED_BLUE])
            pinDrive = NRF_GPIO_PIN_H0H1;
        else if(ledEnabled[BOARD_LED_RED])
            pinDrive = NRF_GPIO_PIN_H0D1;
        else //if(ledEnabled[BOARD_LED_BLUE])
            pinDrive = NRF_GPIO_PIN_D0H1;

        nrf_gpio_cfg(pBoardConfig->ledRed,
             NRF_GPIO_PIN_DIR_OUTPUT,
             NRF_GPIO_PIN_INPUT_DISCONNECT,
             NRF_GPIO_PIN_NOPULL,
             pinDrive,
             NRF_GPIO_PIN_NOSENSE);
    }

    return errCode;
}

uint32_t board_EnableLed(board_led_t led, bool enable)
{
    if (boardRev == REVCNT)
        return NRF_ERROR_NOT_FOUND;

    if (boardRev == REV22_REV23)
    {
        return rev2223Handling(led, enable);
    }
    else
    {
        uint32_t pin = led == BOARD_LED_RED ? pBoardConfig->ledRed : pBoardConfig->ledBlue;
        if (enable)
            nrf_gpio_pin_set(pin);
        else
            nrf_gpio_pin_clear(pin);

        return NRF_SUCCESS;
    }
}

/**END OF FILE*****************************************************************/
