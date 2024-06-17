/**
  ******************************************************************************
  * @file    hmi.c
  * @author  Thomas Reisnecker
  * @brief   ?
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hmi.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "sdk_errors.h"
#include "nrf_nvic.h"
#include "nrf_nvmc.h"
#include "app_util.h"
#include "app_error.h"
#include <stddef.h>

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint16_t id;
    uint16_t rev;
} device_info_t;

typedef struct
{
    uint32_t scl;
    uint32_t buttonPin;
    uint32_t ledRed;
    uint32_t ledGreen;
    uint32_t ledBlue;
} hmi_info_t;

typedef struct
{
    bool red   : 1;
    bool green : 1;
    bool blue  : 1;
} ledState_t;

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)   (sizeof(x)/sizeof(x[0]))

/* Private defines -----------------------------------------------------------*/
#define LEDNA           0xFF

#define BUTTONDFU       11
#define LEDADV          10
#define LEDCON          27
#define LEDUPD          LEDNA

#define UICR_DEVINFO    0x10001080  // the address, where the device info is stored

#define DEVICE_INFO     \
{                       \
    .id = 0x8724,       \
    .rev = 0x0200       \
}

/* Private variables ---------------------------------------------------------*/
volatile device_info_t deviceInfo __attribute__ ((section(".uicrDevInfo"))) = DEVICE_INFO;

static hmi_info_t const hmiInfo[] =
{
    {
        .scl = 14,
        .buttonPin = 11,
        .ledRed = 10,
        .ledGreen = LEDNA,
        .ledBlue = 27,
    },
    {
        .scl = 3,
        .buttonPin = 9,
        .ledRed = 13,
        .ledGreen = LEDNA,
        .ledBlue = 13,
    },
};

static hmi_info_t const* pHmiInfo;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static hmi_info_t const* findBoard()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(hmiInfo); i++)
    {
        uint32_t pinState;

        nrf_gpio_cfg_input(hmiInfo[i].scl, NRF_GPIO_PIN_PULLDOWN);   // activate pull down

        nrf_delay_us(50);

        pinState = nrf_gpio_pin_read(hmiInfo[i].scl);

        nrf_gpio_cfg_default(hmiInfo[i].scl);

        if (pinState) // if the pin is still high, than the external I2C pull up has won
            return &hmiInfo[i];
    }

    return NULL;
}

/** @brief function for checking and setting the bootloader device info
 *  @note  These informations are normally programmed with the bootloader, but
 *         I forgot to implement in the old bootloader.
 * @return NRF_SUCCESS or a reset
 */
static ret_code_t setBootloaderProtection()
{
    device_info_t const* pDevInfo = (device_info_t const*)UICR_DEVINFO;
    device_info_t const devInfo = DEVICE_INFO;
    ret_code_t errCode = NRF_SUCCESS;

    // check if device info is already set
    if (pDevInfo->id == devInfo.id && pDevInfo->rev == devInfo.rev)
        return NRF_SUCCESS;

    // check if empty
    if (pDevInfo->id == 0xFFFF && pDevInfo->rev == 0xFFFF)
    {
        uint8_t enabled;
        (void)sd_softdevice_is_enabled(&enabled);

        // not possible to write to UICR while softdevice is enabled, so disable first
        if (enabled)
            errCode = sd_softdevice_disable();

        if (errCode == NRF_SUCCESS)
        {
            nrf_nvmc_write_words(UICR_DEVINFO, (uint32_t const*)&devInfo, BYTES_TO_WORDS(sizeof(devInfo)));
            nrf_delay_ms(100);
        }

        // re-enabling softdevice is not sufficient, so initialize a reset (this should happen only once, so no problem)
        return enabled ? sd_nvic_SystemReset() : NRF_SUCCESS;
    }
    /// TODO: Check if set, but wrong?
    else
        APP_ERROR_HANDLER(0);

    return NRF_ERROR_INTERNAL;
}

/* Public functions ----------------------------------------------------------*/
void hmi_Init()
{
    pHmiInfo = findBoard();

    if (pHmiInfo == NULL)
        return;

    // first check if device info is set
    setBootloaderProtection();

    // button config
    nrf_gpio_cfg_input(pHmiInfo->buttonPin, NRF_GPIO_PIN_PULLUP);

    // led config
    if (pHmiInfo->ledBlue == pHmiInfo->ledRed)
    {   // rev 2.2/2.3 board
        nrf_gpio_cfg_default(pHmiInfo->ledBlue);
    }
    else
    {   // rev 2.1 board
        nrf_gpio_cfg_output(pHmiInfo->ledRed);
        nrf_gpio_pin_clear(pHmiInfo->ledRed);
        nrf_gpio_cfg_output(pHmiInfo->ledBlue);
        nrf_gpio_pin_clear(pHmiInfo->ledBlue);
    }
}

void hmi_GetClockConfig(nrf_clock_lf_cfg_t* pConfig)
{
    static const nrf_clock_lf_cfg_t clockConfig =
    {
        .source = NRF_CLOCK_LF_SRC_RC,
        .rc_ctiv = 16,
        .rc_temp_ctiv = 1
    };

    if (pConfig == NULL)
        return;

    *pConfig = clockConfig;
}

bool hmi_IsDfuButtonPressed()
{
    if (pHmiInfo == NULL)
        return false;

    return nrf_gpio_pin_read(pHmiInfo->buttonPin) == 0 ? true : false;
}

void hmi_SetLed(hmi_leds_t led, bool state)
{
    if (pHmiInfo == NULL)
        return;

    static ledState_t ledState;

    switch (led)
    {
    case HMI_LED_ADVERTISING: ledState.red = state;   break;
    case HMI_LED_CONNECTED:   ledState.blue = state;  break;
    case HMI_LED_UPDATING:    ledState.green = state; break;
    default: return;
    }

    if (pHmiInfo->ledBlue == pHmiInfo->ledRed)
    {   // rev 2.2/2.3 boards
        if (ledState.red)
        {
            nrf_gpio_cfg_output(pHmiInfo->ledRed);
            nrf_gpio_pin_clear(pHmiInfo->ledRed);
        }
        else if (ledState.blue)
        {
            nrf_gpio_cfg_output(pHmiInfo->ledBlue);
            nrf_gpio_pin_set(pHmiInfo->ledBlue);
        }
        else
            nrf_gpio_cfg_default(pHmiInfo->ledRed);
    }
    else
    {   // rev 2.1 code
        if (ledState.red)
            nrf_gpio_pin_set(pHmiInfo->ledRed);
        else
            nrf_gpio_pin_clear(pHmiInfo->ledRed);

        if (ledState.blue)
            nrf_gpio_pin_set(pHmiInfo->ledBlue);
        else
            nrf_gpio_pin_clear(pHmiInfo->ledBlue);
    }
}

/**END OF FILE*****************************************************************/



