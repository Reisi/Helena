/**
  ******************************************************************************
  * @file    helena_hmi.c
  * @author  Thomas Reisnecker
  * @brief   button and status led handling (template)
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
//#define BRD_LOG_ENABLED

#ifdef BRD_LOG_ENABLE
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BRD_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "helena_board.h"
#include "string.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    BRD_REV_21,
    BRD_REV_22_23,
    BRD_REV_CNT
} brd_rev_t;

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)           (sizeof(x)/sizeof(x[0]))

/* Private defines -----------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
static hln_brd_t const boards[] =
{
    {
        .pHwRevString = "2.1",
        .button = 11,
        .analogeVin = 6,
        .ledPins.red = 10,
        .ledPins.blue = 27,
        .twiPins.scl = 14,
        .twiPins.sda = 13,
        .comPins.rx = 15,
        .comPins.tx = 16,
        .imuCfg.intPin = 12,
        .imuCfg.imuOrient =
        {
            1,  0,  0,
            0,  0,  1,
            0, -1,  0
        },
    },
    {
        .pHwRevString = "2.2/2.3",
        .button = 9,
        .analogeVin = 1,
        .ledPins.red = 13,
        .ledPins.blue = 13,
        .twiPins.scl = 3,
        .twiPins.sda = 2,
        .comPins.rx = 10,
        .comPins.tx = 10,
        .imuCfg.intPin = 4,
        .imuCfg.imuOrient =
        {
            -1,  0,  0,
             0,  0,  1,
             0,  1,  0
        }
    }
};

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static hln_brd_t const* findBoard()
{
    // all board variants have the SCL pin on a different pin, this can be used
    // to identify the board
    for (brd_rev_t i = 0; i < BRD_REV_CNT; i++)
    {
        uint32_t pinState;

        nrf_gpio_cfg_input(boards[i].twiPins.scl, NRF_GPIO_PIN_PULLDOWN);   // activate pull down

        nrf_delay_us(50);

        pinState = nrf_gpio_pin_read(boards[i].twiPins.scl);

        nrf_gpio_cfg_default(boards[i].twiPins.scl);

        LOG_INFO("[brd]: board %s detected", boards[i].pHwRevString);

        if (pinState) // if the pin is still high, than the external I2C pull up has won
            return &boards[i];
    }

    LOG_INFO("[brd]: no board detected");

    return NULL;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t hln_brd_Init(hln_brd_t const** ppBrd)
{
    if (ppBrd == NULL)
        return NRF_ERROR_NULL;

    *ppBrd = findBoard();

    return *ppBrd == NULL ? NRF_ERROR_NOT_FOUND : NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
