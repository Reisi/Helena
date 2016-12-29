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

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    rev20 = 0,
    rev21,
    revCnt
} revisionEnum;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const board_ConfigStruct boardConfig[revCnt] =
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
};
const board_ConfigStruct * pBoardConfig;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
uint32_t board_Init()
{
    for (uint_fast8_t i = 0; i < revCnt; i++)
    {
        nrf_gpio_cfg_input(boardConfig[i].i2cSCL, NRF_GPIO_PIN_PULLDOWN);
        nrf_gpio_cfg_input(boardConfig[i].i2cSDA, NRF_GPIO_PIN_PULLDOWN);
        nrf_delay_us(50);
        if (nrf_gpio_pin_read(boardConfig[i].i2cSCL))// && nrf_gpio_pin_read(boardConfig[i].i2cSDA))
        {
            nrf_gpio_cfg_default(boardConfig[i].i2cSCL);
            nrf_gpio_cfg_default(boardConfig[i].i2cSDA);
            pBoardConfig = &boardConfig[i];
            return NRF_SUCCESS;
        }
        else
        {
            nrf_gpio_cfg_default(boardConfig[i].i2cSCL);
            nrf_gpio_cfg_default(boardConfig[i].i2cSDA);
        }
    }
    return NRF_ERROR_NOT_FOUND;
}

/**END OF FILE*****************************************************************/
