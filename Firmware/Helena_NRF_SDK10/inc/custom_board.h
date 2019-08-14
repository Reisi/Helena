/**
  ******************************************************************************
  * @file    custom_board.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/12/02
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_BOARD_H_INCLUDED
#define CUSTOM_BOARD_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
#define LEDS_NUMBER     2

#define BSP_LED_0       (pBoardConfig->ledBlue)
#define BSP_LED_1       (pBoardConfig->ledRed)

#define BUTTONS_NUMBER  1
#define BSP_BUTTON_0    (pBoardConfig->button)
#define BUTTON_PULL     NRF_GPIO_PIN_PULLUP

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION

#define HW_REV_STR_LEN  8

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t    button;
    uint32_t    ledBlue;
    uint32_t    ledRed;
    uint32_t    bdepRX;
    uint32_t    bdepTX;
    uint32_t    analogVin;
    uint32_t    i2cSCL;
    uint32_t    i2cSDA;
    uint32_t    mpuInt;
    signed char gyroOrientation[9];
    char        hwRevStr[HW_REV_STR_LEN];
} board_ConfigStruct;

/* Exported variables --------------------------------------------------------*/
extern const board_ConfigStruct * pBoardConfig;

/* Exported functions ------------------------------------------------------- */
uint32_t board_Init(void);

#endif /* CUSTOM_BOARD_H_INCLUDED */

/**END OF FILE*****************************************************************/
