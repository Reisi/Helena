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
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
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
} board_config_t;

typedef enum
{
    BOARD_LED_RED,
    BOARD_LED_BLUE,
    BOARD_LED_CNT
} board_led_t;



/* Exported variables --------------------------------------------------------*/
extern const board_config_t * pBoardConfig;

/* Exported functions ------------------------------------------------------- */
uint32_t board_Init(void);

uint32_t board_EnableLed(board_led_t led, bool enable);

#endif /* CUSTOM_BOARD_H_INCLUDED */

/**END OF FILE*****************************************************************/
