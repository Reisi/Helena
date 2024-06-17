/**
  ******************************************************************************
  * @file    helena_board.h
  * @author  Thomas Reisnecker
  * @brief   Header for helena board identification module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HLN_BRD_H_INCLUDED
#define HLN_BRD_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "nrf_sdm.h"
#include "board.h"

/* Exported constants --------------------------------------------------------*/
#define HLN_LED_NA  0xFFFFFFFFF

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t red;
//    uint32_t green;
    uint32_t blue;
} hln_ledPins_t;

typedef struct
{
    uint32_t scl;
    uint32_t sda;
} hln_twiPins_t;

typedef struct
{
    uint32_t intPin;
    signed char imuOrient[9];
} hln_imuConfig_t;

typedef struct
{
    char const* pHwRevString;
    uint32_t button;
    uint32_t analogeVin;
    hln_ledPins_t ledPins;
    hln_twiPins_t twiPins;
    brd_comPins_t comPins;
    hln_imuConfig_t imuCfg;
} hln_brd_t;

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the hmi module
 *
 * @return NRF_SUCCESS or a propagated error from but_DebounceInit
 */
ret_code_t hln_brd_Init(hln_brd_t const** ppBrd);

#endif // HLN_BRDH_INCLUDED

/**END OF FILE*****************************************************************/

