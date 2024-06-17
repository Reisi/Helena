/**
  ******************************************************************************
  * @file    log.c
  * @author  Thomas Reisnecker
  * @brief   logger module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "log.h"
#include "nrf_delay.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
void log_Init(void)
{
#ifdef DEBUG
    SEGGER_RTT_Init();

    nrf_delay_ms(100);
    LOG_INFO("Helena started %d", 2);
    SEGGER_RTT_printf(0, "Helena started %d", 4);
#endif
}

/**END OF FILE*****************************************************************/