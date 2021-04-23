/**
  ******************************************************************************
  * @file    debug.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    15/10/26
  * @brief   header file for error handling module
  *
  * @note    this module handles different error handling strategies, depending
  *          on different debug levels. Below is a list of different levels,
  *          note that not all levels in the list are implemented yet and that
  *          the list is not complete yet. The debug level code shall be defined
  *          in the project settings.
  *
  *          HELENA_DEBUG_NONE
  *          no debugging, device performs a reset if error handler is called
  *
  *          HELENA_DEBUG_SWD
  *          just normal debugging with SWD interface is used.
  *
  *          HELENA_DEBUG_RTT
  *          debugging via SWD interface and additional debug information over
  *          a terminal window using SEGGERS Real Time Terminal (requires J-LINK
  *          debug probe).
  *
  *          BTDEBUG
  *          debugging while no debugger is connected. Debug message will be
  *          logged into an non initialized buffer, and device will be reset.
  *          buffer can be read out via Nordic uart service, or when connecting
  *          debug probe. Debug information will be lost if power is unplugged.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBUG_H
#define DEBUG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#if defined BTDEBUG
#include "ble.h"
#endif

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initiate the debug module
 */
void debug_Init(void);

/** @brief execute function of the debug module
 *
 * @note call this function on a regular basis in the main loop
 */
void debug_Execute(void);

/** @brief function to initiate the factory reset
 *
 * @details this function deletes the application memory pages using "brute
 *          force".
 *
 * @param[in] reset true if a reset has to be initiated afterwards
 * @return    NRF_SUCCESS
 *            the relayed error code of sd_flash_page_erase
 */
uint32_t debug_FactoryReset(bool reset);

#if defined BTDEBUG
void debug_OnNusEvt(uint8_t *pData, uint16_t length);
void debug_OnSysEvent(uint32_t sysEvt);
#endif

#endif /* ERROR_H */

/**END OF FILE*****************************************************************/
