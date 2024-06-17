/**
  ******************************************************************************
  * @file    dbg_wdt_reset.h
  * @author  Thomas Reisnecker
  * @brief   header file for watchdog test command over debug interface
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DBG_CMD_WDT_H_INCLUDED
#define DBG_CMD_WDT_H_INCLUDED

#ifdef DEBUG_EXT

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void dbg_cmd_Wdt(char const* pSubcommand, uint16_t length, uint16_t connHandle);

#endif // DEBUG_EXT

#endif // DBG_WDT_RESET_H_INCLUDED

/**END OF FILE*****************************************************************/
