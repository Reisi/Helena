/**
  ******************************************************************************
  * @file    dbg_cmd_reset.h
  * @author  Thomas Reisnecker
  * @brief   header file for reset command over debug interface
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DBG_CMD_RESET_H_INCLUDED
#define DBG_CMD_RESET_H_INCLUDED

#ifdef DEBUG_EXT

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void dbg_cmd_Reset(char const* pSubcommand, uint16_t length, uint16_t connHandle);

#endif // DEBUG_EXT

#endif // DBG_CMD_RESET_H_INCLUDED

/**END OF FILE*****************************************************************/
