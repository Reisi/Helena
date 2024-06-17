/**
  ******************************************************************************
  * @file    dbg_cmd_mem.h
  * @author  Thomas Reisnecker
  * @brief   header file for memory command over debug interface
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DBG_CMD_MEM_H_INCLUDED
#define DBG_CMD_MEM_H_INCLUDED

#ifdef DEBUG_EXT

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void dbg_cmd_Mem(char const* pSubcommand, uint16_t length, uint16_t connHandle);

#endif // DEBUG_EXT

#endif // DBG_CMD_RESET_H_INCLUDED

/**END OF FILE*****************************************************************/
