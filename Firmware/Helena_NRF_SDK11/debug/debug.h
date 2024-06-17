/**
  ******************************************************************************
  * @file    debug.h
  * @author  Thomas Reisnecker
  * @brief   debug module header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "app_error.h"

#ifdef DEBUG
#include "SEGGER_RTT.h"
#endif

#ifdef DEBUG_EXT
#include "section_vars.h"
#include "ble_nus.h"
#endif // DEBUG_EXT

/* Exported types ------------------------------------------------------------*/
#ifdef DEBUG_EXT
typedef void (*dbg_commandInit_t)(void);
typedef void (*dbg_commandFunc_t)(char const* pSubcommand, uint16_t lenght, uint16_t connHandle);
typedef void (*dbg_commandExecute_t)(void);

typedef struct
{
    char const* pCommandName;
    dbg_commandInit_t pCommandInit;
    dbg_commandFunc_t pCommandFunc;
    dbg_commandExecute_t pCommandExecute;
} dbg_commands_t;

NRF_SECTION_VARS_REGISTER_SYMBOLS(dbg_commands_t, debug_commands);
#endif // DEBUG_EXT

/* Exported constants --------------------------------------------------------*/
#if defined DEBUG_EXT
#define DBG_MAX_DATA_LEN    BLE_NUS_MAX_DATA_LEN
#endif // DEBUG_EXT

/* Exported macros -----------------------------------------------------------*/
#if defined DEBUG_EXT
#define DEBUG_REGISTER(command) NRF_SECTION_VARS_ADD(debug_commands, command)
#define DEBUG_CNT()             NRF_SECTION_VARS_COUNT(dbg_commands_t, debug_commands)
#define DEBUG_GET(cnt)          NRF_SECTION_VARS_GET(cnt, dbg_commands_t, debug_commands)
#endif // DEBUG_EXT

/* Exported functions ------------------------------------------------------- */
ret_code_t dbg_Init(void);

void dbg_Execute(void);

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info);

#if defined DEBUG_EXT
ret_code_t dbg_DataSend(uint8_t* pData, uint16_t* pLength, uint16_t connHandle);
#endif

#endif /* DEBUG_H_INCLUDED */

/**END OF FILE*****************************************************************/
