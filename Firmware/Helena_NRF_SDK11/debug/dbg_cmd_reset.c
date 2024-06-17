/**
  ******************************************************************************
  * @file    debug.c
  * @author  Thomas Reisnecker
  * @brief   debug and error handling module
  ******************************************************************************
  */

#ifdef DEBUG_EXT

/* Includes ------------------------------------------------------------------*/
#include "nrf_delay.h"
#include "string.h"
#include "debug.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void dbg_cmd_Reset(char const* pSubcommand, uint16_t length, uint16_t connHandle)
{
    (void)pSubcommand;
    (void)length;

    static char const rspMsg[] = "resetting...\r\n";
    uint32_t errCode;
    uint16_t len;

    len = strlen(rspMsg);
    errCode = dbg_DataSend((uint8_t*)rspMsg, &len, connHandle);
    APP_ERROR_CHECK(errCode);

    nrf_delay_ms(1000);
    NVIC_SystemReset();
}

/* section variable ----------------------------------------------------------*/
DEBUG_REGISTER(const dbg_commands_t dbg_cmd_reset) =
{
    .pCommandName    = "reset",
    .pCommandInit    = NULL,
    .pCommandFunc    = dbg_cmd_Reset,
    .pCommandExecute = NULL,
};

#endif // DEBUG_EXT

/**END OF FILE*****************************************************************/
