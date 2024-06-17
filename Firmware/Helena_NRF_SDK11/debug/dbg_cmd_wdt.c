/**
  ******************************************************************************
  * @file    dbg_cmd_wdt.c
  * @author  Thomas Reisnecker
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
void dbg_cmd_Wdt(char const* pSubcommand, uint16_t length, uint16_t connHandle)
{
    (void)pSubcommand;
    (void)length;

    static char const rspMsg[] = "testing wdt...\r\n";
    uint32_t errCode;
    uint16_t len;

    len = strlen(rspMsg);
    errCode = dbg_DataSend((uint8_t*)rspMsg, &len, connHandle);
    APP_ERROR_CHECK(errCode);

    while(1);
}

/* section variable ----------------------------------------------------------*/
DEBUG_REGISTER(const dbg_commands_t dbg_cmd_reset) =
{
    .pCommandName    = "wdt",
    .pCommandInit    = NULL,
    .pCommandFunc    = dbg_cmd_Wdt,
    .pCommandExecute = NULL,
};

#endif // DEBUG_EXT

/**END OF FILE*****************************************************************/
