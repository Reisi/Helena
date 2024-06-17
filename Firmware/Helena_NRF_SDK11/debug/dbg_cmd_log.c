/**
  ******************************************************************************
  * @file    dbg_cmd_log.c
  * @author  Thomas Reisnecker
  * @brief   debug error log
  ******************************************************************************
  */

#ifdef DEBUG_EXT

/* Includes ------------------------------------------------------------------*/
#include "SEGGER_RTT.h"
#include "debug.h"
#include "string.h"
#include "ble.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef void (*subCommandFunc_t)(uint16_t connHandle);

typedef struct
{
    char* pName;
    subCommandFunc_t func;
} subCommand_t;

typedef struct
{
    uint16_t     connHandle;
    unsigned int rdOff;
} rttBufRead_t;

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(array)   (sizeof(array)/sizeof(array[0]))

/* Private defines -----------------------------------------------------------*/
#define MAGICNUMBER 0xDEB6A71D  // magic number to determine if no init section is valid

/* Private function prototype ------------------------------------------------*/
static void read(uint16_t connHandle);
static void clear(uint16_t connHandle);
static void fake(uint16_t connHandle);

/* Private variables ---------------------------------------------------------*/
static const subCommand_t subCmds[] =
{
    {.pName = " --read",  .func = read},
    {.pName = " --clear", .func = clear},
    {.pName = " --fake",  .func = fake},
};

static uint16_t     rttRead, rttWrite;
static uint32_t     isNoInitValid __attribute__((section(".noinit")));
static rttBufRead_t rttBufRead;

/* Private functions ---------------------------------------------------------*/
__attribute__ ((constructor)) static void saveRtt()
{
    if (isNoInitValid == MAGICNUMBER)
    {
        rttRead  = _SEGGER_RTT.aUp[0].RdOff;
        rttWrite = _SEGGER_RTT.aUp[0].WrOff;
    }
    else
    {
        memset(&_SEGGER_RTT, 0, sizeof(SEGGER_RTT_CB));
    }
}

static void dbg_cmd_LogInit()
{
    if (isNoInitValid == MAGICNUMBER)
    {
        _SEGGER_RTT.aUp[0].RdOff = rttRead;
        _SEGGER_RTT.aUp[0].WrOff = rttWrite;
    }
    else
        isNoInitValid = MAGICNUMBER;

    rttBufRead.rdOff = _SEGGER_RTT.aUp[0].SizeOfBuffer;
    rttBufRead.connHandle = BLE_CONN_HANDLE_INVALID;
}

static void read(uint16_t connHandle)
{
    rttBufRead.rdOff = _SEGGER_RTT.aUp[0].RdOff;
    rttBufRead.connHandle = connHandle;
}

static void clear(uint16_t connHandle)
{
    static char const rspMsg[] = "done\r\n";
    uint16_t len = strlen(rspMsg);
    ret_code_t errCode;

    _SEGGER_RTT.aUp[0].RdOff = _SEGGER_RTT.aUp[0].WrOff;

    errCode = dbg_DataSend((uint8_t*)rspMsg, &len, connHandle);
    if (errCode != NRF_SUCCESS)
        return;
}

static void fake(uint16_t connHandle)
{
    (void)connHandle;

    APP_ERROR_HANDLER(0);
}

static void dbg_cmd_Log(char const* pSubcommand, uint16_t length, uint16_t connHandle)
{
    // search subcommand
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(subCmds); i++)
    {
        int16_t len = strlen(subCmds[i].pName);
        if (length < len)
            continue;   // no need to compare if received message is shorter than command
        if (strncmp(pSubcommand, subCmds[i].pName, len) == 0)
        {
            subCmds[i].func(connHandle);
            return;
        }
    }

    // subcommand not found, send list of available commands
    ret_code_t errCode;
    uint16_t len;
    char list[DBG_MAX_DATA_LEN + 1];

    strcpy(list, "usage:\r\n");
    len = strlen(list);

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(subCmds); i++)
    {
        // send message if there is not enough space for the next command name (incl. linefeed)
        if (len + 2 + strlen(subCmds[i].pName) > sizeof(list))  // additional 2 bytes for \r\n
        {
            errCode = dbg_DataSend((uint8_t*)list, &len, connHandle);
            if (errCode != NRF_SUCCESS)
                return;

            len = 0;
            list[0] = '\0';
        }

        strcat(list, subCmds[i].pName);
        strcat(list, "\r\n");
        len = strlen(list);
    }

    // send last message
    (void)dbg_DataSend((uint8_t*)list, &len, connHandle);
}

static void dbg_cmd_LogExecute()
{
    SEGGER_RTT_BUFFER_UP* pBuf = &_SEGGER_RTT.aUp[0];

    if (rttBufRead.rdOff != pBuf->SizeOfBuffer &&
        rttBufRead.connHandle != BLE_CONN_HANDLE_INVALID)
    {
        if (rttBufRead.rdOff != pBuf->WrOff)
        {
            ret_code_t errCode;
            uint16_t len;
            char* pString;

            pString = &_SEGGER_RTT.aUp[0].pBuffer[rttBufRead.rdOff];
            len = pBuf->WrOff;
            if (len < rttBufRead.rdOff)
                len = pBuf->SizeOfBuffer;
            len -= rttBufRead.rdOff;
            if (len > DBG_MAX_DATA_LEN)
                len = DBG_MAX_DATA_LEN;
            errCode = dbg_DataSend((uint8_t*)pString, &len, rttBufRead.connHandle);
            if (errCode == NRF_SUCCESS)
            {
                rttBufRead.rdOff += len;
                rttBufRead.rdOff %= pBuf->SizeOfBuffer;
            }
            else if (errCode != NRF_ERROR_RESOURCES)
                rttBufRead.rdOff = pBuf->WrOff;
        }

        if (rttBufRead.rdOff == pBuf->WrOff)
        {
            static char const rspMsg[] = "done\r\n";
            uint16_t len = strlen(rspMsg);
            ret_code_t errCode = dbg_DataSend((uint8_t*)rspMsg, &len, rttBufRead.connHandle);

            rttBufRead.rdOff = pBuf->SizeOfBuffer;
            rttBufRead.connHandle = BLE_CONN_HANDLE_INVALID;

            if (errCode != NRF_SUCCESS)
                return;
        }
    }
}

/* section variable ----------------------------------------------------------*/
DEBUG_REGISTER(const dbg_commands_t dbg_cmd_log) =
{
    .pCommandName    = "errorlog",
    .pCommandInit    = dbg_cmd_LogInit,
    .pCommandFunc    = dbg_cmd_Log,
    .pCommandExecute = dbg_cmd_LogExecute,
};

#endif // DEBUG_EXT

/**END OF FILE*****************************************************************/
