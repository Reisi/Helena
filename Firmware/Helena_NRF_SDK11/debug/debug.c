/**
  ******************************************************************************
  * @file    debug.c
  * @author  Thomas Reisnecker
  * @brief   debug module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "watchdog.h"
#include "log.h"

#ifdef DEBUG
#include "nrf_delay.h"
#include <string.h>
#include "SEGGER_RTT.h"
#include "hmi.h"
#include "app_error.h"
#endif // DEBUG

#ifdef DEBUG_EXT
#include "btle.h"
#include "ble_nus.h"
#endif // DEBUG_EXT

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/*typedef struct
{
    unsigned WrOff;
    unsigned RdOff;
} rttBuf_t; */

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define DEAD_BEEF   0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/* Private function prototypes -----------------------------------------------*/
#ifdef DEBUG_EXT
static uint32_t initNus(void * pContext);
static void onNusBtleEvent(void * pContext, ble_evt_t * p_ble_evt);
#endif

/* Private variables ---------------------------------------------------------*/
//static volatile rttBuf_t rttBuf __attribute__((section(".noinit")));
#ifdef DEBUG_EXT
static ble_nus_t nusGattS;
BTLE_SERVICE_REGISTER(nusService, initNus, onNusBtleEvent, NULL);
#endif

/* Private functions ---------------------------------------------------------*/
#ifdef DEBUG_EXT
static void nusDataHandler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    //if (pEvt->type != BLE_NUS_EVT_RX_DATA)
    //    return;

    char const* pString = (char const*)p_data;
    //uint16_t length = length;

    // search command
    for (uint_fast8_t i = 0; i < DEBUG_CNT(); i++)
    {
        dbg_commands_t const* pCommand = DEBUG_GET(i);
        int16_t len = strlen(pCommand->pCommandName);
        if (length < len)
            continue;   // no need to compare if received message is shorter than command
        if (strncmp(pString, pCommand->pCommandName, len) == 0)
        {
            if (pCommand->pCommandFunc != NULL)
                (*pCommand->pCommandFunc)(&pString[len], length - len, p_nus->conn_handle);
            return;
        }
    }

    // subcommand not found, send list of available commands
    ret_code_t errCode;
    uint16_t len;
    char list[DBG_MAX_DATA_LEN + 1];

    strcpy(list, "not found, cmds:\r\n");
    len = strlen(list);

    for (uint_fast8_t i = 0; i < DEBUG_CNT(); i++)
    {
        dbg_commands_t const* pCommand = DEBUG_GET(i);

        // send message if there is not enough space for the next command name (incl. linefeed)
        if (len + 2 + strlen(pCommand->pCommandName) > sizeof(list))  // additional 2 bytes for \r\n
        {
            errCode = dbg_DataSend((uint8_t*)list, &len, p_nus->conn_handle);
            if (errCode != NRF_SUCCESS)
                return;

            len = 0;
            list[0] = '\0';
        }

        strcat(list, pCommand->pCommandName);
        strcat(list, "\r\n");
        len = strlen(list);
    }

    // send last message
    (void)dbg_DataSend((uint8_t*)list, &len, p_nus->conn_handle);
}

static void onNusBtleEvent(void * pContext, ble_evt_t * pBleEvt)
{
    (void)pContext;

    ble_nus_on_ble_evt(&nusGattS, pBleEvt);
}

static uint32_t initNus(void * pContext)
{
    (void)pContext;

    ble_nus_init_t nusInit = {.data_handler = nusDataHandler};
    return ble_nus_init(&nusGattS, &nusInit);
}
#endif

/* Public functions ----------------------------------------------------------*/
ret_code_t dbg_Init(void)
{
    log_Init();

#ifdef DEBUG_EXT
    for (uint_fast8_t i = 0; i < DEBUG_CNT(); i++)
    {
        dbg_commands_t const* pCommand = DEBUG_GET(i);
        if (pCommand->pCommandInit != NULL)
            pCommand->pCommandInit();
    }

#endif

    return wdg_Init();
}

void dbg_Execute(void)
{
#ifdef DEBUG_EXT
    for (uint_fast8_t i = 0; i < DEBUG_CNT(); i++)
    {
        dbg_commands_t const* pCommand = DEBUG_GET(i);
        if (pCommand->pCommandExecute != NULL)
            pCommand->pCommandExecute();
    }
#endif

    wdg_Feed();
}

#ifdef DEBUG_EXT
ret_code_t dbg_DataSend(uint8_t* pData, uint16_t* pLength, uint16_t connHandle)
{
    if (connHandle != nusGattS.conn_handle)
        return NRF_ERROR_INVALID_PARAM; /// TODO: check for better error

    return ble_nus_string_send(&nusGattS, pData, *pLength);
}
#endif

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
#ifdef DEBUG
    error_info_t* pInfo = (error_info_t*)info;
    uint32_t errCode = pInfo->err_code;
    uint16_t line = pInfo->line_num;
    char const* pFile;

    pFile = strrchr((char const*)pInfo->p_file_name, '/');
    pFile = pFile == NULL ? (char const*)pInfo->p_file_name : pFile + 1;

    SEGGER_RTT_printf(0, "0x%02x at %u in %s\r\n", errCode, line, pFile);

    // for the case that SEGGER RTT viewer is not running a reset will be
    // performed
    /*nrf_delay_us(5000);
    rttBuf.RdOff = _SEGGER_RTT.aUp[0].RdOff;
    rttBuf.WrOff = _SEGGER_RTT.aUp[0].WrOff;
    if(_SEGGER_RTT.aUp[0].RdOff != _SEGGER_RTT.aUp[0].WrOff)
        NVIC_SystemReset();*/

#else
    (void)id;
    (void)pc;
    (void)info;

    NVIC_SystemReset();
#endif
}

/**END OF FILE*****************************************************************/
