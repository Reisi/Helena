/**
  ******************************************************************************
  * @file    debug.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    15/10/23
  * @brief   debug and error handling module for Helena
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#if !defined HELENA_DEBUG_NONE && !defined HELENA_DEBUG_SWD && !defined HELANA_DEBUG_RTT
#error "no debug state defined, define HELENA_DEBUG_NONE, HELENA_DEBUG_SWD and/or HELENA_DEBUG_RTT in project settings!"
#endif

#if defined HELENA_DEBUG_RTT && !defined HELENA_DEBUG_SWD
#error "RTT debugging needs SWD debugging as well, please enable HELENA_DEBUG_SWD too"
#endif

#if defined HELENA_DEBUG_RTT && !defined DEBUG
#warning "you also have to define DEBUG, otherwise no line and file data are available"
#endif

#if defined HELENA_DEBUG_FIELD_TESTING && !defined HELENA_DEBUG_RTT
#error "field testing is using rtt resources, enable it, too"
#endif

#if defined HELENA_DEBUG_RTT
#include "SEGGER_RTT.h"
#endif

#if defined HELENA_DEBUG_FIELD_TESTING
#include "ble_nus.h"
#include "power.h"
#include "ble_err.h"
#include "nrf_delay.h"
#include "crc16.h"
#include "fstorage.h"
#endif

#include "debug.h"
#include "nrf.h"
#include "app_util_platform.h"
#include "nrf_drv_wdt.h"
#include <string.h>

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/**@brief data struct for rtt read and write pointers in noinit section */
typedef struct
{
    uint32_t magicNumber;       /**< magic number to check integrity */
    uint16_t crc;               /**< crc of magic number to check integrity */
    volatile unsigned int WrOff;/**< rtt write pointer */
    volatile unsigned int RdOff;/**< rtt read pointer */
} rttPointersStruct;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define MAGICVAILDNUMBER    0x78151503

/* Private variables ---------------------------------------------------------*/
#ifdef HELENA_DEBUG_RTT
static char *pDebugMessages[DBG_CNT] =
{
    "Helena Error",
    "NRF Error",
    "MPU Error"
};
#endif

#if defined HELENA_DEBUG_FIELD_TESTING
static ble_nus_t    nusGattsData;                                           /**< database for nordic uart service */
static rttPointersStruct rttPointers __attribute__((section(".noinit")));   /**< rtt pointers stored in noinit section to survive reset */
static bool isErrorLogPending, isErrorLogMorePending, isMemoryClearPending; /**< Flags for pending actions */
static volatile bool eraseInProgress = false;                               /**< Flag indication if flash erase is in progress */
#endif

static nrf_drv_wdt_channel_id watchdogChannel;                              /**< watchdog channel used in debug module */

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void hard_fault_handler_c(unsigned int * hardfault_args, unsigned lr_value);

#ifdef HELENA_DEBUG_FIELD_TESTING
/**@brief Function to send error log over ble through nordic uart service */
static void sendErrorLog(void *pData, uint16_t size)
{
    (void)pData;
    (void)size;

    uint32_t errCode;
    char* pNoLogMsg = "error log is empty\r\n";

    // check if error log is empty
    if (_SEGGER_RTT.aUp[0].RdOff == _SEGGER_RTT.aUp[0].WrOff)
    {
        errCode = ble_nus_string_send(&nusGattsData, (uint8_t*)pNoLogMsg, strlen(pNoLogMsg) - 1);
        if (errCode != BLE_ERROR_NO_TX_BUFFERS && errCode != NRF_ERROR_BUSY)
            isErrorLogPending = false;
    }
    else
    {
        isErrorLogPending = false;
        isErrorLogMorePending = false;
        while (_SEGGER_RTT.aUp[0].RdOff != _SEGGER_RTT.aUp[0].WrOff)
        {
            // check size of error log
            uint32_t contentSize = _SEGGER_RTT.aUp[0].WrOff - _SEGGER_RTT.aUp[0].RdOff;
            contentSize %= _SEGGER_RTT.aUp[0].SizeOfBuffer;
            // if error log is bigger than max. transfer size it has to be sent in several chunks
            uint32_t thisStringSize = BLE_NUS_MAX_DATA_LEN;
            if (thisStringSize > contentSize)
                thisStringSize = contentSize;
            if (thisStringSize > (_SEGGER_RTT.aUp[0].SizeOfBuffer - _SEGGER_RTT.aUp[0].WrOff))
                thisStringSize = (_SEGGER_RTT.aUp[0].SizeOfBuffer - _SEGGER_RTT.aUp[0].WrOff);
            errCode = ble_nus_string_send(&nusGattsData, (uint8_t*)&_SEGGER_RTT.aUp[0].pBuffer[_SEGGER_RTT.aUp[0].RdOff],
                                           thisStringSize);
            if (errCode == NRF_SUCCESS)
            {
                _SEGGER_RTT.aUp[0].RdOff += thisStringSize;
                _SEGGER_RTT.aUp[0].RdOff %= _SEGGER_RTT.aUp[0].SizeOfBuffer;
                rttPointers.RdOff = _SEGGER_RTT.aUp[0].RdOff;
            }
            else if (errCode == BLE_ERROR_NO_TX_BUFFERS || errCode == NRF_ERROR_BUSY)
            {
                isErrorLogMorePending = true;
                break;
            }
        }
    }
}

/**@brief Function to clear user memory */
static void clearMem()
{
    uint32_t errCode;

    errCode = sd_flash_page_erase((FS_PAGE_END_ADDR - 1 * FS_PAGE_SIZE) / FS_PAGE_SIZE);
    if (errCode == NRF_SUCCESS)
    {
        eraseInProgress = true;
    }
    else
    {
        APP_ERROR_HANDLER(errCode);
    }
    while (eraseInProgress == true);
    errCode = sd_flash_page_erase((FS_PAGE_END_ADDR - 2 * FS_PAGE_SIZE) / FS_PAGE_SIZE);
    if (errCode == NRF_SUCCESS)
    {
        eraseInProgress = true;
    }
    else
    {
        APP_ERROR_HANDLER(errCode);
    }
    while (eraseInProgress == true);
    errCode = ble_nus_string_send(&nusGattsData, (uint8_t*)"done, cycle power\r\n", 19);
    APP_ERROR_CHECK(errCode);

    isMemoryClearPending = false;
}

static void onBleNusEvt(ble_nus_t *pNus, uint8_t *pData, uint16_t length)
{
    char* pString = (char*)pData;

    if (strncmp(pString, "get error log", 13) == 0)
    {
        isErrorLogPending = true;
    }
    if (strncmp(pString, "fake error", 10) == 0)
    {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }
    if (strncmp(pString, "clear mem", 9) == 0)
    {
        isMemoryClearPending = true;
    }
}
#endif

static void watchdogEventHandler()
{
    /// maybe do something
}

/* Public functions ----------------------------------------------------------*/
void debug_Init()
{
#if defined HELENA_DEBUG_NONE
    /**< nothing to do... */
#endif

#if defined HELENA_DEBUG_SWD
    /**< nothing to do... */
#endif

#if defined HELENA_DEBUG_RTT
    // check if noinit data is valid
    SEGGER_RTT_Init();
    if (rttPointers.magicNumber == MAGICVAILDNUMBER &&
        rttPointers.crc == crc16_compute((const uint8_t*)&rttPointers.magicNumber, sizeof(rttPointers.magicNumber), NULL))
    {
        // noinit data is valid, use last known read and write pointers
        _SEGGER_RTT.aUp[0].WrOff = rttPointers.WrOff;
        _SEGGER_RTT.aUp[0].RdOff = rttPointers.RdOff;
    }
    else
    {
        // noinit data isn't valid, initialize
        rttPointers.magicNumber = MAGICVAILDNUMBER;
        rttPointers.crc = crc16_compute((const uint8_t*)&rttPointers.magicNumber, sizeof(rttPointers.magicNumber), NULL);
        rttPointers.WrOff = _SEGGER_RTT.aUp[0].WrOff;
        rttPointers.RdOff = _SEGGER_RTT.aUp[0].RdOff;
    }
#endif

    uint32_t errCode;
    nrf_drv_wdt_config_t wdtConfig =
    {
        .behaviour = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT,
        .reload_value = 4000,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };
    errCode = nrf_drv_wdt_init(&wdtConfig, &watchdogEventHandler);
    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_HANDLER(errCode);
    }
    APP_ERROR_CHECK(nrf_drv_wdt_channel_alloc(&watchdogChannel));
    nrf_drv_wdt_enable();
}

void debug_Execute()
{
    nrf_drv_wdt_channel_feed(watchdogChannel);
#if defined HELENA_DEBUG_FIELD_TESTING
    if (isErrorLogPending || isErrorLogMorePending)
        sendErrorLog(NULL, 0);
    if (isMemoryClearPending)
        clearMem();
#endif
}

#if defined HELENA_DEBUG_FIELD_TESTING
void debug_FieldTestingInit()
{
    uint32_t err_code;

    // initialization of the nordic uart service for test support
    ble_nus_init_t nusInit;
    memset(&nusInit, 0, sizeof(nusInit));

    nusInit.data_handler = &onBleNusEvt;

    err_code = ble_nus_init(&nusGattsData, &nusInit);
    APP_ERROR_CHECK(err_code);
}

void debug_OnBleEvt(ble_evt_t * pBleEvt)
{
    ble_nus_on_ble_evt(&nusGattsData, pBleEvt);
}

void debug_OnSysEvent(uint32_t sysEvt)
{
    if (sysEvt == NRF_EVT_FLASH_OPERATION_SUCCESS)
        eraseInProgress = false;
    else if (sysEvt == NRF_EVT_FLASH_OPERATION_ERROR)
        APP_ERROR_CHECK(0);
}
#endif

void debug_ErrorHandler(debugMessageEnum debug, uint32_t err_code, uint32_t line, const uint8_t* pfile)
{
#if defined HELENA_DEBUG_NONE
    NVIC_SystemReset();
#endif

    volatile bool loop = true;  /**< variable decides if ErrorHandler stays in infinite while loop if called */

#if defined HELENA_DEBUG_RTT
    loop = false;               /**< no infinite loop necessary, error message will be displayed in terminal */
#endif

#if defined HELENA_DEBUG_SWD && !defined HELENA_DEBUG_RTT
    (void)err_code;
    (void)line;
    (void)pfile;
#endif

#if defined HELENA_DEBUG_RTT
    // reduce string to filename
    const char* pFile;
    pFile = strrchr((char*)pfile, '/');
    pFile = pFile == NULL ? (const char*)pfile : pFile + 1;

    SEGGER_RTT_WriteString(0, pDebugMessages[debug]);
    switch (debug)
    {
    case DBG_HELENA:
    case DBG_NRF:
        SEGGER_RTT_printf(0, " %x, %u, %s\r\n", err_code, line, pFile);
        break;
    case DBG_MPU:
        SEGGER_RTT_printf(0, " %d, %u, %s\r\n", err_code, line, pFile);
        break;
    default:
        break;
    }
#ifdef HELENA_DEBUG_FIELD_TESTING
    // wait until RTT client has read out information, if no client is connected
    // device will be reset through watchdog
    /*do
    {
        rttPointers.WrOff = _SEGGER_RTT.aUp[0].WrOff;
        rttPointers.RdOff = _SEGGER_RTT.aUp[0].RdOff;
    }
    while (rttPointers.WrOff != rttPointers.RdOff);*/
    //while (_SEGGER_RTT.aUp[0].WrOff != _SEGGER_RTT.aUp[0].RdOff);
#endif
#endif

    CRITICAL_REGION_ENTER();
    while(loop);
    CRITICAL_REGION_EXIT();
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    debug_ErrorHandler(DBG_NRF, error_code, line_num, p_file_name);
}

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler( void ) __attribute__( ( naked ) );

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
    __asm volatile
    (
        "MOVS r0, #4                                        \n"
        "MOV r1, LR                                         \n"
        "TST r0, r1                                         \n"
        "BEQ stacking_used_MSP                              \n"
        "MRS R0, PSP                                        \n"
        "B get_LR_and_branch                                \n"
        "stacking_used_MSP:                                 \n"
        "MRS R0, MSP                                        \n"
        "get_LR_and_branch:                                 \n"
        "MOV R1, LR                                         \n"
        "LDR R2, handler2_address_const                     \n"
        "BX R2                                              \n"
        "handler2_address_const: .word hard_fault_handler_c \n"
    );
}

// From Joseph Yiu, minor edits by FVH
// hard fault handler in C,
// with stack frame location as input parameter
// called from HardFault_Handler in file xxx.s
void hard_fault_handler_c(unsigned int * hardfault_args, unsigned lr_value)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;

#if defined HELENA_DEBUG_RTT
    stacked_r0 = ((unsigned long) hardfault_args[0]);
    stacked_r1 = ((unsigned long) hardfault_args[1]);
    stacked_r2 = ((unsigned long) hardfault_args[2]);
    stacked_r3 = ((unsigned long) hardfault_args[3]);

    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    stacked_psr = ((unsigned long) hardfault_args[7]);

    SEGGER_RTT_printf (0, "\n\n[Hard fault handler - all numbers in hex]\n");
    SEGGER_RTT_printf (0, "R0 = %x\n", stacked_r0);
    SEGGER_RTT_printf (0, "R1 = %x\n", stacked_r1);
    SEGGER_RTT_printf (0, "R2 = %x\n", stacked_r2);
    SEGGER_RTT_printf (0, "R3 = %x\n", stacked_r3);
    SEGGER_RTT_printf (0, "R12 = %x\n", stacked_r12);
    SEGGER_RTT_printf (0, "LR [R14] = %x  subroutine call return address\n", stacked_lr);
    SEGGER_RTT_printf (0, "PC [R15] = %x  program counter\n", stacked_pc);
    SEGGER_RTT_printf (0, "PSR = %x\n", stacked_psr);
    SEGGER_RTT_printf (0, "LR = %x\n", lr_value);
#else
    (void)stacked_r0;
    (void)stacked_r1;
    (void)stacked_r2;
    (void)stacked_r3;
    (void)stacked_r12;
    (void)stacked_lr;
    (void)stacked_pc;
    (void)stacked_psr;
#endif
  while (1);
}

/**END OF FILE*****************************************************************/



