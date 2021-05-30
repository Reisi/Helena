/**
  ******************************************************************************
  * @file    debug.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    15/10/23
  * @brief   debug and error handling module for Helena
  ******************************************************************************
  */

/* Debug flag checking -------------------------------------------------------*/
#ifdef BTDEBUG
#ifndef DEBUG
#error "bluetooth debugging requires DEBUG flag"
#endif
#endif

/* Includes ------------------------------------------------------------------*/
#include "nrf_drv_wdt.h"
#include "debug.h"
#include "app_util_platform.h"

#ifdef DEBUG
#include "SEGGER_RTT.h"
#include <string.h>
#endif // DEBUG

#ifdef BTDEBUG
#include "btle.h"
#include "ble.h"
#include "nrf_delay.h"
#include "fpint.h"
#include <stdlib.h>
#include "ble_nus.h"
#include "fstorage.h"
#include "main.h"
#include "Helena_base.h"
#include "i2c.h"
#include "app_error.h"
#include "crc16.h"
#endif // BTDEBUG

/* External variables --------------------------------------------------------*/
#ifdef BTDEBUG
extern uint32_t __StackTop;
extern uint32_t __StackLimit;
extern uint32_t __HeapLimit;
extern uint32_t __HeapBase;
#endif // BTDEBUG

/* Private typedef -----------------------------------------------------------*/
#ifdef DEBUG
typedef struct
{
    uint32_t magicNumber;       // magic number to check integrity
    uint16_t crc;               // crc of magic number to check integrity
    volatile unsigned int WrOff;// rtt write pointer
    volatile unsigned int RdOff;// rtt read pointer
} rttBuf_t;
#endif // DEBUG

#if defined BTDEBUG
typedef void (*debugCommand_t)(char const* pSubcommand);

typedef struct
{
    char const* pCommand;
    debugCommand_t pCommandFunc;
} commands_t;

typedef struct
{
    bool isPending;
    bool volatile eraseInProgress;
} memory_t;
#endif // defined BTDEBUG

/* Private macros ------------------------------------------------------------*/
#ifdef BTDEBUG
#define COUNT_OF(x)     (sizeof(x)/sizeof(x[0]))
#endif // BTDEBUG

/* Private defines -----------------------------------------------------------*/
#ifdef DEBUG
#define MAGICVAILDNUMBER    0x78151503
#endif // DEBUG

/* Private function prototype ------------------------------------------------*/
#ifdef BTDEBUG
static void errorlogCommand(char const* pSubcommand);
static void resetCommand(char const* pSubcommand);
static void memoryCommand(char const* pSubcommand);
static void calibrateCommand(char const* pSubcommand);
#endif // BTDEBUG

/* Private variables ---------------------------------------------------------*/
static nrf_drv_wdt_channel_id watchdogChannel;

#ifdef DEBUG
static volatile rttBuf_t rttBuf __attribute__((section(".noinit")));   // rtt pointers stored in noinit section to survive reset
#endif // DEBUG

#ifdef BTDEBUG
static const commands_t commands[] =
{
    {"errorlog", &errorlogCommand},
    {"reset", &resetCommand},
    {"memory", &memoryCommand},
    {"cal", &calibrateCommand}
};
static memory_t memory;
#endif // BTDEBUG

/* Private functions ---------------------------------------------------------*/
static void watchdogEventHandler()
{
    /// maybe do something
}

#ifdef BTDEBUG
static void errorlogCommand(char const* pSubcommand)
{
    if (strncmp(pSubcommand, " --read", 7) == 0)
    {
        unsigned RdOff = _SEGGER_RTT.aUp[0].RdOff;

        while(RdOff != _SEGGER_RTT.aUp[0].WrOff)
        {
            char* pString = &_SEGGER_RTT.aUp[0].pBuffer[RdOff];
            int16_t len = _SEGGER_RTT.aUp[0].WrOff;             // set len to end
            if (len < RdOff)                            // set to bufferend if rollover
                len = _SEGGER_RTT.aUp[0].SizeOfBuffer;
            len -= RdOff;                               // calculate length
            if (len > BTLE_MAXNUSLENGHT)                // limit length
                len = BTLE_MAXNUSLENGHT;
            uint32_t errCode = btle_SendNusString((uint8_t*)pString, len);
            if (errCode == NRF_SUCCESS)
            {
                RdOff += len;
                RdOff %= _SEGGER_RTT.aUp[0].SizeOfBuffer;
            }
            else if (errCode != BLE_ERROR_NO_TX_BUFFERS)
            {
                APP_ERROR_CHECK(errCode);
            }
        }

    }
    else if (strncmp(pSubcommand, " --clear", 8) == 0)
    {
        static char const rspMsg[] = "done\r\n";
        uint32_t errCode;

        _SEGGER_RTT.aUp[0].RdOff = _SEGGER_RTT.aUp[0].WrOff;
        rttBuf.RdOff = _SEGGER_RTT.aUp[0].RdOff;
        rttBuf.WrOff = _SEGGER_RTT.aUp[0].WrOff;

        errCode = btle_SendNusString((uint8_t*)rspMsg, strlen(rspMsg));
        APP_ERROR_CHECK(errCode);
    }
    else if (strncmp(pSubcommand, " --fake", 7) == 0)
    {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }
    else
    {
        static char const* usage[] =
        {
            "usage:\r\n",
            "--read\r\n",
            "--clear\r\n",
            "--fake\r\n"
        };

        for (uint8_t i = 0; i < COUNT_OF(usage); i++)
        {
            uint32_t errCode;
            do
            {
                errCode = btle_SendNusString((uint8_t*)usage[i], strlen(usage[i]));
            } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        }
    }
}

static void resetCommand(char const* pSubcommand)
{
    static char const rspMsg[] = "resetting...\r\n";
    uint32_t errCode;

    _SEGGER_RTT.aUp[0].WrOff = _SEGGER_RTT.aUp[0].RdOff;

    errCode = btle_SendNusString((uint8_t*)rspMsg, strlen(rspMsg));
    APP_ERROR_CHECK(errCode);

    nrf_delay_ms(1000);
    NVIC_SystemReset();
}

uint32_t debug_FactoryReset(bool reset)
{
    uint32_t errCode;

    errCode = sd_flash_page_erase((FS_PAGE_END_ADDR - 1 * FS_PAGE_SIZE) / FS_PAGE_SIZE);
    if (errCode == NRF_SUCCESS)
        memory.eraseInProgress = true;
    else
        return errCode;

    while (memory.eraseInProgress == true);

    errCode = sd_flash_page_erase((FS_PAGE_END_ADDR - 2 * FS_PAGE_SIZE) / FS_PAGE_SIZE);
    if (errCode == NRF_SUCCESS)
        memory.eraseInProgress = true;
    else
        return errCode;

    while (memory.eraseInProgress == true);

    if (reset)
        NVIC_SystemReset();

    return NRF_SUCCESS;
}

static void memoryClear()
{
    uint32_t errCode;

    memory.isPending = false;

    /*errCode = sd_flash_page_erase((FS_PAGE_END_ADDR - 1 * FS_PAGE_SIZE) / FS_PAGE_SIZE);
    if (errCode == NRF_SUCCESS)
    {
        memory.eraseInProgress = true;
    }
    else
    {
        APP_ERROR_HANDLER(errCode);
    }
    while (memory.eraseInProgress == true);
    errCode = sd_flash_page_erase((FS_PAGE_END_ADDR - 2 * FS_PAGE_SIZE) / FS_PAGE_SIZE);
    if (errCode == NRF_SUCCESS)
    {
        memory.eraseInProgress = true;
    }
    else
    {
        APP_ERROR_HANDLER(errCode);
    }
    while (memory.eraseInProgress == true);*/

    errCode = debug_FactoryReset(false);
    APP_ERROR_CHECK(errCode);

    do
    {
        errCode = btle_SendNusString((uint8_t*)"done, cycle power\r\n", 19);
    } while (errCode == BLE_ERROR_NO_TX_BUFFERS);

    APP_ERROR_CHECK(errCode);
}

static void memoryCommand(char const* pSubcommand)
{
    if (strncmp(pSubcommand, " --stack", 8) == 0)
    {
        char rsp[BTLE_MAXNUSLENGHT+1];
        uint32_t errCode, stackSize, stackUsed;
        uint8_t* pStack;
        uint8_t compare;
        //uint_fast16_t stackSize;

        stackSize = (uint32_t)((uint8_t*)&__StackTop - (uint8_t*)&__StackLimit);

        strcpy(rsp, "size: ");
        itoa(stackSize, &rsp[strlen(rsp)], 10);
        strcat(rsp, "B\r\n");
        do
        {
            errCode = btle_SendNusString((uint8_t*)rsp, strlen(rsp));
        } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        APP_ERROR_CHECK(errCode);

        pStack = (uint8_t*)&__StackLimit;
        compare = 0x55;
        if (*pStack != compare)             // stack overflow
        {
            pStack = (uint8_t*)&__HeapLimit;// check if heap already reached
            compare = 0x33;
        }
        if (*pStack != compare)             // stack already reached heap
        {
            strcpy(rsp, "fucked up!");
        }
        else
        {
            while(*pStack == compare) {pStack++;}
            stackUsed = (uint32_t)((uint8_t*)&__StackTop - pStack);
            strcpy(rsp, "used: ");
            itoa(stackUsed, &rsp[strlen(rsp)], 10);
            strcat(rsp, "B\r\n");
        }
        do
        {
            errCode = btle_SendNusString((uint8_t*)rsp, strlen(rsp));
        } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        APP_ERROR_CHECK(errCode);
    }
    else if (strncmp(pSubcommand, " --heap", 7) == 0)
    {
        char rsp[BTLE_MAXNUSLENGHT+1];
        uint32_t errCode, heapSize, heapUsed;
        uint8_t* pHeap;

        heapSize = (uint32_t)((uint8_t*)&__HeapLimit - (uint8_t*)&__HeapBase);

        strcpy(rsp, "size: ");
        itoa(heapSize, &rsp[strlen(rsp)], 10);
        strcat(rsp, "B\r\n");
        do
        {
            errCode = btle_SendNusString((uint8_t*)rsp, strlen(rsp));
        } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        APP_ERROR_CHECK(errCode);

        pHeap = (uint8_t*)&__HeapBase;
        while (*pHeap == 0xAA) {pHeap++;}
        heapUsed = (uint32_t)(pHeap - (uint8_t*)&__HeapBase);
        strcpy(rsp, "used: ");
        itoa(heapUsed, &rsp[strlen(rsp)], 10);
        strcat(rsp, "B\r\n");
        do
        {
            errCode = btle_SendNusString((uint8_t*)rsp, strlen(rsp));
        } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        APP_ERROR_CHECK(errCode);
    }
    else if (strncmp(pSubcommand, " --clear", 8) == 0)
    {
        memory.isPending = true;
    }
    else
    {
        static char const* usage[] =
        {
            "usage:\r\n",
            "--stack\r\n",
            "--heap\r\n",
            "--clear\r\n"
        };

        for (uint8_t i = 0; i < COUNT_OF(usage); i++)
        {
            uint32_t errCode;
            do
            {
                errCode = btle_SendNusString((uint8_t*)usage[i], strlen(usage[i]));
            } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
            APP_ERROR_CHECK(errCode);
        }
    }
}

static bool isCalSupported()
{
    light_driverRevision_t rev = main_GetDrvRev();

    if (rev == LIGHT_DRIVERREVUNKNOWN || rev < LIGHT_DRIVERREV12)
        return false;
    else
        return true;
}

static bool extractCalib(char const* pS, uint8_t* buf)
{
    int16_t sign, temp;
    q1_7_t gain;

    // start with sign for temperature offset
    if (*pS == '-')
    {
        sign = -1;
        pS++;
    }
    else
        sign = 1;
    if (!(*pS >= '0' && *pS <= '9'))
        return false;

    // extract temperature offset
    temp = 0;
    while (*pS >= '0' && *pS <= '9')
    {
        temp *= 10;
        temp += *pS - '0';
        pS++;
    }
    temp = temp * sign;
    buf[0] = (uint16_t)temp >> 8;
    buf[1] = (uint16_t)temp & 0x00FF;

    pS++;
    if (!(*pS >= '0' && *pS <= '9'))
        return false;

    // extract left side gain
    gain= 0;
    while (*pS >= '0' && *pS <= '9')
    {
        gain *= 10;
        gain += *pS - '0';
        pS++;
    }
    buf[2] = gain;

    pS++;
    if (!(*pS >= '0' && *pS <= '9'))
        return false;

    // extract right side gain
    gain= 0;
    while (*pS >= '0' && *pS <= '9')
    {
        gain *= 10;
        gain += *pS - '0';
        pS++;
    }
    buf[3] = gain;

    buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];

    return true;
}

static void calibrateCommand(char const* pSubcommand)
{
    /// TODO: move to main context, otherwise the i2c read and write functions might fail
    if (isCalSupported() == false)
    {
        uint32_t errCode;
        do
        {
            errCode = btle_SendNusString((uint8_t*)"not supported\r\n", 15);
        } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
    }
    else if (strncmp(pSubcommand, " --read", 7) == 0)
    {
        char rsp[BTLE_MAXNUSLENGHT+1];
        uint32_t errCode;
        uint8_t buffer[4];
        int16_t tempOff;

        errCode = i2c_read(HELENABASE_ADDRESS, HELENABASE_RA_TEMPOFFSET_H, 4, buffer);
        if (errCode != NRF_SUCCESS)
        {
            do
            {
                errCode = btle_SendNusString((uint8_t*)"failed\r\n", 8);
            } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
            APP_ERROR_CHECK(errCode);
            return;
        }
        tempOff = (uint16_t)buffer[0] << 8 | buffer[1];
        itoa(tempOff, rsp, 10);
        strcat(rsp, ",");
        itoa(buffer[2], &rsp[strlen(rsp)], 10);
        strcat(rsp, ",");
        itoa(buffer[3], &rsp[strlen(rsp)], 10);
        do
        {
            errCode = btle_SendNusString((uint8_t*)rsp, strlen(rsp));
        } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        APP_ERROR_CHECK(errCode);
    }
    else if (strncmp(pSubcommand, " --st ", 6) == 0)
    {
        uint32_t errCode;
        uint8_t buffer[5];

        do
        {
            errCode = btle_SendNusString((uint8_t*)"deactivated\r\n", 13);
        } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        APP_ERROR_CHECK(errCode);
        return;

        if (extractCalib(&pSubcommand[6], buffer) == false)
        {
            do
            {
                errCode = btle_SendNusString((uint8_t*)"wrong parameters\r\n", 18);
            } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
            APP_ERROR_CHECK(errCode);
            return;
        }

        errCode = i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_TEMPOFFSET_H, 5, buffer);
        if (errCode != NRF_SUCCESS)
        {
            do
            {
                errCode = btle_SendNusString((uint8_t*)"failed\r\n", 8);
            } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        }
        else
        {
            do
            {
                errCode = btle_SendNusString((uint8_t*)"done\r\n", 6);
            } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
        }
        APP_ERROR_CHECK(errCode);
    }
    else
    {
        static char const* usage[] =
        {
            "usage:\r\n",
            "--read\r\n",
            "--st ot,gl,gr\r\n"
        };

        for (uint8_t i = 0; i < COUNT_OF(usage); i++)
        {
            uint32_t errCode;
            do
            {
                errCode = btle_SendNusString((uint8_t*)usage[i], strlen(usage[i]));
            } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
            APP_ERROR_CHECK(errCode);
        }
    }
}
#endif // BTDEBUG

/* Public functions ----------------------------------------------------------*/
void debug_Init()
{
#ifdef DEBUG
    // check if noinit data is valid
    SEGGER_RTT_Init();
    if (rttBuf.magicNumber == MAGICVAILDNUMBER &&
        rttBuf.crc == crc16_compute((const uint8_t*)&rttBuf.magicNumber, sizeof(rttBuf.magicNumber), NULL))
    {
        // noinit data is valid, use last known read and write pointers
        _SEGGER_RTT.aUp[0].WrOff = rttBuf.WrOff;
        _SEGGER_RTT.aUp[0].RdOff = rttBuf.RdOff;
        SEGGER_RTT_WriteString(0, "Helena reset\r\n");
    }
    else
    {
        // noinit data isn't valid, initialize
        rttBuf.magicNumber = MAGICVAILDNUMBER;
        rttBuf.crc = crc16_compute((const uint8_t*)&rttBuf.magicNumber, sizeof(rttBuf.magicNumber), NULL);
        rttBuf.WrOff = _SEGGER_RTT.aUp[0].WrOff;
        rttBuf.RdOff = _SEGGER_RTT.aUp[0].RdOff;
        SEGGER_RTT_WriteString(0, "Helena power on\r\n");
    }
#endif // DEBUG

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
#if defined BTDEBUG
    if (memory.isPending)
        memoryClear();
#endif
}

#if defined BTDEBUG
void debug_OnNusEvt(uint8_t* pData, uint16_t length)
{
    char const* pString = (char const*)pData;

    // search command
    for (uint8_t i = 0; i < COUNT_OF(commands); i++)
    {
        int16_t len = strlen(commands[i].pCommand);
        if (strncmp(pString, commands[i].pCommand, len) == 0)
        {
            if (commands[i].pCommandFunc != NULL)
                (*commands[i].pCommandFunc)(&pString[len]);
            return;
        }
    }

    // command not found, send list of available commands
    static char const defaultMsg[] = "not found, cmds:\r\n";
    uint32_t errCode;

    errCode = btle_SendNusString((uint8_t*)defaultMsg, strlen(defaultMsg));
    APP_ERROR_CHECK(errCode);
    for (uint8_t i = 0; i < COUNT_OF(commands); i++)
    {
        char command[21];
        if (strlen(commands[i].pCommand) > sizeof(command) - 3)
            continue;
        strcpy(command, commands[i].pCommand);
        strcat(command, ",\r\n");
        do
        {
            errCode = btle_SendNusString((uint8_t*)command, strlen(command));
        } while (errCode == BLE_ERROR_NO_TX_BUFFERS);
    }
}

void debug_OnSysEvent(uint32_t sysEvt)
{
    if (sysEvt == NRF_EVT_FLASH_OPERATION_SUCCESS)
        memory.eraseInProgress = false;
    else if (sysEvt == NRF_EVT_FLASH_OPERATION_ERROR)
        APP_ERROR_CHECK(0);
}
#endif

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    volatile bool loop = true;
#ifdef DEBUG
    loop = false; // no looping when debug flag is set

    const char* pFile;
    pFile = strrchr((char*)p_file_name, '/');
    pFile = pFile == NULL ? (const char*)p_file_name : pFile + 1;

    SEGGER_RTT_printf(0, "0x%02x at %u in %s\r\n", error_code, line_num, pFile);

#ifdef BTDEBUG
    nrf_delay_ms(5);
    rttBuf.RdOff = _SEGGER_RTT.aUp[0].RdOff;
    rttBuf.WrOff = _SEGGER_RTT.aUp[0].WrOff;
#endif // BTDEBUG

#endif // DEBUG

    CRITICAL_REGION_ENTER();
    while(loop);            // reset through watchdog when looping
    CRITICAL_REGION_EXIT();
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

#ifdef DEBUG
    (void)stacked_r0;
    (void)stacked_r1;
    (void)stacked_r2;
    (void)stacked_r3;
    (void)stacked_r12;
    (void)stacked_psr;

    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    SEGGER_RTT_printf (0, "LR [R14] = %x  src ret addr\n", stacked_lr);
    SEGGER_RTT_printf (0, "PC [R15] = %x\n", stacked_pc);

    /*stacked_r0 = ((unsigned long) hardfault_args[0]);
    stacked_r1 = ((unsigned long) hardfault_args[1]);
    stacked_r2 = ((unsigned long) hardfault_args[2]);
    stacked_r3 = ((unsigned long) hardfault_args[3]);

    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    stacked_psr = ((unsigned long) hardfault_args[7]);

    SEGGER_RTT_printf (0, "[Hard fault]\n");
    SEGGER_RTT_printf (0, "R0 = %x\n", stacked_r0);
    SEGGER_RTT_printf (0, "R1 = %x\n", stacked_r1);
    SEGGER_RTT_printf (0, "R2 = %x\n", stacked_r2);
    SEGGER_RTT_printf (0, "R3 = %x\n", stacked_r3);
    SEGGER_RTT_printf (0, "R12 = %x\n", stacked_r12);
    SEGGER_RTT_printf (0, "LR [R14] = %x  subroutine call return address\n", stacked_lr);
    SEGGER_RTT_printf (0, "PC [R15] = %x  program counter\n", stacked_pc);
    SEGGER_RTT_printf (0, "PSR = %x\n", stacked_psr);
    SEGGER_RTT_printf (0, "LR = %x\n", lr_value);*/
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

/**END OF FILE*****************************************************************/
