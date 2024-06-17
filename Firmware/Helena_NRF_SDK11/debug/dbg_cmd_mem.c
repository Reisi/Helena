/**
  ******************************************************************************
  * @file    dbg_cmd_mem.c
  * @author  Thomas Reisnecker
  * @brief   debug extension for memory informations over ble
  ******************************************************************************
  */

#ifdef DEBUG_EXT

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "string.h"
#include "stdlib.h"

/* External variables --------------------------------------------------------*/
extern uint32_t __StackTop;
extern uint32_t __StackLimit;
extern uint32_t __HeapLimit;
extern uint32_t __HeapBase;

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(array)   (sizeof(array)/sizeof(array[0]))

/* Private defines -----------------------------------------------------------*/
#define STACKPATTERN    0x55
#define HEAPPATTERN     0xAA
#define FREEPATTERN     0x33

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
__attribute__ ((constructor)) static void markMemory()
{
    // fill heap with pattern
    memset((uint8_t*)&__HeapBase, HEAPPATTERN, (uint8_t*)&__HeapLimit - (uint8_t*)&__HeapBase);

    // fill free space with pattern
    memset((uint8_t*)&__HeapLimit, FREEPATTERN, (uint8_t*)&__StackLimit - (uint8_t*)&__HeapLimit);

    // Messing with the stack is not a good idea, but this function is called
    // before main() so that there is not any interrupt active, which would
    // change the stack pointer. The call to memset() on the other hand will
    // change the stack pointer. Therefore the size to be written is reduced by
    // 256 Bytes. The stack usage is definitely bigger, so that should not be
    // an issue.
    memset((uint8_t*)&__StackLimit, STACKPATTERN, ((uint8_t*)GET_SP() - (uint8_t*)&__StackLimit) - 256);
}

static void dbg_cmd_Mem(char const* pSubcommand, uint16_t length, uint16_t connHandle)
{
    if (length >= 8 && strncmp(pSubcommand, " --stack", 8) == 0)
    {
        ret_code_t errCode;
        char rsp[DBG_MAX_DATA_LEN + 1];     // +1 for \0
        uint32_t stackSize, stackUsed;
        uint8_t* pStack;
        uint8_t compare;
        uint16_t len;

        // determine stack size and send
        stackSize = (uint32_t)((uint8_t*)&__StackTop - (uint8_t*)&__StackLimit);
        strcpy(rsp, "size: ");
        itoa(stackSize, &rsp[strlen(rsp)], 10);
        strcat(rsp, "B\r\n");
        len = strlen(rsp);
        errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        if (errCode != NRF_SUCCESS)
            return;

        // determine stack usage
        /// TODO: maybe check size even if overflow occured by checking free space
        pStack = (uint8_t*)&__StackLimit;
        compare = STACKPATTERN;
        if (*pStack != compare)             // stack overflow
        {
            strcpy(rsp, "overflow!");
        }
        else
        {
            while(*pStack == compare) {pStack++;}
            stackUsed = (uint32_t)((uint8_t*)&__StackTop - pStack);
            strcpy(rsp, "used: ");
            itoa(stackUsed, &rsp[strlen(rsp)], 10);
            strcat(rsp, "B\r\n");
        }
        len = strlen(rsp);
        errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        if (errCode != NRF_SUCCESS)
            return;
    }
    else if (length >= 7 && strncmp(pSubcommand, " --heap", 7) == 0)
    {
        ret_code_t errCode;
        char rsp[BLE_NUS_MAX_DATA_LEN + 1];
        uint32_t heapSize, heapUsed;
        uint8_t* pHeap;
        uint16_t len;

        // determine heap size and send
        heapSize = (uint32_t)((uint8_t*)&__HeapLimit - (uint8_t*)&__HeapBase);
        if (heapSize == 0)
            strcpy(rsp, "not used");
        else
        {
            strcpy(rsp, "size: ");
            itoa(heapSize, &rsp[strlen(rsp)], 10);
            strcat(rsp, "B\r\n");
        }

        len = strlen(rsp);
        errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        if (errCode != NRF_SUCCESS || heapSize == 0)
            return;

        // determine head usage
        pHeap = (uint8_t*)&__HeapLimit;
        do {pHeap--;} while (*pHeap == HEAPPATTERN);
        heapUsed = (uint32_t)(pHeap - (uint8_t*)&__HeapBase) + 1;
        strcpy(rsp, "used: ");
        itoa(heapUsed, &rsp[strlen(rsp)], 10);
        strcat(rsp, "B\r\n");
        len = strlen(rsp);
        errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        if (errCode != NRF_SUCCESS)
            return;
    }
    else
    {
        static char const* usage[] =
        {
            "usage:\r\n--stack\r\n",
            "--heap\r\n"
        };

        for (uint8_t i = 0; i < ARRAY_SIZE(usage); i++)
        {
            ret_code_t errCode;
            uint16_t len = strlen(usage[i]);
            errCode = dbg_DataSend((uint8_t*)usage[i], &len, connHandle);
            if (errCode != NRF_SUCCESS)
                return;
        }
    }
}

/* section variable ----------------------------------------------------------*/
DEBUG_REGISTER(const dbg_commands_t dbg_cmd_mem) =
{
    .pCommandName    = "memory",
    .pCommandInit    = NULL,
    .pCommandFunc    = dbg_cmd_Mem,
    .pCommandExecute = NULL,
};

#endif // DEBUG_EXT

/**END OF FILE*****************************************************************/
