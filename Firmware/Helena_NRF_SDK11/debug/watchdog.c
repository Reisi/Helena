/**
  ******************************************************************************
  * @file    watchdog.c
  *          https://devzone.nordicsemi.com/f/nordic-q-a/30075/saving-the-location-of-watchdog-timeout
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define BTLE_LOG_ENABLED

#ifdef BTLE_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BTLE_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "nrf_drv_wdt.h"
#include "app_util_platform.h"
#include "string.h"
#include "nrf51.h"
//#include "debug.h"

#ifdef DEBUG_EXT
#include "crc16.h"
#endif // DEBUG_EXT

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t r0;  ///< R0 register.
    uint32_t r1;  ///< R1 register.
    uint32_t r2;  ///< R2 register.
    uint32_t r3;  ///< R3 register.
    uint32_t r12; ///< R12 register.
    uint32_t lr;  ///< Link register.
    uint32_t pc;  ///< Program counter.
    uint32_t psr; ///< Program status register.
} stack_t;

typedef struct
{
    //stack_t stack;
    uint32_t pc;
    uint16_t crc;
} savedPC_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static nrf_drv_wdt_config_t const config =
{
    .behaviour          = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT,
    .reload_value       = 4000,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH
};

static nrf_drv_wdt_channel_id watchdogChannel;

#ifdef DEBUG_EXT
static savedPC_t savedPC __attribute__((section(".noinit")));
#endif // DEBUG_EXT

/* Private functions ---------------------------------------------------------*/
#ifdef DEBUG_EXT
void dumpStack(stack_t const * pStack)
{
    savedPC.pc = pStack->pc;
    savedPC.crc = crc16_compute((const uint8_t *)&savedPC.pc, sizeof(savedPC.pc), NULL);

    //memcpy(&savedPC.stack, pStack, sizeof(stack_t));
    /*Note: RAM is not guaranteed to be retained retained through WD reset:
      CRC may be used to verify the data integrity*/
    //savedPC.crc = crc16_compute((const uint8_t *)&savedPC.stack, sizeof(stack_t), NULL);
}

static void watchdogHandler()
{
    // dump stack
    __ASM volatile(
                     "   mrs r0, msp                             \n"
                     "   ldr r3, =dumpStack                      \n"
                     "   bx r3                                   \n"
                  );
}

static void checkResetReason()
{
    uint32_t resetReason;
    uint16_t crc;

    resetReason = NRF_POWER->RESETREAS;//nrf_power_resetreas_get();
    NRF_POWER->RESETREAS = resetReason;//nrf_power_resetreas_clear(resetReason);

    // check if reset was caused by watchdog
    if (resetReason & POWER_RESETREAS_DOG_Msk)
    {
        //crc = crc16_compute((const uint8_t *)&savedPC.stack, sizeof(stack_t), NULL);
        crc = crc16_compute((const uint8_t *)&savedPC.pc, sizeof(savedPC.pc), NULL);
        if (crc == savedPC.crc)
        {
            LOG_ERROR("[wdt]: reset! PC 0x%08X", savedPC.pc);
        }
        else
        {
            LOG_ERROR("[wdt]: reset!");
        }
    }
    memset(&savedPC, 0, sizeof(savedPC_t));
}
#else
static void watchdogHandler()
{

}
#endif // DEBUG_EXT


/* Public functions ----------------------------------------------------------*/
ret_code_t wdg_Init()
{
    ret_code_t errCode;

#ifdef DEBUG_EXT
    checkResetReason();
#endif // DEBUG_EXT

    errCode = nrf_drv_wdt_init(&config, watchdogHandler);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[wdt]: init error %d", errCode);
        return errCode;
    }

    errCode = nrf_drv_wdt_channel_alloc(&watchdogChannel);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[wdt]: channel allocation error %d", errCode);
        return errCode;
    }

    nrf_drv_wdt_enable();
    nrf_drv_wdt_channel_feed(watchdogChannel);

    return NRF_SUCCESS;
}

void wdg_Feed()
{
    nrf_drv_wdt_channel_feed(watchdogChannel);
}

/**END OF FILE*****************************************************************/
