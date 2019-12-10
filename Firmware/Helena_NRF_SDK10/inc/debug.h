/**
  ******************************************************************************
  * @file    debug.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    15/10/26
  * @brief   header file for error handling module
  *
  * @note    this module handles different error handling strategies, depending
  *          on different debug levels. Below is a list of different levels,
  *          note that not all levels in the list are implemented yet and that
  *          the list is not complete yet. The debug level code shall be defined
  *          in the project settings.
  *
  *          HELENA_DEBUG_NONE
  *          no debugging, device performs a reset if error handler is called
  *
  *          HELENA_DEBUG_SWD
  *          just normal debugging with SWD interface is used.
  *
  *          HELENA_DEBUG_RTT
  *          debugging via SWD interface and additional debug information over
  *          a terminal window using SEGGERS Real Time Terminal (requires J-LINK
  *          debug probe).
  *
  *          BTDEBUG
  *          debugging while no debugger is connected. Debug message will be
  *          logged into an non initialized buffer, and device will be reset.
  *          buffer can be read out via Nordic uart service, or when connecting
  *          debug probe. Debug information will be lost if power is unplugged.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBUG_H
#define DEBUG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
//#include <stdbool.h>
//#include "compiler_abstraction.h"
#if defined BTDEBUG
#include "ble.h"
#endif

/* Exported types ------------------------------------------------------------*/
/*typedef enum
{
    DBG_HELENA = 0, DBG_NRF, DBG_MPU, DBG_CNT
} debugMessageEnum;*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/*#if defined HELENA_DEBUG_SWD
#define MPU_ERROR_HANDLER(ERR_CODE)                         \
    do                                                      \
    {                                                       \
        debug_ErrorHandler(DBG_MPU, (ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
    } while (0)
#define HELENA_ERROR_HANDLER(ERR_CODE)                         \
    do                                                      \
    {                                                       \
        debug_ErrorHandler(DBG_HELENA, (ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
    } while (0)
#else
#define MPU_ERROR_HANDLER(ERR_CODE)                         \
    do                                                      \
    {                                                       \
        debug_ErrorHandler(DBG_MPU, (ERR_CODE), 0, 0);  \
    } while (0)
#define HELENA_ERROR_HANDLER(ERR_CODE)                         \
    do                                                      \
    {                                                       \
        debug_ErrorHandler(DBG_HELENA, (ERR_CODE), 0, 0);  \
    } while (0)
#endif

#define MPU_ERROR_CHECK(ERR_CODE)                           \
    do                                                      \
    {                                                       \
        const int LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != 0)                  \
        {                                                   \
            MPU_ERROR_HANDLER(LOCAL_ERR_CODE);              \
        }                                                   \
    } while (0)
#define HELENA_ERROR_CHECK(ERR_CODE)                           \
    do                                                      \
    {                                                       \
        const int LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != 0)                  \
        {                                                   \
            HELENA_ERROR_HANDLER(LOCAL_ERR_CODE);              \
        }                                                   \
    } while (0)
*/
/* Exported functions ------------------------------------------------------- */
void debug_Init(void);
void debug_Execute(void);
//void debug_ErrorHandler(debugMessageEnum debug, uint32_t err_code, uint32_t line, const uint8_t* pfile);
//void debug_ErrorHandler(uint32_t err_code, uint32_t line, const uint8_t* pfile);
#if defined BTDEBUG
//void debug_FieldTestingInit(void);
//void debug_OnBleEvt(ble_evt_t * pBleEvt);
void debug_OnNusEvt(uint8_t *pData, uint16_t length);
void debug_OnSysEvent(uint32_t sysEvt);
#endif

#endif /* ERROR_H */

/**END OF FILE*****************************************************************/
