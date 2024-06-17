/**
  ******************************************************************************
  * @file    loh.h
  * @author  Thomas Reisnecker
  * @brief   logger module headerfile
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOG_H_INCLUDED
#define LOG_H_INCLUDED

/* Exported types ------------------------------------------------------------*/
#define LOG_LEVEL_DISABLED  0
#define LOG_LEVEL_ERROR     1
#define LOG_LEVEL_INFO      2

/*typedef enum
{
    LOG_LEVEL_DISABLED = 0,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_INFO
} log_level_t;*/

/* Exported constants --------------------------------------------------------*/
#ifndef LOG_LEVEL
#ifdef DEBUG
#define LOG_LEVEL       LOG_LEVEL_ERROR
#else
#define LOG_LEVEL       LOG_LEVEL_DISABLED
#endif // DEBUG
#endif // LOG_LEVEL

/* Includes ------------------------------------------------------------------*/
#if LOG_LEVEL != LOG_LEVEL_DISABLED
#include "SEGGER_RTT.h"
#endif // LOG_LEVEL

/* Exported macros -----------------------------------------------------------*/
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOG_ERROR(...)                  \
do                                      \
{                                       \
    SEGGER_RTT_printf(0, __VA_ARGS__);  \
    SEGGER_RTT_printf(0, "\r\n");       \
} while(0)
#else
#define LOG_ERROR(...)
#endif // LOG_LEVEL

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(...)                   \
do                                      \
{                                       \
    SEGGER_RTT_printf(0, __VA_ARGS__);  \
    SEGGER_RTT_printf(0, "\r\n");       \
} while(0)
#else
#define LOG_INFO(...)
#endif // LOG_LEVEL

/* Exported functions ------------------------------------------------------- */
void log_Init(void);

#endif /* LOG_H_INCLUDED */

/**END OF FILE*****************************************************************/
