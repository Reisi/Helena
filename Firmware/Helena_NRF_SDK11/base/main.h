/**
  ******************************************************************************
  * @file    main.h
  * @author  Thomas Reisnecker
  * @brief   Header for main.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "app_timer.h"
#include "section_vars.h"
#include "softdevice_handler.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    sys_evt_handler_t pHandler;
    //void            * pContext;
} const main_SysEventHandler_t;

NRF_SECTION_VARS_REGISTER_SYMBOLS(main_SysEventHandler_t, sys_events);

/* Exported constants --------------------------------------------------------*/
#define MAIN_TIMER_PRESCALER        0

/* Exported macros -----------------------------------------------------------*/
#define MAIN_TIMER_TICKS(x)         APP_TIMER_TICKS(x, MAIN_TIMER_PRESCALER)

#define SYSEVENT_REGISTER(event)    NRF_SECTION_VARS_ADD(sys_events, event)

/* Exported functions ------------------------------------------------------- */
ret_code_t main_ResetIdleTimer(void);

#endif /* MAIN_H */

/**END OF FILE*****************************************************************/
