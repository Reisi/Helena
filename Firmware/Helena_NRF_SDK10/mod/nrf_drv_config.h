/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef NRF_DRV_CONFIG_H
#define NRF_DRV_CONFIG_H

/* CLOCK */
#define CLOCK_CONFIG_XTAL_FREQ          NRF_CLOCK_XTALFREQ_Default
#define CLOCK_CONFIG_LF_SRC             NRF_CLOCK_LF_SRC_RC
#define CLOCK_CONFIG_LF_RC_CAL_INTERVAL RC_2000MS_CALIBRATION_INTERVAL
#define CLOCK_CONFIG_IRQ_PRIORITY       APP_IRQ_PRIORITY_LOW

/* GPIOTE */
#define GPIOTE_ENABLED 1

#if (GPIOTE_ENABLED == 1)
#define GPIOTE_CONFIG_USE_SWI_EGU false
#define GPIOTE_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_HIGH
#define GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 2
#endif

/* TIMER */
#define TIMER0_ENABLED 0

#define TIMER1_ENABLED 1

#if (TIMER1_ENABLED == 1)
#define TIMER1_CONFIG_FREQUENCY    NRF_TIMER_FREQ_2MHz
#define TIMER1_CONFIG_MODE         TIMER_MODE_MODE_Timer
#define TIMER1_CONFIG_BIT_WIDTH    TIMER_BITMODE_BITMODE_08Bit
#define TIMER1_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_HIGH

#define TIMER1_INSTANCE_INDEX      (TIMER0_ENABLED)
#endif

#define TIMER2_ENABLED 0
#define TIMER_COUNT (TIMER0_ENABLED + TIMER1_ENABLED + TIMER2_ENABLED)

/* RTC */
#define RTC0_ENABLED 0
#define RTC1_ENABLED 0
#define RTC_COUNT                (RTC0_ENABLED+RTC1_ENABLED)
#define NRF_MAXIMUM_LATENCY_US 2000

/* RNG */
#define RNG_ENABLED 0

/* SPI */
#define SPI0_ENABLED 0
#define SPI1_ENABLED 0

/* UART */
#define UART0_ENABLED 0

#define TWI0_ENABLED 1

#if (TWI0_ENABLED == 1)
#define TWI0_CONFIG_FREQUENCY    NRF_TWI_FREQ_100K
#define TWI0_CONFIG_SCL          0
#define TWI0_CONFIG_SDA          1
#define TWI0_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TWI0_INSTANCE_INDEX      0
#endif

#define TWI1_ENABLED 0

#if (TWI1_ENABLED == 1)
#define TWI1_CONFIG_FREQUENCY    NRF_TWI_FREQ_100K
#define TWI1_CONFIG_SCL          0
#define TWI1_CONFIG_SDA          1
#define TWI1_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

#define TWI1_INSTANCE_INDEX      (TWI0_ENABLED)
#endif

#define TWI_COUNT                (TWI0_ENABLED+TWI1_ENABLED)

/* QDEC */
#define QDEC_ENABLED 0

/* LPCOMP */
#define LPCOMP_ENABLED 0

/* WDT */
#define WDT_ENABLED 1

#if (WDT_ENABLED == 1)
#define WDT_CONFIG_BEHAVIOUR     NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT
#define WDT_CONFIG_RELOAD_VALUE  32768
#define WDT_CONFIG_IRQ_PRIORITY  APP_IRQ_PRIORITY_HIGH
#endif

#include "nrf_drv_config_validation.h"

#endif // NRF_DRV_CONFIG_H
