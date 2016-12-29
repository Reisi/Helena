#ifndef BOARD_CUSTOM_H
#define BOARD_CUSTOM_H

#define LEDS_NUMBER     2
#define BUTTONS_NUMBER  1
#define BUTTON_PULL     NRF_GPIO_PIN_PULLUP

#if defined HELENA_REV20
#define BSP_LED_0       24
#define BSP_LED_1       25
#define BSP_BUTTON_0    27
#elif defined HELENA_REV21
#define BSP_LED_0       27
#define BSP_LED_1       10
#define BSP_BUTTON_0    11
#else
#error "helena board revision not defined"
#endif

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION

#endif
