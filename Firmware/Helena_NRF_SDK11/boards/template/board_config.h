/**
  ******************************************************************************
  * @file    board_config.h
  * @author  Thomas Reisnecker
  * @brief   compile time configuration for template
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOARD_CONFIG_H_INCLUDED
#define BOARD_CONFIG_H_INCLUDED

/* Configuration -------------------------------------------------------------*/
/**< The max. lenght of the device name, 0 if device name change is not supported */
#define MAX_NAME_LENGTH             0

/**< The low frequency clock configuration */
#define CLOCK_CONFIG                \
{                                   \
    .source = NRF_CLOCK_LF_SRC_RC,  \
    .rc_ctiv = 16,                  \
    .rc_temp_ctiv = 1               \
}

/**< The number of supported peripheral links (only 1 is supported) */
#define PERIPHERAL_LINK_COUNT       1

/**< The number of supported central links (min. 2) */
#define CENTRAL_LINK_COUNT          2

/**< The advertising behavior, see @REF btle_advType_t */
#define ADV_TYPE                    BTLE_ADV_TYPE_ALWAYS_OPEN

/**< The size of the attribute table */
#define ATTR_TAB_SIZE               0x3E8

/**< The number of 128bit base uuid types */
// one for hps and kd2 service, one for dfu and one for nus
#ifdef DEBUG_EXT
#define LONG_UUID_COUNT             3
#else
#define LONG_UUID_COUNT             2
#endif

#endif // BOARD_CONFIG_H_INCLUDED

/**END OF FILE*****************************************************************/
