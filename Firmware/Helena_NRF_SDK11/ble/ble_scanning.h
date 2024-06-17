/**
  ******************************************************************************
  * @file    ble_scanning.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/06/15
  * @brief   header file for ble_scanning.c
  *          this module was made the same style as the ble_advertising module
  *          provided by Nordic.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_SCANNING_H_INCLUDED
#define BLE_SCANNING_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "ble.h"
#include "ble_gap.h"

/* Exported types ------------------------------------------------------------*/
/**@brief Advertising modes.
*/
typedef enum
{
    BLE_SCAN_MODE_IDLE,                 /**< Idle; no scanning is ongoing and will be started automatically. */
    BLE_SCAN_MODE_FAST,                 /**< Fast scanning; white list will used if available. */
    BLE_SCAN_MODE_SLOW,                 /**< Slow scanning; white list will used if available. */
} ble_scan_mode_t;

/**@brief Scan events.
 *
 * @details These events are propagated to the main application if a handler was provided during
 *          initialization of the Scanning Module. Events for modes that are not used can be
 *          ignored. Similarly, BLE_SCAN_EVT_WHITELIST_REQUEST can be ignored if whitelist
 *          is not used.
 */
typedef enum
{
    BLE_SCAN_EVT_IDLE,                  /**< Idle; no scanning is ongoing an will not be started automatically. */
    BLE_SCAN_EVT_PAUSE,                 /**< scanning module has entered pause mode. */
    BLE_SCAN_EVT_FAST,                  /**< Fast Scanning has started. */
    BLE_SCAN_EVT_SLOW,                  /**< Slow scanning has started. */
    BLE_SCAN_EVT_FAST_WHITELIST,        /**< Fast scanning using the whitelist has started. */
    BLE_SCAN_EVT_SLOW_WHITELIST,        /**< Slow scanning using the whitelist has started.*/
    BLE_SCAN_EVT_WHITELIST_REQUEST,     /**< Request a whitelist from the main application. For whitelist scanning to work, the whitelist must be set when this event occurs. */
    BLE_SCAN_EVT_ADV_REPORT_RECEIVED,   /**< An Advertising report has been received */
} ble_scan_evts_t;

/**@brief Options for the different scanning modes.
 *
 * @details This structure is used to enable or disable scanning modes and to configure time-out
 *          periods and scanning intervals.
 */
typedef struct
{
    bool     ble_scan_whitelist_enabled;/**< Enable or disable use of the whitelist. */
    bool     ble_scan_active_scanning;  /**< Enable or disable active scanning. */
    bool     ble_scan_fast_enabled;     /**< Enable or disable fast scanning mode. */
    uint16_t ble_scan_fast_interval;    /**< Scan interval for fast scanning. */
    uint16_t ble_scan_fast_window;      /**< Scan window for fast scanning. */
    uint16_t ble_scan_fast_timeout;     /**< Time-out (in seconds) for fast scanning. */
    bool     ble_scan_slow_enabled;     /**< Enable or disable slow scanning mode. */
    uint16_t ble_scan_slow_interval;    /**< Scan interval for slow scanning. */
    uint16_t ble_scan_slow_window;      /**< Scan window for slow scanning. */
    uint16_t ble_scan_slow_timeout;     /**< Time-out (in seconds) for slow scanning. */
} ble_scan_modes_config_t;

/**@brief Scan event data structure.
 *
 * @details This data structure is propagated to the main application if a handler was provided during
 *          initialization of the Scanning Module. Events for modes that are not used can be
 *          ignored. Similarly, BLE_SCAN_EVT_WHITELIST_REQUEST can be ignored if whitelist
 *          is not used.
 */
typedef struct
{
    ble_scan_evts_t                  ble_scan_event;    /**< The event source for this event */
    const ble_gap_evt_adv_report_t * p_ble_adv_report;  /**< For the event BLE_SCAN_EVT_ADV_REPORT_RECEIVED, this point to the received advertising report. */
} ble_scan_evt_t;

/**@brief BLE scanning event handler type. */
typedef void (*ble_scanning_evt_handler_t) (const ble_scan_evt_t * const p_scan_evt);

/**@brief BLE scanning error handler type. */
typedef void (*ble_scanning_error_handler_t) (uint32_t nrf_error);

/* Exported constants --------------------------------------------------------*/

/** Defines to make the mode options easier to set during advertising init.*/
#define BLE_ADV_WHITELIST_ENABLED   true
#define BLE_ADV_WHITELIST_DISABLED  false

#define BLE_SCAN_ACTIVE_ENABLED     true
#define BLE_SCAN_ACTIVE_DISABLED    false

#define BLE_SCAN_FAST_ENABLED       true
#define BLE_SCAN_FAST_DISABLED      false

#define BLE_SCAN_SLOW_ENABLED       true
#define BLE_SCAN_SLOW_DISABLED      false

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**@brief Function for handling BLE events.
 *
 * @details This function must be called from the BLE stack event dispatcher for
 *          the module to handle BLE events that are relevant for the Scanning Module.
 *
 * @param[in] p_ble_evt BLE stack event.
 */
void ble_scanning_on_ble_evt(const ble_evt_t * const p_ble_evt);


/**@brief Function for initializing the Scanning Module.
 *
 * @details Builds a structure to be passed to the stack when starting scanning.
 *          The supplied scanning data is copied to a local structure and is manipulated
 *          depending on what scanning modes are started in @ref ble_scanning_start.
 *
 * @param[in] p_config      Select which scanning modes and intervals will be utilized.
 * @param[in] evt_handler   Event handler that will be called upon scanning events.
 * @param[in] error_handler Error handler that will propogate internal errors to the main applications.
 *
 * @retval NRF_SUCCESS If initialization was successful. Otherwise, an error code is returned.
 */
uint32_t ble_scanning_init(ble_scan_modes_config_t const    * p_config,
                           ble_scanning_evt_handler_t const   evt_handler,
                           ble_scanning_error_handler_t const error_handler);


/**@brief Function for starting scanning.
 *
 * @details You can start scanning in any of the scanning modes that you enabled
 *          during initialization.
 *
 * @param[in] scanning_mode  scanning mode.
 *
 * @retval @ref NRF_SUCCESS On success, else an error code indicating reason for failure.
 * @retval @ref NRF_ERROR_INVALID_STATE
 */
uint32_t ble_scanning_start(ble_scan_mode_t scanning_mode);


/**@brief Function for pausing scanning.
 *
 * @param[in] enable    true to pause scanning module, false to resume
 *
 * @retval @ref NRF_SUCCESS On success, else an error code indicating reason for failure.
 * @retval @ref NRF_ERROR_INVALID_STATE
 */
uint32_t ble_scanning_pause(bool enable);

/**@brief Function for setting a whitelist.
 *
 * @details The whitelist must be set by the application upon receiving a
 *          @ref BLE_SCAN_EVT_WHITELIST_REQUEST event. Without the whitelist, the whitelist
 *          scanning for fast and slow modes will not be run.
 *
 * @param[in] p_whitelist  Pointer to a whitelist.
 *
 * @retval @ref NRF_SUCCESS Successfully stored pointers to the whitelist into the scanning module.
 * @retval @ref NRF_ERROR_INVALID_STATE If a reply was not expected.
 */
uint32_t ble_scanning_whitelist_reply(ble_gap_whitelist_t * p_whitelist);


/**@brief Function for disabling whitelist scanning.
 *
 * @details This function temporarily disables whitelist scanning.
 *          Calling this function resets the current time-out countdown.
 *          This is disabled by calling either @ref ble_scanning_start or
 *          ref ble_scanning_pause.
 *
 * @param[in] scanning_mode  scanning mode.
 *
 * @retval @ref NRF_SUCCESS On success, else an error message propogated from the Softdevice.
 */
uint32_t ble_scanning_start_without_whitelist(ble_scan_mode_t scanning_mode);
/** @} */

#endif /* BLE_SCANNING_H_INCLUDED */

/**END OF FILE*****************************************************************/
