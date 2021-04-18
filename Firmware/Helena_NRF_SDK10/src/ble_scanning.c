/**
  ******************************************************************************
  * @file    ?.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    ?
  * @brief   ?
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "ble_scanning.h"
#include "app_trace.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define LOG app_trace_log

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static ble_scan_mode_t              m_scan_mode_current;    /**< Current scan mode. */
static ble_scan_modes_config_t      m_scan_modes_config;    /**< Configuration structure. */
static ble_scanning_evt_handler_t   m_evt_handler;          /**< Handler for the scanning events. */
static ble_scanning_error_handler_t m_error_handler;        /**< Handler for the error events of the scanning module. */
static ble_scan_evts_t              m_scan_evt;             /**< Scanning event propogated to the main application. */
static bool                         m_scan_pause;           /**< indicator if scanning module is in pause mode or not */

static ble_gap_whitelist_t          m_whitelist;                                         /**< Struct that points to whitelisted addresses. */
static ble_gap_addr_t             * mp_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT]; /**< Pointer to a list of addresses. Pointed to by the whitelist */
static ble_gap_irk_t              * mp_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];   /**< Pointer to a list of Identity Resolving Keys (IRK). Pointed to by the whitelist */
static bool                         m_whitelist_temporarily_disabled = false;            /**< Flag to keep track of temporary disabling of the whitelist. */
static bool                         m_whitelist_reply_expected = false;                  /**< Flag to verify that whitelist is only set when it is requested. */


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**@brief Function for checking that the whitelist has entries.
 */
static bool whitelist_has_entries(ble_gap_whitelist_t const * whitelist)
{
    if ((whitelist->addr_count != 0) || (whitelist->irk_count != 0))
    {
        return true;
    }
    return false;
}

static uint32_t scanning_start(ble_scan_mode_t scanning_mode)
{
    uint32_t              err_code;
    ble_gap_scan_params_t scan_params;

    if (m_scan_pause == true)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    //stop scanning if necessary
    if (m_scan_mode_current != BLE_SCAN_MODE_IDLE)
    {
        err_code = sd_ble_gap_scan_stop();
        if (err_code != NRF_ERROR_INVALID_STATE && err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    m_scan_mode_current = scanning_mode;

    // If a mode is disabled, continue to the next mode.
    if (m_scan_mode_current == BLE_SCAN_MODE_FAST && !m_scan_modes_config.ble_scan_fast_enabled)
    {
        m_scan_mode_current = BLE_SCAN_MODE_SLOW;
    }
    if ((m_scan_mode_current == BLE_SCAN_MODE_SLOW && !m_scan_modes_config.ble_scan_slow_enabled) ||
        m_scan_mode_current == BLE_SCAN_MODE_IDLE)
    {
        m_scan_mode_current = BLE_SCAN_MODE_IDLE;
        m_scan_evt          = BLE_SCAN_EVT_IDLE;
    }

    // Fetch the whitelist.
    if (   (m_evt_handler != NULL)
        && (m_scan_mode_current == BLE_SCAN_MODE_FAST || m_scan_mode_current == BLE_SCAN_MODE_SLOW)
        && (m_scan_modes_config.ble_scan_whitelist_enabled)
        && (!m_whitelist_temporarily_disabled))
    {
        ble_scan_evt_t scan_evt;
        scan_evt.ble_scan_event = BLE_SCAN_EVT_WHITELIST_REQUEST;
        scan_evt.p_ble_adv_report = NULL;
        m_whitelist_reply_expected = true;
        m_evt_handler(&scan_evt);
    }
    else
    {
        m_whitelist_reply_expected = false;
    }

    //in case whitelist was not provided
    if (m_whitelist_reply_expected)
    {
        m_whitelist_reply_expected = false;
        return NRF_ERROR_NOT_FOUND;
    }

    //check if whitelist has entries
    if (   !whitelist_has_entries(&m_whitelist)
        && m_scan_modes_config.ble_scan_whitelist_enabled
        && !m_whitelist_temporarily_disabled)
    {
        m_scan_mode_current = BLE_SCAN_MODE_IDLE;
        m_scan_evt          = BLE_SCAN_EVT_IDLE;
        LOG("[SCAN]: No Devices to scan for, going into idle mode.\r\n");
    }

    // Set scanning parameters and events according to selected advertising mode.
    switch (m_scan_mode_current)
    {
        case BLE_SCAN_MODE_FAST:
            scan_params.active   = m_scan_modes_config.ble_scan_active_scanning;
            scan_params.interval = m_scan_modes_config.ble_scan_fast_interval;
            scan_params.window   = m_scan_modes_config.ble_scan_fast_window;
            scan_params.timeout  = m_scan_modes_config.ble_scan_fast_timeout;

            if (m_whitelist_temporarily_disabled || !m_scan_modes_config.ble_scan_whitelist_enabled)
            {
                scan_params.selective   = 0;
                scan_params.p_whitelist = NULL;

                m_scan_evt = BLE_SCAN_EVT_FAST;
                LOG("[SCAN]: Starting fast scanning.\r\n");
            }
            else
            {
                scan_params.selective   = 1;
                scan_params.p_whitelist = &m_whitelist;

                m_scan_evt = BLE_SCAN_EVT_FAST_WHITELIST;
                LOG("[SCAN]: Starting fast scanning with withlist.\r\n");
            }
            break;

        case BLE_SCAN_MODE_SLOW:
            scan_params.active   = m_scan_modes_config.ble_scan_active_scanning;
            scan_params.interval = m_scan_modes_config.ble_scan_slow_interval;
            scan_params.window   = m_scan_modes_config.ble_scan_slow_window;
            scan_params.timeout  = m_scan_modes_config.ble_scan_slow_timeout;

            if (m_whitelist_temporarily_disabled || !m_scan_modes_config.ble_scan_whitelist_enabled)
            {
                scan_params.selective   = 0;
                scan_params.p_whitelist = NULL;

                m_scan_evt = BLE_SCAN_EVT_SLOW;
                LOG("[SCAN]: Starting slow scanning.\r\n");
            }
            else
            {
                scan_params.selective   = 1;
                scan_params.p_whitelist = &m_whitelist;

                m_scan_evt = BLE_SCAN_EVT_SLOW_WHITELIST;
                LOG("[SCAN]: Starting slow scanning with withlist.\r\n");
            }
            break;

        default:
            break;
    }

    if (m_scan_mode_current != BLE_SCAN_MODE_IDLE)
    {
        err_code = sd_ble_gap_scan_start(&scan_params);
        if(err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    if (m_evt_handler != NULL)
    {
        ble_scan_evt_t scan_evt;
        scan_evt.ble_scan_event = m_scan_evt;
        scan_evt.p_ble_adv_report = NULL;
        m_evt_handler(&scan_evt);
    }

    //m_whitelist_temporarily_disabled = false;

    return NRF_SUCCESS;
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_scanning_init(ble_scan_modes_config_t const    * p_config,
                           ble_scanning_evt_handler_t const   evt_handler,
                           ble_scanning_error_handler_t const error_handler)
{
    if (p_config == NULL)
    {
        return NRF_ERROR_NULL;
    }
    m_scan_mode_current = BLE_SCAN_MODE_IDLE;
    m_scan_modes_config = *p_config;
    m_evt_handler       = evt_handler;
    m_error_handler     = error_handler;

    // Prepare Whitelist. Address and IRK double pointers point to allocated arrays.
    m_whitelist.pp_addrs = mp_whitelist_addr;
    m_whitelist.pp_irks  = mp_whitelist_irk;

    return NRF_SUCCESS;
}

uint32_t ble_scanning_start(ble_scan_mode_t scanning_mode)
{
    m_whitelist_temporarily_disabled = false;

    return scanning_start(scanning_mode);
}

void ble_scanning_on_ble_evt(const ble_evt_t * const p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_TIMEOUT:
        if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
        {
            switch (m_scan_mode_current)
            {
            case BLE_SCAN_MODE_FAST:
                LOG("[ADV]: Timed out from fast advertising.\r\n");
                m_scan_mode_current = BLE_SCAN_MODE_SLOW;
                if (m_scan_pause == false)
                {
                    err_code = scanning_start(BLE_SCAN_MODE_SLOW);
                    if ((err_code != NRF_SUCCESS) && (m_error_handler != NULL))
                    {
                        m_error_handler(err_code);
                    }
                }
                break;
            case BLE_SCAN_MODE_SLOW:
                LOG("[ADV]: Timed out from slow advertising.\r\n");
                m_scan_mode_current = BLE_SCAN_MODE_IDLE;
                if (m_evt_handler != NULL)
                {
                    ble_scan_evt_t scan_evt;
                    scan_evt.ble_scan_event = BLE_SCAN_EVT_IDLE;
                    scan_evt.p_ble_adv_report = NULL;
                    m_evt_handler(&scan_evt);
                }
                break;
            default:
                break;
            }
        }
        else if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
        {
    case BLE_GAP_EVT_CONNECTED:
    case BLE_GAP_EVT_DISCONNECTED:
            m_scan_mode_current = BLE_SCAN_MODE_FAST;
            if (m_scan_pause == false)
            {
                err_code = scanning_start(BLE_SCAN_MODE_FAST);
                if ((err_code != NRF_SUCCESS) && (m_error_handler != NULL))
                {
                    m_error_handler(err_code);
                }
            }
        }
        break;
    case BLE_GAP_EVT_ADV_REPORT:
        if (m_evt_handler != NULL && m_scan_mode_current != BLE_SCAN_MODE_IDLE && !m_scan_pause)
        {
            ble_scan_evt_t scan_evt;
            scan_evt.ble_scan_event = BLE_SCAN_EVT_ADV_REPORT_RECEIVED;
            scan_evt.p_ble_adv_report = &p_ble_evt->evt.gap_evt.params.adv_report;
            m_evt_handler(&scan_evt);
        }
        break;
    default:
        break;
    }
}

uint32_t ble_scanning_pause(bool enable)
{
    uint32_t              err_code;

    if (enable == m_scan_pause)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    //stop scanning if necessary
    if (m_scan_mode_current != BLE_SCAN_MODE_IDLE && enable == true)
    {
        err_code = sd_ble_gap_scan_stop();
        if (err_code != NRF_SUCCESS)
        {
            m_scan_pause = true;
            return err_code;
        }
    }

    m_scan_pause = enable;

    if (enable)
    {
        if(m_evt_handler != NULL)
        {
            ble_scan_evt_t scan_evt;
            scan_evt.ble_scan_event = BLE_SCAN_EVT_PAUSE;
            scan_evt.p_ble_adv_report = NULL;
            m_evt_handler(&scan_evt);
        }
    }
    else
    {
        return scanning_start(m_scan_mode_current);
    }

    return NRF_SUCCESS;
}

uint32_t ble_scanning_whitelist_reply(ble_gap_whitelist_t * p_whitelist)
{
    uint32_t i;

    if(m_whitelist_reply_expected == false)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    m_whitelist.addr_count = p_whitelist->addr_count;
    m_whitelist.irk_count  = p_whitelist->irk_count;

    for (i = 0; i < m_whitelist.irk_count; i++)
    {
        mp_whitelist_irk[i] = p_whitelist->pp_irks[i];
    }

    for (i = 0; i < m_whitelist.addr_count; i++)
    {
        mp_whitelist_addr[i] = p_whitelist->pp_addrs[i];
    }

    m_whitelist_reply_expected = false;
    return NRF_SUCCESS;
}

uint32_t ble_scanning_start_without_whitelist(ble_scan_mode_t scanning_mode)
{
    uint32_t err_code;

    if(     m_scan_modes_config.ble_scan_whitelist_enabled == BLE_ADV_WHITELIST_ENABLED
        && !m_whitelist_temporarily_disabled)
    {
        m_whitelist_temporarily_disabled = true;

        err_code = scanning_start(scanning_mode);
        if ((err_code != NRF_SUCCESS) && (m_error_handler != NULL))
        {
            m_error_handler(err_code);
        }
    }
    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
