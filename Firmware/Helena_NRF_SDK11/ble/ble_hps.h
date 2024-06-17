/**
  ******************************************************************************
  * @file    ble_hps.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @brief   Helen Project Service implementation
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_HPS_H_INCLUDED
#define BLE_HPS_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/* Exported defines ----------------------------------------------------------*/
#define BLE_UUID_HPS_SERVICE    0x0301      /**< The UUID of the Helen Project service */

#define BLE_HPS_GATT_STATUS_ATTERR_INVALID_PARAM BLE_GATT_STATUS_ATTERR_APP_BEGIN

#define BLE_HPS_CP_MAX_RSP_LEN      3

#define BLE_HPS_MIN_BUF_SIZE(sizeofmodes)   (sizeofmodes) //((sizeofmodes / 18 + 1) * 24 + 2)

/* Exported types ------------------------------------------------------------*/
/**@brief Helen Project Service Event Types */
typedef enum
{
    BLE_HPS_EVT_M_NOTIVICATION_ENABLED, /**< Measurements notification enabled event */
    BLE_HPS_EVT_M_NOTIVICATION_DISABLED,/**< Measurements notification disabled event */
    BLE_HPS_EVT_CP_INDICATION_ENABLED,  /**< Control Point notification enabled event */
    BLE_HPS_EVT_CP_INDICATION_DISABLED, /**< Control Point notification disabled event */
    BLE_HPS_EVT_MODE_CONFIG_CHANGED,    /**< Request to change the Mode Configuration */
    BLE_HPS_EVT_CP_WRITE,               /**< control point has been written */
} ble_hps_evt_type_t;

typedef enum
{
    BLE_HPS_CP_REQ_MODE          = 1,
    BLE_HPS_CP_SET_MODE          = 2,
    BLE_HPS_CP_REQ_SEARCH        = 3,
    BLE_HPS_CP_REQ_FACTORY_RESET = 5,
    BLE_HPS_CP_SET_MODE_OVERRIDE = 8,
    BLE_HPS_CP_RESPONSE_CODE     = 32,
} ble_hps_cp_opcode_t;

/**@brief Control Control Point response parameter */
typedef enum
{
    BLE_HPS_CP_RSP_VAL_SUCCESS       = 1,
    BLE_HPS_CP_RSP_VAL_NOT_SUPPORTED = 2,
    BLE_HPS_CP_RSP_VAL_INVALID       = 3,
    BLE_HPS_CP_RSP_VAL_FAILED        = 4
} ble_hps_cp_rsp_val_t;

/**@brief Control Control Point procedure status */
typedef enum
{
    BLE_HPS_CP_PROC_STATUS_FREE             = 0,
    BLE_HPS_CP_PROC_IN_PROGRESS             = 1,
    BLE_HPS_CP_RSP_CODE_IND_PENDING         = 2,
    BLE_HPS_CP_RSP_CODE_IND_CONFIRM_PENDING = 3
} ble_hps_cp_proc_status_t;

/**@brief Control Control Point response indication structure */
typedef struct
{
    uint8_t              len;
    uint8_t              encoded_rsp[BLE_HPS_CP_MAX_RSP_LEN];
} ble_hps_cp_rsp_ind_t;

/** @brief Client specific information */
typedef struct
{
    uint16_t conn_handle;                       /**< Handle of the connection (as provided by the BLE stack) */
    bool is_m_notfy_enabled;                    /**< Indicator if measurement notifications are enabled */
    bool is_cp_indic_enabled;                   /**< Indicator if control point indications are enabled */
    ble_hps_cp_proc_status_t procedure_status;  /**< status of possible procedure */
    ble_hps_cp_rsp_ind_t     pending_response;  /**< pending response data */
} ble_hps_client_spec_t;

/** @brief Helen Project mode configuration */
typedef struct
{
    uint16_t ignore         : 1;
    uint16_t first_in_group : 1;
    uint16_t last_in_group  : 1;
    uint16_t is_pref_mode   : 1;
    uint16_t is_temp_mode   : 1;
    uint16_t is_off_mode    : 1;
} ble_hps_mode_config_t;

/** @brief the event parameters for @ref BLE_HPS_EVT_MODE_CONFIG_CHANGED */
typedef struct
{
    ble_hps_mode_config_t const * p_mode_config;    /**< pointer to array of size mode_count */
    void                  const * p_channel_config; /**< pointer to the channel configuration */
    uint16_t                      total_size;       /**< total size of characteristic in bytes */
} ble_hps_modes_t;

/** @brief the control point event parameters for @ref BLE_HPS_CP_REQ_MODE_OVERRIDE */
typedef struct
{
    void const * p_config;
    uint16_t     size;
} ble_hps_cp_channel_config_t;

/** @brief the event parameters for @ref BLE_HPS_EVT_CP_WRITE */
typedef struct
{
    ble_hps_cp_opcode_t opcode;
    union
    {
        uint8_t                     mode_to_set;       /**< the requested mode for @ref BLE_HPS_CP_SET_MODE */
        ble_hps_cp_channel_config_t channel_config;   /**< the channel configuration for @ref BLE_HPS_CP_REQ_MODE_OVERRIDE */
    };
} ble_hps_cp_t;

/** @brief event parameters */
typedef union
{
    ble_hps_modes_t modes;
    ble_hps_cp_t    ctrl_pt;
} ble_hps_evt_params_t;

// Forward declaration of the ble_hps_t type.
typedef struct ble_hps_s ble_hps_t;

/**@brief Helen Project Service Event */
typedef struct
{
    ble_hps_evt_type_t            evt_type; /**< the type of the event */
    ble_hps_t const             * p_hps;    /**< a pointer to the instance */
    ble_hps_client_spec_t const * p_client; /**< a pointer to the client data */
    ble_hps_evt_params_t          params;   /**< the event parameters */
} ble_hps_evt_t;

/**@brief Helen Project Service event handler type
 * @return true if the request is acceptable, otherwise false
 */
typedef bool (*ble_hps_evt_handler_t)(ble_hps_evt_t const * p_evt);

/** @brief channel descriptors */
typedef enum
{
    BLE_HPS_CH_DESC_USER    = 0,
    BLE_HPS_CH_DESC_CURRENT = 1,
    BLE_HPS_CH_DESC_VOLTAGE = 2,
    BLE_HPS_CH_DESC_PWM     = 3,
    BLE_HPS_CH_DESC_SWITCH  = 4
} ble_hps_ch_desc_t;

/** @ref feature characteristic channel size field */
typedef struct
{
    uint8_t           channel_bitsize;      /**< the total size in bit of the channel configuration field */
    uint8_t           sp_ftr_bitsize;       /**< the bit size of the special feature */
    ble_hps_ch_desc_t channel_description;  /**< the description of the channel */
} ble_hps_f_ch_size_t;

/** @ref feature characteristic flags */
typedef struct
{
    uint16_t mode_set_supported       : 1;  /**< indicating if mode setting is supported */
    uint16_t search_request_supported : 1;  /**< indicating if search can be requested */
    uint16_t factory_reset_supported  : 1;  /**< indicating if a device factory reset can be requested */
    uint16_t mode_override_supported  : 1;  /**< indicating if mode overriding is supported */
} ble_hps_f_flags_t;

/** @ref feature characteristic initialization structure */
typedef struct
{
    uint8_t                     mode_count;     /**< the number of available modes */
    uint8_t                     channel_count;  /**< the number of available channels */
    ble_hps_f_ch_size_t const * p_channel_size; /**< pointer to array of channel size */
    ble_hps_f_flags_t           flags;          /**< the feature flags */
} ble_hps_feature_init_t;

/** @ref feature characteristic */
typedef struct
{
    uint8_t             mode_count;             /**< the number of available modes */
    uint8_t             channel_count;          /**< the number of available channels */
    ble_hps_f_flags_t   flags;                  /**< the feature flags */
} ble_hps_feature_t;

/** @brief Helen Project modes characteristic
 *  @note the mode characteristic is not initialized at the stack, therefor
 *        these pointers need to point to static memory and the channel
 *        configuration must be directly subsequent to the mode configuration.*/
typedef struct
{
    ble_hps_mode_config_t * p_mode_config;    /**< pointer to array of size mode_count */
    void                  * p_channel_config; /**< pointer to the channel configuration */
    uint16_t                total_size;       /**< total size of characteristic in bytes */
} ble_hps_modes_init_t;

/**@brief Helen Project Service queued write buffer structure.
 * @note  This buffer must be provided by the application if the modes characteristics is
 *        to long for single write operations. */
typedef struct
{
    bool volatile inUse;                    /**< indicator if the buffer is free */
    ble_user_mem_block_t const * p_block;   /**< pointer to the buffer */
} ble_hps_qwr_buffer_t;

/**@brief Helen Project Service init structure. This contains all options and data
 *        needed for initialization of the service */
typedef struct
{
    ble_hps_feature_init_t  features;           /**< the supported features */
    ble_hps_modes_init_t    modes;              /**< the initial mode configuration */
    security_req_t          modes_wr_sec;       /**< Security requirement for writing Helen Modes characteristic. */
    security_req_t          cp_wr_sec;          /**< Security requirement for writing control point characteristic. */
    ble_hps_evt_handler_t   evt_handler;        /**< Event handler to be called for handling events in the Helen Project Service */
    ble_srv_error_handler_t error_handler;      /**< Function to be called in case of an error. */
    ble_hps_qwr_buffer_t  * p_qwr_buf;          /**< pointer to the buffer for queued write operations */
} ble_hps_init_t;

/**@brief Helen Project Service structure. This contains various status
 *        information for the service */
struct ble_hps_s
{
    uint8_t                     uuid_type;          /**< UUID type for Helen Project Service Base UUID. */
    uint16_t                    service_handle;     /**< Handle of the Helen Project Service (as provided by the BLE stack) */
    ble_gatts_char_handles_t    m_handles;          /**< Handles related to the measurement characteristic */
    ble_gatts_char_handles_t    cp_handles;         /**< Handles related to the control point characteristic */
    ble_gatts_char_handles_t    modes_handles;      /**< Handles related to the Helen Modes characteristic */
    ble_hps_feature_t           features;           /**< the supported features */
    uint16_t                    modes_len;          /**> the length of the modes characteristics */
    ble_hps_evt_handler_t       evt_handler;        /**< Event handler to be called for handling events in the Helen Project Service */
    ble_srv_error_handler_t     error_handler;      /**< Function to be called in case of an error. */
    ble_hps_qwr_buffer_t  *     p_qwr_buf;          /**< pointer to the buffer for queued write operations */
    ble_hps_client_spec_t *     p_client;           /**< client specific data */
};

/** @brief the measurements flags field */
typedef struct
{
    uint8_t output_power_present  : 1;  /**< Flag indication that the output power field is present */
    uint8_t temperature_present   : 1;  /**< Flag indication that the temperature field is present */
    uint8_t input_voltage_present : 1;  /**< Flag indication that the input voltage field is present */
} ble_hps_m_flags_t;

/** @brief the measurement structure */
typedef struct
{
    ble_hps_m_flags_t flags;            /**< Flags indicating which fields are present */
    uint8_t           mode;             /**< current mode */
    uint16_t          output_power;     /**< output power in mW */
    int8_t            temperature;      /**< temperature in °C */
    uint16_t          input_voltage;    /**< input voltage in mW */
} ble_hps_m_t;

/* Exported macros ---------------------------------------------------------- */
#define GLUE(X, Y)  X ## Y
#define BLE_HPS_DEF(_name, _hps_max_clients)                                 \
static ble_hps_client_spec_t GLUE(_name, _client_data)[_hps_max_clients];    \
static ble_hps_t _name =                                                     \
{                                                                            \
    .p_client = GLUE(_name, _client_data),                                   \
};

/* Exported functions ------------------------------------------------------- */
/** @brief Function for initializing the Helen Project Service
 *
 * @param[out]  p_hps       Helen Project Service structure. This structure will
 *                          have to be supplied by the application. It will be
 *                          initialized by this function, and will later be used
 *                          to identify this particular service instance
 * @param[in]   max_clients Number of maximum connected clients
 * @param[in]   p_hps_init  Information needed to initialize the service
 * @return      NRF_SUCCESS on successful initialization of service, otherwise
 *              an error code
 */
uint32_t ble_hps_init(ble_hps_t * p_hps, uint8_t max_clients, const ble_hps_init_t * p_hps_init);

/** @brief Function for handling the Applications's BLE stack events.
 *
 * @param[in]   p_hps       Helen Project Service Structure
 * @param[in]   p_ble_evt   Event received from the BLE stack
 */
void ble_hps_on_ble_evt(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt);

/** @brief Function to send measurement data if notifications have been enabled
 *
 * @param[in] p_hps        the helen service structure
 * @param[in] conn_handle  the connection to send the message to
 * @param[in] p_data       the data to send
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_STATE *
 */
uint32_t ble_hps_measurement_send(ble_hps_t const * p_hps, uint16_t conn_handle, ble_hps_m_t const * p_data);

#endif /* BLE_LCS_H_INCLUDED */

/**END OF FILE*****************************************************************/
