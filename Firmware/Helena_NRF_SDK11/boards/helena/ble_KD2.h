/**
  ******************************************************************************
  * @file    ble_KD2.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @brief   KD2 Feature and Control Point characteristic implementation
  *
  * @details This module implements the KD2 Feature and KD2 Control Point
  *          characteristics.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_KD2_H_INCLUDED
#define BLE_KD2_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/* Exported defines ----------------------------------------------------------*/
#ifndef BLE_KD2_BLE_OBSERVER_PRIO
#define BLE_KD2_BLE_OBSERVER_PRIO   2
#endif // BLE_KD2_BLE_OBSERVER_PRIO

#define BLE_KD2_CP_MAX_LEN        15
#define BLE_KD2_CP_MIN_LEN        1

#define BLE_KD2_MAX_CHANNELS      5

/* Exported types ------------------------------------------------------------*/
/**@brief Light Control Service Event Types */
typedef enum
{
    BLE_KD2_EVT_CP_INDICATION_ENABLED,    /**< KD2 Control Point Indication enabled event */
    BLE_KD2_EVT_CP_INDICATION_DISABLED,   /**< KD2 Control Point Indication disabled event */
    BLE_KD2_EVT_CP_EVT,                   /**< write event form the KD2 Control Point */
} ble_KD2_evt_type_t;

typedef enum
{
    BLE_KD2_CP_OP_REQ_CHN_CONFIG      = 1, /**< Operator to request a channel configuration */
    BLE_KD2_CP_OP_SET_CHN_CONFIG      = 2, /**< Operator to set a channel configuration */
    BLE_KD2_CP_OP_REQ_COM_PIN_CONFIG  = 3, /**< Operator to request the com pin configuration */
    BLE_KD2_CP_OP_SET_COM_PIN_CONFIG  = 4, /**< Operator to set the com pin configuration */
    BLE_KD2_CP_OP_REQ_INT_COMP        = 5, /**< Operator to request the internal compensation */
    BLE_KD2_CP_OP_SET_INT_COMP        = 6, /**< Operator to set the internal compensation */
    BLE_KD2_CP_OP_REQ_EXT_COMP        = 7, /**< Operator to request the external comepnsation */
    BLE_KD2_CP_OP_SET_EXT_COMP        = 8, /**< Operator to set the external compensation */
    BLE_KD2_CP_OP_REQ_IMU_CALIB_STATE = 9, /**< Operator to request the imu compensation state */
    BLE_KD2_CP_OP_START_IMU_CALIB     = 10, /**< Operator to set the external compensation */
    BLE_KD2_CP_OP_RSP_CODE            = 32,/**< Response Code */
} ble_KD2_cp_op_code_t;

/**@brief KD2 Control Control Point response parameter */
typedef enum
{
    BLE_KD2_CP_RSP_VAL_SUCCESS       = 1,
    BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED = 2,
    BLE_KD2_CP_RSP_VAL_INVALID       = 3,
    BLE_KD2_CP_RSP_VAL_FAILED        = 4
} ble_KD2_cp_rsp_val_t;

/**@brief KD2 Control Control Point procedure status */
typedef enum
{
    BLE_KD2_CP_PROC_STATUS_FREE             = 0,
    BLE_KD2_CP_PROC_IN_PROGRESS             = 1,
    BLE_KD2_CP_RSP_CODE_IND_PENDING         = 2,
    BLE_KD2_CP_RSP_CODE_IND_CONFIRM_PENDING = 3
} ble_KD2_cp_proc_status_t;

/**@brief KD2 Control Control Point response indication structure */
typedef struct
{
    ble_KD2_cp_rsp_val_t   rsp_code;
    uint8_t                len;
    uint8_t                encoded_rsp[BLE_KD2_CP_MAX_LEN];
} ble_KD2_cp_rsp_ind_t;

typedef enum
{
    BLE_KD2_OPTIC_NA  = 0,
    BLE_KD2_OPTIC_15D = 1,
    BLE_KD2_OPTIC_22D = 2,
    BLE_KD2_OPTIC_30D = 3,
} ble_KD2_optic_type_t;

typedef struct
{
    uint16_t             output_power;  /**< output power in Watt, resolution 1/1000 */
    uint8_t              output_limit;  /**< output limit in % */
    ble_KD2_optic_type_t optic_type;    /**< installed optic */
    int16_t              optic_offset;  /**< optic offset angle in °, resolution 1/100 */
} ble_KD2_channel_config_t;

/**@brief KD2 com pin modes */
typedef enum
{
    BLE_KD2_COM_PIN_NOT_USED = 0,
    BLE_KD2_COM_PIN_COM      = 1,
    BLE_KD2_COM_PIN_BUTTON   = 2,
    BLE_KD2_COM_PIN_PWM      = 3,
} ble_KD2_com_pin_t;

/**@brief KD2 internal compensation structure */
typedef struct
{
    uint16_t voltage_gain;
    int16_t  voltage_offset;
    uint16_t current_gain;
    int16_t  current_offset;
    uint16_t temperature_gain;
    int16_t  temperature_offset;
} ble_KD2_int_comp_t;

/**@brief KD2 external compensation structure */
typedef struct
{
    int16_t temperature_offset;
    uint8_t left_current_gain;
    uint8_t right_current_gain;
} ble_KD2_ext_comp_t;

/**@brief KD2 Control Point event parameters */
typedef union
{
    struct
    {
        uint8_t                  channel;
        ble_KD2_channel_config_t config;
    }                            channel_config;
    ble_KD2_com_pin_t            com_pin;
    ble_KD2_int_comp_t           int_comp;
    ble_KD2_ext_comp_t           ext_comp;
    bool                         imu_calib_state;
} ble_KD2_cp_params_t;

/**@brief client specific data */
typedef struct
{
    uint16_t                 conn_handle;             /**< Handle if the current connection (as provided by the BLE stack) */
    bool                     is_KD2cp_indic_enabled;  /**< Indicator if light control point indications are enabled */
    ble_KD2_cp_proc_status_t procedure_status;        /**< status of possible procedure */
    ble_KD2_cp_rsp_ind_t     response;                /**< pending response data */
} ble_KD2_client_spec_t;

// Forward declaration of the ble_KD2_t type.
typedef struct ble_KD2_s ble_KD2_t;

/**@brief KD2 Control Service Event */
typedef struct
{
    ble_KD2_evt_type_t            evt_type; /**< the type of the event */
    ble_KD2_t const             * p_KD2;    /**< a pointer to the instance */
    ble_KD2_client_spec_t const * p_client; /**< a pointer to the client data */
    ble_KD2_cp_op_code_t          cp_evt;   /**< the control point event in case of @ref BLE_KD2_EVT_CP_EVT */
    ble_KD2_cp_params_t const   * p_params; /**< the event parameters */
} ble_KD2_evt_t;

/**@brief KD2 Control Service event handler type */
typedef void (*ble_KD2_evt_handler_t)(ble_KD2_evt_t const * p_evt);

/**@brief KD2 configuration feature structure. This contains the supported configuration features */
typedef struct
{
    uint8_t channel_config_supported : 1;
    uint8_t com_pin_mode_supported   : 1;
    uint8_t internal_comp_supported  : 1;
    uint8_t external_comp_supported  : 1;
    uint8_t imu_calib_supported      : 1;
} ble_KD2_config_f_t;

/**@brief KD2 channel feature structure. This contains the supported channel features */
typedef struct
{
    uint8_t adaptive_supported : 1;
} ble_KD2_channel_f_t;

/**qbrief KD2 feature structure. */
typedef struct
{
    ble_KD2_config_f_t  config;
    ble_KD2_channel_f_t channel;
} ble_KD2_f_t;

/**@brief KD2 Control Service init structure. This contains all options and data
 *        needed for initialization of the service */
typedef struct
{
    security_req_t          KD2_f_rd_sec;       /**< Security requirement for reading KD2 feature characteristic. */
    security_req_t          KD2_cp_cccd_wr_sec; /**< Security requirement for writing KD2 control point characteristic CCCD. */
    security_req_t          KD2_cp_wr_sec;      /**< Security requirement for writing KD2 control point characteristic. */
    ble_KD2_evt_handler_t   evt_handler;        /**< Event handler to be called for handling events in the KD2 Control Service */
    ble_srv_error_handler_t error_handler;      /**< Function to be called in case of an error. */
    ble_KD2_f_t             feature;            /**< supported features */
} ble_KD2_init_t;

/**@brief KD2 Control Service structure. This contains various status
 *        information for the service */
struct ble_KD2_s
{
    uint8_t                  uuid_type;
    uint16_t                 service_handle;/**< Handle of the Helen Project Service (as provided by the BLE stack) */
    ble_gatts_char_handles_t f_handles;     /**< Handles related to the feature characteristic */
    ble_gatts_char_handles_t cp_handles;    /**< Handles related to the control point characteristic */
    ble_KD2_evt_handler_t    evt_handler;   /**< Event handler to be called for handling events in the KD2 Control Service */
    ble_srv_error_handler_t  error_handler; /**< Function to be called in case of an error. */
    ble_KD2_f_t              feature;       /**< supported features */
    ble_KD2_client_spec_t  * p_client;      /**< client specific data */
};

/**@brief KD2 Control Control Point Response parameter structure */
typedef struct
{
    ble_KD2_cp_op_code_t    opcode;
    ble_KD2_cp_rsp_val_t    status;
    ble_KD2_cp_params_t     params;
} ble_KD2_cp_rsp_t;

/* Exported macros ---------------------------------------------------------- */
#define GLUE(X, Y)  X ## Y
#define BLE_KD2_DEF(_name, _KD2_max_clients)                              \
static ble_KD2_client_spec_t GLUE(_name, _client_data)[_KD2_max_clients]; \
static ble_KD2_t _name =                                                  \
{                                                                         \
    .p_client = GLUE(_name, _client_data),                                \
};

/* Exported functions ------------------------------------------------------- */
/** @brief Function for initializing the KD2 Control Service
 *
 * @param[out]  p_KD2         KD2 Control Service structure. This structure
 *                            will have to be supplied by the application. It
 *                            will be initialized by this function, and will
 *                            later be usedto identify this particular service
 *                            instance
 * @param[in]   p_KD2_init  Information needed to initialize the service
 * @return      NRF_SUCCESS on successful initialization of service, otherwise
 *              an error code *
 */
uint32_t ble_KD2_init(ble_KD2_t * p_KD2, uint8_t max_clients, ble_KD2_init_t const * p_KD2_init);

/** @brief Function for handling the Applications's BLE stack events.
 *
 * @param[in]   p_context   context
 * @param[in]   p_ble_evt   Event received from the BLE stack
 */
void ble_KD2_on_ble_evt(void * p_context, ble_evt_t * p_ble_evt);

//uint32_t ble_KD2_brd_char_add(uint8_t uuid_type, uint16_t service_handle, void * p_context);

/** @brief Function for sending a control point response
 *
 * @param[in] p_KD2      KD2 Control Service structure
 * @param[in] conn_handle  the connection handle to send the response to
 * @param[in] p_rsp        Response data structure
 * @return    NRF_SUCCESS on successful response, otherwise an error code
 */
uint32_t ble_KD2_cp_response(ble_KD2_t * p_KD2, uint16_t conn_handle, ble_KD2_cp_rsp_t const * p_rsp);

#endif /* BLE_KD2S_H_INCLUDED */

/**END OF FILE*****************************************************************/
