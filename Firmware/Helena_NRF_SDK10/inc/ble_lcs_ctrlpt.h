/**
  ******************************************************************************
  * @file    ble_lcs_ctrlpt.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/08
  * @brief   Light Control Service control point implementation
  *
  * @details This module implements the Light Control Service control behavior.
  *          It is used by the Control Point Service module for control point
  *          mechanisms like configuring and setting light modes and grouping.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_LCS_CTRLPT_H_INCLUDED
#define BLE_LCS_CTRLPT_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "ble_lcs.h"

/* Exported defines ----------------------------------------------------------*/
#define BLE_LCS_CTRLPT_MAX_LEN                  19
#define BLE_LCS_CTRLPT_MIN_LEN                  1

#define BLE_LCS_CTRLPT_MIN_RESPONSE_SIZE        3
#define BLE_LCS_CTRLPT_MAX_RESPONSE_SIZE        19

#define BLE_LCS_CTRLPT_OPCODE_POS               0
#define BLE_LCS_CTRLPT_PARAMETER_POS            1
#define BLE_LCS_CTRLPT_REQUEST_OPCODE_POS       1
#define BLE_LCS_CTRLPT_RESPONSE_CODE_POS        2
#define BLE_LCS_CTRLPT_RESPONSE_PARAMETER_POS   3

#define BLE_LCS_CTRLPT_MAX_NUM_OF_MODES         8

/* Exported types ------------------------------------------------------------*/
/**@brief Light Control Service Control Point event types */
typedef enum
{
    BLE_LCS_CTRLPT_EVT_REQ_MODE_CNT,    /**< Operator to request mode count */
    BLE_LCS_CTRLPT_EVT_SET_MODE,        /**< Operator to set a specific mode */
    BLE_LCS_CTRLPT_EVT_REQ_GRP_CNFG,    /**< Operator to request number of mode groups */
    BLE_LCS_CTRLPT_EVT_CNFG_GROUP,      /**< Operator to configure grouping */
    BLE_LCS_CTRLPT_EVT_REQ_MODE_CNFG,   /**< Operator to request a list of mode configurations */
    BLE_LCS_CTRLPT_EVT_CNFG_MODE,       /**< Operator to configure a specific mode */
    BLE_LCS_CTRLPT_EVT_REQ_LED_CNFG,    /**< Operator to request current led configuration */
    BLE_LCS_CTRLPT_EVT_CHK_LED_CNFG,    /**< Operator to start the check procedure for led configuration */
    BLE_LCS_CTRLPT_EVT_REQ_SENS_OFF,    /**< Operator to request the current sensor offset values */
    BLE_LCS_CTRLPT_EVT_CALIB_SENS_OFF,  /**< Operator to start the sensor offset calibration */
    BLE_LCS_CTRLPT_EVT_REQ_LIMITS,      /**< Operator to request the current current limits */
    BLE_LCS_CTRLPT_EVT_SET_LIMITS,      /**< Operator to set the current limits  */
} ble_lcs_ctrlpt_evt_type_t;

/**@brief Light Control Control Point mode configuration structure */
typedef struct
{
    uint8_t              mode_number_start;
    uint8_t              mode_entries;
    ble_lcs_light_mode_t config[BLE_LCS_CTRLPT_MAX_NUM_OF_MODES];
} ble_lcs_ctrlpt_mode_cnfg_t;

/**@brief Light Control Control Point set current limits structure */
typedef struct
{
    int8_t  flood;
    int8_t  spot;
} ble_lcs_ctrlpt_set_limits_t;

/**@brief Light Control Service Control Point write values */
typedef union
{
    uint8_t                     mode_list_start;
    uint8_t                     set_mode;
    ble_lcs_ctrlpt_mode_cnfg_t  mode_config;
    uint8_t                     group_config;
    ble_lcs_ctrlpt_set_limits_t current_limits;
} ble_lcs_ctrlpt_write_val_t;

/**@brief Light Control Service Control Point event */
typedef struct
{
    ble_lcs_ctrlpt_evt_type_t    evt_type;
    ble_lcs_ctrlpt_write_val_t * p_params;
} ble_lcs_ctrlpt_evt_t;

/**@brief Light Control Control Point operator codes */
typedef enum
{
    BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNT     = 1,
    BLE_LCS_CTRLPT_OP_CODE_SET_MODE         = 2,
    BLE_LCS_CTRLPT_OP_CODE_REQ_GRP_CNFG     = 3,
    BLE_LCS_CTRLPT_OP_CODE_CNFG_GROUP       = 4,
    BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNFG    = 5,
    BLE_LCS_CTRLPT_OP_CODE_CNFG_MODE        = 6,
    BLE_LCS_CTRLPT_OP_CODE_REQ_LED_CNFG     = 7,
    BLE_LCS_CTRLPT_OP_CODE_CHK_LED_CNFG     = 8,
    BLE_LCS_CTRLPT_OP_CODE_REQ_SENS_OFF     = 9,
    BLE_LCS_CTRLPT_OP_CODE_CALIB_SENS_OFF   = 10,
    BLE_LCS_CTRLPT_OP_CODE_REQ_LIMITS       = 11,
    BLE_LCS_CTRLPT_OP_CODE_SET_LIMITS       = 12,
    BLE_LCS_CTRLPT_OP_CODE_RESPONSE         = 32
} ble_lcs_ctrlpt_op_code_t;

/**@brief Light Control Control Point response parameter */
typedef enum
{
    BLE_LCS_CTRLPT_RSP_CODE_SUCCESS       = 1,
    BLE_LCS_CTRLPT_RSP_CODE_NOT_SUPPORTED = 2,
    BLE_LCS_CTRLPT_RSP_CODE_INVALID       = 3,
    BLE_LCS_CTRLPT_RSP_CODE_FAILED        = 4
} ble_lcs_ctrlpt_rsp_code_t;

/**@brief Light Control Control Point procedure status */
typedef enum
{
    BLE_LCS_CTRLPT_PROC_STATUS_FREE             = 0,
    BLE_LCS_CTRLPT_PROC_IN_PROGRESS             = 1,
    BLE_LCS_CTRLPT_RSP_CODE_IND_PENDING         = 2,
    BLE_LCS_CTRLPT_RSP_CODE_IND_CONFIRM_PENDING = 3
} ble_lcs_ctrlpt_proc_status_t;

/**@brief Light Control Control Point Response parameter for BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNFG */
typedef struct
{
    ble_lcs_light_mode_t * p_list;
    uint8_t                num_of_entries;
} ble_lcs_ctrlpt_rsp_param_mode_config_t;

/**@brief Light Control Control Point Response parameter for BLE_LCS_CTRLPT_OP_CODE_REQ_LED_CNFG and BLE_LCS_CTRLPT_OP_CODE_CHK_LED_CNFG */
typedef struct
{
    uint8_t cnt_flood;
    uint8_t cnt_spot;
} ble_lcs_ctrlpt_rsp_param_led_config_t;

/**@brief Light Control Control Point Response parameter for BLE_LCS_CTRLPT_OP_CODE_REQ_SENS_OFF and BLE_LCS_CTRLPT_OP_CODE_CALIB_SENS_OFF */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ble_lcs_ctrlpt_rsp_param_sens_offset_t;

/**@brief Light Control Control Point Response parameter for  BLE_LCS_CTRLPT_OP_CODE_REQ_LIMITS */
typedef struct
{
    int8_t flood;
    int8_t spot;
} ble_lcs_ctrlpt_rsp_param_limits_t;

/**@brief Light Control Control Point response parameter values */
typedef union
{
    uint8_t                                mode_cnt;
    uint8_t                                group_config;
    ble_lcs_ctrlpt_rsp_param_mode_config_t mode_config_list;
    ble_lcs_ctrlpt_rsp_param_led_config_t  led_config;
    ble_lcs_ctrlpt_rsp_param_sens_offset_t sens_offset;
    ble_lcs_ctrlpt_rsp_param_limits_t      current_limits;
} ble_lcs_ctrlpt_rsp_params_t;

/**@brief Light Control Control Point Response parameter structure */
typedef struct
{
    ble_lcs_ctrlpt_op_code_t    opcode;
    ble_lcs_ctrlpt_rsp_code_t   status;
    ble_lcs_ctrlpt_rsp_params_t params;
} ble_lcs_ctrlpt_rsp_t;

// Forward definition of the ble_lcs_ctrlpt_t type
typedef struct ble_lcs_ctrlpt_s ble_lcs_ctrlpt_t;

/**@brief Light Control Service Control Point event handler type */
typedef void (*ble_lcs_ctrlpt_evt_handler_t)(ble_lcs_ctrlpt_t * p_lcs_ctrlpt,
                                                                  ble_lcs_ctrlpt_evt_t * p_lcs_ctrlpt_evt);

/**@brief Light Control Control Point initialization structure. This contains
 *        all options and data needed for initialization of the Light Control
 *        Control Point module */
typedef struct
{
    uint8_t                      uuid_type;         /**< UUID type for Light Control Service Base UUID. */
    ble_srv_cccd_security_mode_t lc_ctrlpt_attr_md; /**< Initial security level for cycling speed and cadence control point attribute */
    ble_lcs_lf_t                 supported_features;/**< Supported features */
    uint16_t                     service_handle;    /**< Handle of the parent service (as provided by the BLE stack) */
    ble_lcs_ctrlpt_evt_handler_t evt_handler;       /**< event handler */
    ble_srv_error_handler_t      error_handler;     /**< Function to be called in case of an error */
} ble_lcs_ctrlpt_init_t;

/**@brief Light Control Control Point response indication structure */
typedef struct
{
    ble_lcs_ctrlpt_rsp_code_t rsp_code;
    uint8_t                   len;
    uint8_t                   encoded_rsp[BLE_LCS_CTRLPT_MAX_LEN];
} ble_lcs_ctrlpt_rsp_ind_t;

/**@brief Light Control Control Point structure. This contains various status
 *        information for the Light Control Control Point behavior. */
struct ble_lcs_ctrlpt_s
{
    uint8_t                      uuid_type;         /**< UUID type for Light Control Service Base UUID. */
    ble_lcs_lf_t                 supported_features;/**< supported control point features */
    uint16_t                     service_handle;    /**< Handle of the parent service (as provided by the BLE stack) */
    ble_gatts_char_handles_t     lc_ctrlpt_handles; /**< Handles related to the Light Control Control Point characteristic */
    uint16_t                     conn_handle;       /**< Handle of the current connection (as provided by the BLE stack) */
    ble_lcs_ctrlpt_evt_handler_t evt_handler;       /**< event handler */
    ble_lcs_ctrlpt_proc_status_t procedure_status;  /**< status of possible procedure */
    ble_srv_error_handler_t      error_handler;     /**< Function to be called in case of an error */
    ble_lcs_ctrlpt_rsp_ind_t     response;          /**< pending response data */
};

/* Exported functions ------------------------------------------------------- */
/** @brief Function for initializing the Light Control Control Point
 *
 * @param[in]   p_lcs_ctrlpt_t      Light Control Control Point structure
 * @param[in]   p_lcs_ctrlpt_init   Information needed to initialize the Control Point behavior
 * @return      NRF_SUCCESS on successful initialization, otherwise an error code
 */
uint32_t ble_lcs_ctrlpt_init(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, ble_lcs_ctrlpt_init_t * p_lcs_ctrlpt_init);

/** @brief Light Control Control Point BLE stack event handler
 *
 * @param[in]   p_lcs_ctrlpt_t  Light Control Control Point structure
 * @param[in]   p_ble_evt       Event received from the BLE stack
 */
void ble_lcs_ctrlpt_on_ble_evt(ble_lcs_ctrlpt_t * p_lcs_ctrlpt_t, ble_evt_t * p_ble_evt);

/** @brief Function for sending mode count response
 *
 * @details Function for sending a control point response when the control point
 *          received was BLE_LCS_CTRLPT_EVT_REQ_MODE_CNT
 *
 * @param[in]   p_lcs_ctrlpt_t  Light Control Control Point structure
 * @param[in]   p_rsps          Response data structure
 * @return      NRF_SUCCESS on successful response, otherwise an error code
 */
uint32_t ble_lcs_ctrlpt_mode_resp(ble_lcs_ctrlpt_t * p_lcs_ctrlpt_t, const ble_lcs_ctrlpt_rsp_t * p_rsps);

#endif /* BLE_LCS_CTRLPT_H_INCLUDED */

/**END OF FILE*****************************************************************/
