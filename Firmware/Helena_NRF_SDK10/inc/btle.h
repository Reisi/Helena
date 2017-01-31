/**
  ******************************************************************************
  * @file    btle.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/08
  * @brief   header for helenas bluetooth module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BTLE_H_INCLUDED
#define BTLE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "comreloaded.h"

/* Exported defines ----------------------------------------------------------*/
#define BTLE_LIGHT_FLAG_OVERCURRENT     (1<<0)
#define BTLE_LIGHT_FLAG_TEMPERATURE     (1<<1)
#define BTLE_LIGHT_FLAG_INPUTVOLTAGE    (1<<2)
#define BTLE_LIGHT_FLAG_DUTYCYCLELIMIT  (1<<3)

/* Exported types ------------------------------------------------------------*/
/**< btle events */
typedef enum
{
    BTLE_EVT_CONNECTION = 0,    /**< connection related events */
    BTLE_EVT_HID,               /**< hid related events  */
    BTLE_EVT_LCS_CTRL_POINT     /**< events related to the light control service control point */
} btle_EventEnum;

typedef enum
{
    BTLE_EVT_CONN_CENTRAL_SEARCH_START,
    BTLE_EVT_CONN_CENTRAL_SEARCH_STOP,
    BTLE_EVT_CONN_CENTRAL_CONNECTED,
    BTLE_EVT_CONN_CENTRAL_DISCONNECTED,
    BTLE_EVT_CONN_PERIPH_CONNECTED,
    BTLE_EVT_CONN_PERIPH_DISCONNECTED
} btle_ConnectionEventEnum;

typedef enum
{
    BTLE_EVT_HID_VOL_UP_SHORT,
    BTLE_EVT_HID_VOL_UP_LONG,
    BTLE_EVT_HID_VOL_DOWN_SHORT,
    BTLE_EVT_HID_VOL_DOWN_LONG
} btle_HidEventType;

typedef enum
{
    BTLE_EVT_LCSCP_REQ_MODE_CNT,
    BTLE_EVT_LCSCP_REQ_GROUP_CNT,
    BTLE_EVT_LCSCP_REQ_MODE_CONFIG,
    BTLE_EVT_LCSCP_SET_MODE,
    BTLE_EVT_LCSCP_CONFIG_MODE,
    BTLE_EVT_LCSCP_CONFIG_GROUP,
    BTLE_EVT_LCSCP_REQ_LED_CONFIG,
    BTLE_EVT_LCSCP_CHECK_LED_CONFIG,
    BTLE_EVT_LCSCP_REQ_SENS_OFFSET,
    BTLE_EVT_LCSCP_CALIB_SENS_OFFSET
} btle_LcsCtrlPtEventEnum;

/**< available light modes for btle module */
typedef enum
{
    BTLE_LIGHT_MODE_OFF = 0,
    BTLE_LIGHT_MODE_FLOOD,
    BTLE_LIGHT_MODE_SPOT,
    BTLE_LIGHT_MODE_FLOOD_AND_SPOT,
    BTLE_LIGHT_MODE_FLOOD_PITCH_COMPENSATED,
    BTLE_LIGHT_MODE_SPOT_PITCH_COMPENSATED,
    BTLE_LIGHT_MODE_FLOOD_AND_SPOT_PITCH_COMPENSATED,
    BTLE_LIGHT_MODE_FLOOD_CLONED,
    BTLE_LIGHT_MODE_SPOT_CLONED,
    BTLE_LIGHT_MODE_FLOOD_PITCH_COMPENSATED_CLONED,
    BTLE_LIGHT_MODE_SPOT_PITCH_COMPENSATED_CLONED,
} btle_LightModeEnum;

/**< btle light configuration type */
typedef struct
{
    btle_LightModeEnum modeType;
    uint8_t intensity;
} btle_LightModeConfig;

/**< btle event structure */
typedef struct
{
    btle_EventEnum evt;                 /**< event type */
    union
    {
        btle_ConnectionEventEnum conn;
        btle_HidEventType        hid;
        btle_LcsCtrlPtEventEnum  lcscp;
    } subEvt;
    union
    {
        uint8_t modeConfigStart;        /**< parameter for event type BTLE_EVT_REQ_MODE_CONFIG */
        uint8_t modeToSet;              /**< parameter for event type BTLE_EVT_SET_MOPE */
        struct
        {
            uint8_t modeNumber;             // first mode number to change
            uint8_t listEntries;            // number of modes to change
            btle_LightModeConfig * pConfig; // list of mode configurations
        } modeToConfig;                 /**< parameter for event type BTLE_EVT_CONFIG_MODE */
        uint8_t groupConfig;
    } lcscpEventParams;
} btle_EventStruct;

/**< return values for the btle event handler */
typedef enum
{
    BTLE_RET_SUCCESS = 1,
    BTLE_RET_NOT_SUPPORTED = 2,
    BTLE_RET_INVALID = 3,
    BTLE_RET_FAILED = 4
} btle_EventReturnEnum;

typedef struct
{
    btle_LcsCtrlPtEventEnum evt;                 /**< event type */
    btle_EventReturnEnum    retCode;
    union
    {
        uint8_t modeCnt;                /**< response parameter for event type BTLE_EVT_REQ_MODE_CNT */
        uint8_t groupCnt;               /**< response parameter for event type BTLE_EVT_REQ_GROUP_CNT */
        struct
        {
            btle_LightModeConfig * pList;
            uint8_t listEntries;
        } modeList;                     /**< response parameter for event type BTLE_EVT_REQ_MODE_CONFIG */
        struct
        {
            uint8_t floodCnt;
            uint8_t spotCnt;
        } ledConfig;                    /**< response parameter for event type BTLE_EVT_LCSCP_REQ_LED_CONFIG and BTLE_EVT_LCSCP_CHECK_LED_CONFIG */
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
        } sensOffset;                   /**< response parameter for event type BTLE_EVT_LCSCP_REQ_SENS_OFFSET and BTLE_EVT_LCSCP_CALIB_SENS_OFFSET */
    } responseParams;
} btle_LcscpEventResponseStruct;

/**< btle event handler type */
typedef void (*btle_EventHandler)(btle_EventStruct * pEvt);

/**< available light status flags for btle module */
typedef struct
{
    uint8_t overcurrent    : 1;
    uint8_t inputVoltage   : 1;
    uint8_t temperature    : 1;
    uint8_t dutyCycleLimit : 1;
} btle_lightStatusFlags;

/**< light data structure */
typedef struct
{
    btle_LightModeEnum mode;
    uint8_t intensity;       /**< intensity in % or lux */
    btle_lightStatusFlags statusFlood;
    btle_lightStatusFlags statusSpot;
    uint16_t powerFlood;    /**< flood output power in mW */
    uint16_t powerSpot;     /**< spot output power in mW */
    int8_t temperature;     /**< light temperature in °C */
    uint16_t inputVoltage;  /**< input voltage in mV */
    int8_t pitch;           /**< pitch angle in ° */
} btle_lightDataStruct;

typedef struct
{
    uint8_t floodSupported : 1;
    uint8_t spotSupported  : 1;
    uint8_t pitchSupported : 1;
} btle_LightFeatureStruct;

typedef enum
{
    BTLE_SCAN_MODE_LOW_POWER = 0,
    BTLE_SCAN_MODE_LOW_LATENCY
} btle_ScanModeEnum;

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief Function to initialize the ble stack
 */
void btle_StackInit(void);

/** @brief Function to initialize the btle module
 *
 * @param[in]   deleteBonds true to erase all bond information at
 *              initialization
 * @param[in]   pFeature supported features
 * @param[in]   event handler for btle events
 */
void btle_Init(bool deleteBonds, btle_LightFeatureStruct* pFeature, btle_EventHandler pEvtHandler);

/** @brief Function to set the scan mode
 *
 * @param[in] scanMode  new scanMode to use
 */
void btle_SetScanConfig(btle_ScanModeEnum newScanConfig);

/** @brief Function for relaying comreloaded messages over the com gateway
 *         service to connected devices
 *
 * @param[in]   pMessageIn  message to relay
 * @return      NRF_SUCCESS on successful transfer, otherwise an error code
 */
uint32_t btle_ComGatewayCheck(const com_MessageStruct * pMessageIn);

/** @brief Function for updating light information. If a device is connected and
 *         light control service notifications are enabled a notification
 *         message will be sent o with a period of 1Hz
 *
 * @param[in]   pData   light data to be sent
 * @return      NRF_SUCCESS on successful transfer, otherwise an error code
 */
uint32_t btle_UpdateLightMeasurements(const btle_lightDataStruct * pData);

/** @brief Function for sending response parameters for incoming btle events
 *
 * @param[in]   pRsp    Return parameters
 * @return      NRF_SUCCESS o an error code
 */
uint32_t btle_SendEventResponse(const btle_LcscpEventResponseStruct *pRsp);

/** @brief Function for deleting all bonds
 *
 * @return      NRF_SUCCESS or an error code;
 */
uint32_t btle_DeleteBonds(void);

/** @brief Function to start searching for possible remotes
 *
 * @return      NRF_SUCCESS or an error code
 */
uint32_t btle_SearchForRemote(void);

#endif /* BTLE_H_INCLUDED */

/**END OF FILE*****************************************************************/
