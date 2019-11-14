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
#include "ble.h"

/* Exported defines ----------------------------------------------------------*/
#define BTLE_CONN_HANDLE_INVALID    BLE_CONN_HANDLE_INVALID
#define BTLE_CONN_HANDLE_ALL        BLE_CONN_HANDLE_ALL

/*#define BTLE_LIGHT_FLAG_OVERCURRENT     (1<<0)
#define BTLE_LIGHT_FLAG_TEMPERATURE     (1<<1)
#define BTLE_LIGHT_FLAG_INPUTVOLTAGE    (1<<2)
#define BTLE_LIGHT_FLAG_DUTYCYCLELIMIT  (1<<3)*/

/* Exported types ------------------------------------------------------------*/
/**< btle events */
typedef enum
{
    BTLE_EVT_CONNECTION = 0,    /**< connection related events */
    BTLE_EVT_HID,               /**< hid related events  */
    BTLE_EVT_LCS_CTRL_POINT     /**< events related to the light control service control point */
} btle_eventTypes_t;

typedef enum
{
    BTLE_EVT_CONN_CENTRAL_SEARCH_START,
    BTLE_EVT_CONN_CENTRAL_SEARCH_STOP,
    BTLE_EVT_CONN_CENTRAL_CONNECTED,
    BTLE_EVT_CONN_CENTRAL_DISCONNECTED,
    BTLE_EVT_CONN_PERIPH_CONNECTED,
    BTLE_EVT_CONN_PERIPH_DISCONNECTED
} btle_connectionEventType_t;

typedef enum
{
    BTLE_EVT_HID_VOL_UP_SHORT,
    BTLE_EVT_HID_VOL_UP_LONG,
    BTLE_EVT_HID_VOL_DOWN_SHORT,
    BTLE_EVT_HID_VOL_DOWN_LONG
} btle_hidEventType_t;

typedef enum
{
    BTLE_EVT_LCSCP_REQ_MODE_CNT,
    BTLE_EVT_LCSCP_SET_MODE,
    BTLE_EVT_LCSCP_REQ_GROUP_CNT,
    BTLE_EVT_LCSCP_CONFIG_GROUP,
    BTLE_EVT_LCSCP_REQ_MODE_CONFIG,
    BTLE_EVT_LCSCP_CONFIG_MODE,
    BTLE_EVT_LCSCP_REQ_LED_CONFIG,
    BTLE_EVT_LCSCP_CHECK_LED_CONFIG,
    BTLE_EVT_LCSCP_REQ_SENS_OFFSET,
    BTLE_EVT_LCSCP_CALIB_SENS_OFFSET,
    BTLE_EVT_LCSCP_REQ_LIMITS,
    BTLE_EVT_LCSCP_SET_LIMITS
} btle_LcsCtrlPtEventTypes_t;

/**< available light setup for light control service */
typedef struct
{
    bool flood             : 1;        // flood active
    bool spot              : 1;        // spot active
    bool pitchCompensation : 1;        // pitch compensation enabled
    bool cloned            : 1;        // output cloned to both drivers
    bool taillight         : 1;        // external taillight enabled
    bool brakelight        : 1;        // external brake light enabled
} btle_LcsLightSetup_t;

/**< btle light configuration type */
typedef struct
{
    btle_LcsLightSetup_t setup;
    union
    {
        uint8_t intensityInPercent;
        uint8_t illuminanceInLux;
    };
} btle_LcsModeConfig_t;

/**< btle event structure */
typedef struct
{
    btle_eventTypes_t evt;                 /**< event type */
    union
    {
        btle_connectionEventType_t conn;
        btle_hidEventType_t        hid;
        btle_LcsCtrlPtEventTypes_t lcscp;
    } subEvt;
    uint16_t connHandle;                /**< the connection handle this event is related to, use the same to send response */
    union
    {
        uint8_t modeConfigStart;        /**< parameter for event type BTLE_EVT_REQ_MODE_CONFIG */
        uint8_t modeToSet;              /**< parameter for event type BTLE_EVT_SET_MOPE */
        struct
        {
            uint8_t modeNumber;             // first mode number to change
            uint8_t listEntries;            // number of modes to change
            btle_LcsModeConfig_t const* pConfig; // list of mode configurations
        } modeToConfig;                 /**< parameter for event type BTLE_EVT_CONFIG_MODE */
        uint8_t groupConfig;            /**< parameter for event type BTLE_EVT_LCSCP_CONFIG_GROUP */
        struct
        {
            int8_t floodInPercent;
            int8_t spotInPercent;
        } currentLimits;                /**< parameter for event type BTLE_EVT_LCSCP_SET_LIMITS */
    } lcscpEventParams;
} btle_event_t;

/**< return values for the btle event handler */
typedef enum
{
    BTLE_RET_SUCCESS = 1,
    BTLE_RET_NOT_SUPPORTED = 2,
    BTLE_RET_INVALID = 3,
    BTLE_RET_FAILED = 4
} btle_eventReturn_t;

typedef struct
{
    btle_LcsCtrlPtEventTypes_t evt;                 /**< event type */
    btle_eventReturn_t    retCode;
    union
    {
        uint8_t modeCnt;                /**< response parameter for event type BTLE_EVT_REQ_MODE_CNT */
        uint8_t groupCnt;               /**< response parameter for event type BTLE_EVT_REQ_GROUP_CNT */
        struct
        {
            btle_LcsModeConfig_t const* pList;
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
        struct
        {
            int8_t floodInPercent;
            int8_t spotInPercent;
        } currentLimits;                /**< response parameter for event type BTLE_EVT_LCSCP_REQ_LIMITS */
    } responseParams;
} btle_LcscpEventResponse_t;

/**< btle event handler type */
typedef void (*btle_eventHandler_t)(btle_event_t * pEvt);

/**< available light status flags for btle module */
typedef struct
{
    bool overcurrent    : 1;
    bool inputVoltage   : 1;
    bool temperature    : 1;
    bool dutyCycleLimit : 1;
} btle_lcsStatusFlags_t;

/**< light data structure */
typedef struct
{
    btle_LcsModeConfig_t mode;
    btle_lcsStatusFlags_t statusFlood;
    btle_lcsStatusFlags_t statusSpot;
    uint16_t powerFlood;    /**< flood output power in mW */
    uint16_t powerSpot;     /**< spot output power in mW */
    int8_t temperature;     /**< light temperature in °C */
    uint16_t inputVoltage;  /**< input voltage in mV */
    int8_t pitch;           /**< pitch angle in ° */
} btle_lcsMeasurement_t;

typedef struct
{
    uint8_t floodSupported : 1;
    uint8_t spotSupported  : 1;
    uint8_t pitchSupported : 1;
} btle_lcsFeature_t;

typedef enum
{
    BTLE_SCAN_MODE_LOW_POWER = 0,
    BTLE_SCAN_MODE_LOW_LATENCY
} btle_scanMode_t;

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
void btle_Init(bool deleteBonds, btle_lcsFeature_t* pFeature, btle_eventHandler_t pEvtHandler);

/** @brief Function to set the scan mode
 *
 * @param[in] scanMode  new scanMode to use
 */
void btle_SetScanConfig(btle_scanMode_t newScanConfig);

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
uint32_t btle_UpdateLcsMeasurements(const btle_lcsMeasurement_t * pData);

/** @brief Function for sending response parameters for incoming btle events
 *
 * @param[in]   pRsp       Return parameters
 * @param[in]   connHandle the connection handle this response is related to
 * @return      NRF_SUCCESS o an error code
 */
uint32_t btle_SendEventResponse(const btle_LcscpEventResponse_t *pRsp, uint16_t connHandle);

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

/** @brief Function set the mode of a remote Light Control Device
 *
 * @param[in]   mode mode to set the device to
 * @param[in]   connection handle to send command to, BTLE_CONN_HANDLE_ALL for all connected devices
 * @return      NRF_SUCCESS
 *              NRF_ERROR_INVALID_STATE if no device connected
 */
uint32_t btle_SetMode(uint8_t mode, uint16_t connHandle);

#endif /* BTLE_H_INCLUDED */

/**END OF FILE*****************************************************************/
