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

/* Exported types ------------------------------------------------------------*/
/**< btle events */
typedef enum
{
    BTLE_EVT_CONNECTION = 0,                    // connection related events
    BTLE_EVT_HID,                               // hid related events
    BTLE_EVT_LCS,                               // events related to the light control service
    BTLE_EVT_LCS_CTRL_POINT                     // events related to the light control service control point
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
    //BTLE_EVT_LCSM_ENABLED,
    //BTLE_EVT_LCSM_DISABLED,
    BTLE_EVT_LCS_MEAS_RECEIVED
} btle_LcsMeasEventTypes_t;

typedef enum
{
    BTLE_EVT_LCSCP_REQ_MODE_CNT,                // request mode count
    BTLE_EVT_LCSCP_SET_MODE,                    // set mode
    BTLE_EVT_LCSCP_REQ_GROUP_CNT,               // request group count
    BTLE_EVT_LCSCP_CONFIG_GROUP,                // configure group count
    BTLE_EVT_LCSCP_REQ_MODE_CONFIG,             // request mode configuration
    BTLE_EVT_LCSCP_CONFIG_MODE,                 // configure modes
    BTLE_EVT_LCSCP_REQ_LED_CONFIG,              // request led configuration
    BTLE_EVT_LCSCP_CHECK_LED_CONFIG,            // start procedure to check led configuration
    BTLE_EVT_LCSCP_REQ_SENS_OFFSET,             // request sensor offset
    BTLE_EVT_LCSCP_CALIB_SENS_OFFSET,           // start sensor offset calibration
    BTLE_EVT_LCSCP_REQ_LIMITS,                  // request current limits
    BTLE_EVT_LCSCP_SET_LIMITS,                  // set current limits
    BTLE_EVT_LCSCP_REQ_PREF_MODE,               // request preferred mode
    BTLE_EVT_LCSCP_SET_PREF_MODE                // set preferred mode
} btle_LcsCtrlPtEventTypes_t;

/**< available light setup for light control service */
typedef struct
{
    bool flood             : 1;                 // flood active
    bool spot              : 1;                 // spot active
    bool pitchCompensation : 1;                 // pitch compensation enabled
    bool cloned            : 1;                 // output cloned to both drivers
    bool taillight         : 1;                 // external taillight enabled
    bool brakelight        : 1;                 // external brake light enabled
} btle_LcsLightSetup_t;

/**< btle light configuration type */
typedef struct
{
    btle_LcsLightSetup_t setup;
    union
    {
        uint8_t intensityInPercent;             // used for setups without pitch compensation
        uint8_t illuminanceInLux;               // used for setups with pitch compensation
    };
} btle_LcsModeConfig_t;

/**< available light status flags for btle module */
typedef struct
{
    bool overcurrent    : 1;                    // flag set if requested current exceeds current limit
    bool inputVoltage   : 1;                    // flag set if requested current exceeds voltage limiter value
    bool temperature    : 1;                    // flag set if requested current exceeds temperature limiter value
    bool dutyCycleLimit : 1;                    // flag set if either min. or max. duty-cycle is reached
} btle_lcsStatusFlags_t;

/**< light data structure */
typedef struct
{
    btle_LcsModeConfig_t mode;                  // mode configuration, setup will always be included, intensity just if spot and/or flood are/is enabled
    btle_lcsStatusFlags_t statusFlood;          // status of flood, will be included if flood is enabled
    btle_lcsStatusFlags_t statusSpot;           // status of spot, will be included if spot is enabled
    uint16_t powerFlood;                        // flood output power in mW, will be included if flood is active
    uint16_t powerSpot;                         // spot output power in mW, will be included if spot is active
    int8_t temperature;                         // light temperature in °C, will be included if value is in range -40..85
    uint16_t inputVoltage;                      // input voltage in mV, will be included if value > 0
    int8_t pitch;                               // pitch angle in °, will be included if value is in range -90..+90
} btle_lcsMeasurement_t;

typedef union
{
    uint8_t modeConfigStart;                    // parameter for event type BTLE_EVT_REQ_MODE_CONFIG
    uint8_t modeToSet;                          // parameter for event type BTLE_EVT_SET_MOPE
    struct
    {
        uint8_t modeNumber;                     // first mode number to change
        uint8_t listEntries;                    // number of modes to change
        btle_LcsModeConfig_t const* pConfig;    // list of mode configurations
    } modeToConfig;                             // parameter for event type BTLE_EVT_CONFIG_MODE
    uint8_t groupConfig;                        // parameter for event type BTLE_EVT_LCSCP_CONFIG_GROUP
    struct
    {
        int8_t floodInPercent;                  // flood current limit in % (related to hardware limit)
        int8_t spotInPercent;                   // spot current limit in % (related to hardware limit)
    } currentLimits;                            // parameter for event type BTLE_EVT_LCSCP_SET_LIMITS
            uint8_t prefMode;                   // parameter for event type BTLE_EVT_LCSCP_SET_PREF_MODE
} btle_lcsCtrlPtEvent_t;

/**< btle event structure */
typedef struct
{
    btle_eventTypes_t evt;                      // btle event type
    union
    {
        btle_connectionEventType_t conn;        // sub event type for BTLE_EVT_CONNECTION events
        btle_hidEventType_t        hid;         // sub event type for BTLE_EVT_HID events
        btle_LcsMeasEventTypes_t   lcs;         // sub event type for BTLE_EVT_LCS events
        btle_LcsCtrlPtEventTypes_t lcscp;       // sub event type for BTLE_EVT_LCS_CTRL_POINT events
    } subEvt;
    uint16_t connHandle;                        // the connection handle this event is related to
    union
    {
        btle_lcsMeasurement_t lcsmEventParams;  // event parameters for BTLE_EVT_LCS_MEAS events
        btle_lcsCtrlPtEvent_t lcscpEventParams; // event parameters for BTLE_EVT_LCS_CTRL_POINT events
    };
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
    btle_LcsCtrlPtEventTypes_t evt;             // event type this response is related to
    btle_eventReturn_t    retCode;              // the return code for the event
    union
    {
        uint8_t modeCnt;                        // response parameter for event type BTLE_EVT_REQ_MODE_CNT
        uint8_t groupCnt;                       // response parameter for event type BTLE_EVT_REQ_GROUP_CNT
        struct
        {
            btle_LcsModeConfig_t const* pList;  // list of mode configurations
            uint8_t listEntries;                // number of modes in this list
        } modeList;                             // response parameter for event type BTLE_EVT_REQ_MODE_CONFIG
        struct
        {
            uint8_t floodCnt;                   // number of leds in series connection connected to flood driver
            uint8_t spotCnt;                    // number of leds in series connection connected to spot driver
        } ledConfig;                            // response parameter for event type BTLE_EVT_LCSCP_REQ_LED_CONFIG and BTLE_EVT_LCSCP_CHECK_LED_CONFIG
        struct
        {
            int16_t x;                          // acceleration sensor offset in x-axis, resolution tbd
            int16_t y;                          // acceleration sensor offset in y-axis, resolution tbd
            int16_t z;                          // acceleration sensor offset in z-axis, resolution tbd
        } sensOffset;                           // response parameter for event type BTLE_EVT_LCSCP_REQ_SENS_OFFSET and BTLE_EVT_LCSCP_CALIB_SENS_OFFSET */
        struct
        {
            int8_t floodInPercent;              // flood current limit in % (related to hardware limit)
            int8_t spotInPercent;               // spot current limit in % (related to hardware limit)
        } currentLimits;                        // response parameter for event type BTLE_EVT_LCSCP_REQ_LIMITS
        uint8_t prefMode;                       // response parameter for event type BTLE_EVT_LCSCP_REQ_PREF_MODE
    } responseParams;
} btle_LcscpEventResponse_t;

/**< btle event handler type */
typedef void (*btle_eventHandler_t)(btle_event_t * pEvt);

/**< supported feature structure */
typedef struct
{
    uint8_t floodSupported : 1;
    uint8_t spotSupported  : 1;
    uint8_t pitchSupported : 1;
    /// TODO: expand!
} btle_lcsFeature_t;

/**< scan mode */
typedef enum
{
    BTLE_SCAN_MODE_LOW_POWER = 0,               // low power scanning, scan window 11.25ms, scan interval 1280ms
    BTLE_SCAN_MODE_LOW_LATENCY                  // low latency scanning, scan window 11.25ms, scan interval 44.5ms
} btle_scanMode_t;

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief Function to initialize the ble stack
 */
void btle_StackInit(void);

/** @brief Function to initialize the btle module
 *
 * @param[in]   deleteBonds true to erase all bond information at
 *                          initialization
 * @param[in]   pDrvRev     pointer to driver firmware string
 * @param[in]   pFeature    supported features
 * @param[in]   pEvtHandler event handler for btle events
 */
void btle_Init(bool deleteBonds, char const* pDrvRev, btle_lcsFeature_t* pFeature, btle_eventHandler_t pEvtHandler);

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
