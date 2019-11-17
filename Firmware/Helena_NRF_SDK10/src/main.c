/**
  ******************************************************************************
  * @file    main.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    15/10/26
  * @brief   main module for Helena control board
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "app_timer.h"
#include "btle.h"
#include "comreloaded.h"
#include "com_msg_handling.h"
#include "crc16.h"
#include "debug.h"
#include "hmi.h"
#include "light.h"
#include "main.h"
#include "power.h"
#include "softdevice_handler.h"
#include "app_error.h"
#include "fds.h"
#include "motion_sensor.h"
#include "custom_board.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "fastmath.h"

/* Private defines -----------------------------------------------------------*/
#define APP_TIMER_OP_QUEUE_SIZE     2           // Size of timer operation queues.

#define MAGICNUMBER                 0x83731714  // magic number to verify data in no init section

#define MAX_NUM_OF_MODES            8           // maximum number of modes
#define MAX_NUM_OF_MODES_V13        8           // maximum number for versions prior to v0.14
#define MODE_OFF                    MAX_NUM_OF_MODES
#define MODE_INVALID                UINT8_MAX

#define FDSINSTANCE                 0xCAFE      // number identifying fds data for main module
#define FDSMODECONFIG_V13           0x600D      // number identifying mode configuration fds data till v0.13
#define FDSMODECONFIG               0x80DE      // number identifying mode configuration fds data from v0.14

#define LEDVOLTAGE                  3           // used to calculate output power from led current

#define LIGHT_UPDATE_PERIOD_SHORT   (APP_TIMER_TICKS(10, APP_TIMER_PRESCALER))
#define LIGHT_UPDATE_PERIOD_LONG    (APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER))
#define IDLE_TIMEOUT                (APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)/LIGHT_UPDATE_PERIOD_SHORT)
#define MINIMUM_INPUT_VOLTAGE       (3 << 10)   // 3V per cell in q6_10_t
#define SHUTDOWN_DELAY              (APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER))

#define MODES_DEFAULT                                               \
{                                                                   \
    .mode =                                                         \
    {   /* flood, spot,  pitch, cloned, tail,  brake,  brightness */\
        {{ false, true,  true,  true,   false, false}, {10}      }, \
        {{ false, true,  true,  true,   false, false}, {30}      }, \
        {{ false, false, false, false,  false, false}, {0}       }, \
        {{ false, false, false, false,  false, false}, {0}       }, \
        {{ false, true,  false, true,   false, false}, {90}      }, \
        {{ false, true,  false, true,   false, false}, {210}     }, \
        {{ false, false, false, false,  false, false}, {0}       }, \
        {{ false, false, false, false,  false, false}, {0}       }  \
    },                                                              \
    .groups = 2,                                                    \
    .prefMode = MODE_INVALID                                        \
}

/* Private typedef -----------------------------------------------------------*/
/** @brief bluetooth module related status flags
 */
typedef struct
{
    uint16_t isPeriphConnected;             // hold a valid connection handle if in a connection as peripheral
    uint16_t isCentralConnected;            // hold a valid connection handle if in a connection as central
    uint16_t isModeConfigWritePending;      // hold a valid connection handle if a mode configure write response is pending
    uint16_t isGroupCountPending;           // hold a valid connection handle if a group configuration write response is pending
    uint16_t isLedConfigCheckPending;       // hold a valid connection handle if a led check procedure was requested
    uint16_t isSensorCalibrationPending;    // hold a valid connection handle if sensor offset calibration was requested
    uint16_t isPrefModeWritePending;        // hold a valid connection handle if a preferred mode set response is pending
} btleStatus_t;

/** @brief power state
 */
typedef enum
{
    POWER_SHUTDOWN = 0,                     // complete shutdown, minimum power consumption, power cycle needed to leave
    POWER_STANDBY,                          // light and motion processing off, com and hmi working, ble working with low power scanning
    POWER_IDLE,                             // light off but data read out on a regular basis, motion processing on, com and hmi working, ble working with low latency scanning
    POWER_ON                                // light on and data read out on a regular basis, motion processing on, com and hmi working, ble working with low latency scanning
} powerState_t;

/** @brief selected mode, prepared to be placed in noinit section
 */
typedef struct
{
    uint32_t magicnumber;                   // magic number and
    uint16_t crc;                           // crc value to check if data is valid
    uint8_t currentMode;                    // current mode, MODE_OFF (= MAX_NUM_OF_MODES) to disable
} modeState_t;

/** @brief state status structure
 */
typedef struct
{
    uint32_t idleTimeout;                   // idle timeout prescaler
    volatile bool timeoutFlag;              // flag indicating that main timer interrupt occured
    uint8_t supplyCellCnt;                  // estimated cell count of input voltage
    q6_10_t minInputVoltage;                // minimum input voltage, if input voltage drops below this value, helena enters shutdown mode
    powerState_t power;                     // current power state
    btleStatus_t btle;                      // bluetooth state
    modeState_t *pMode;                     // mode state
} state_t;

/** @brief helena mode setup flags
 */
typedef struct
{
    bool lightFlood             : 1;        // light flood active
    bool lightSpot              : 1;        // light spot active
    bool lightPitchCompensation : 1;        // light pitch compensation
    bool lightCloned            : 1;        // light output cloned to both drivers, ignored if both flood and spot are enabled
    bool comTaillight           : 1;        // secondary taillight enabled through com module
    bool comBrakelight          : 1;        // brake detection enabled end enabled through com module
} helenaModeSetup_t;

/** @brief helena mode structure
 */
typedef struct
{
    helenaModeSetup_t setup;                // setup of this mode
    union
    {
        q8_t intensity;                     // light intensity in case of pitchCompensation is false
        uint8_t illuminanceInLux;           // light illuminance in case of pitchCompensation is true
    };
} helenaMode_t;

/** @brief helena configuration structure
 */
typedef struct
{
    helenaMode_t mode[MAX_NUM_OF_MODES];
    uint8_t groups;
    uint8_t prefMode;
} helenaConfig_t;

/** @brief helena mode types for versions 0.13 and lower
 */
typedef enum
{
    LIGHTMODEV13_OFF = 0,          // light is off
    LIGHTMODEV13_FLOOD,            // flood active
    LIGHTMODEV13_SPOT,             // spot active
    LIGHTMODEV13_FULL,             // flood and spot active
    LIGHTMODEV13_FLOODAPC,         // flood active, with pitch compensation
    LIGHTMODEV13_SPOTAPC,          // spot active with pitch compensation
    LIGHTMODEV13_FULLAPC,          // flood and spot active with pitch compensation
    LIGHTMODEV13_FLOODCLONED,      // flood active, settings cloned to spot driver
    LIGHTMODEV13_SPOTCLONED,       // spot active, settings cloned to flood driver
    LIGHTMODEV13_FLOODAPCCLONED,   // flood active, pitch compensated, settings cloned to spot driver
    LIGHTMODEV13_SPOTAPCCLONED,    // spot active, pitch compensated, settings cloned to flood driver
    LIGHTMODEV13_CNT
} helenaModeTypesV13_t;

/** @brief helena modes for versions 0.13 and lower
 */
typedef struct
{
    helenaModeTypesV13_t mode;
    int8_t intensity;                       // intensity in % or in lux in adaptive modes
} helenaModeV13_t;

/** @brief helena configuration for versions 0.13 and lower
 */
typedef struct
{
    uint8_t modeGroups;
    uint8_t padding[3];                     // padding needed, structure is stored in flash and needs to be word aligned
    helenaModeV13_t modes[MAX_NUM_OF_MODES_V13];
} helenaConfigV13_t;

/* Private variables ---------------------------------------------------------*/
APP_TIMER_DEF(mainTimerId);                                         // timer instance for main timer
static hmi_buttonState_t buttonVolUp, buttonVolDown;                // remote button states
static modeState_t modeState __attribute__((section(".noinit")));   // mode states
static state_t state = {.pMode = &modeState};                       // device states
static helenaConfig_t modeConfig = MODES_DEFAULT;                   // modes and grouping configuration
static light_driverConfig_t ledConfiguration;                       // led and driver information

/* Private macros ------------------------------------------------------------*/
#define COUNT_OF(x)             (sizeof(x)/sizeof(x[0]))
#define SIZE_IN_WORDS(x)        ((sizeof(x) + 3) / 4)

/* Private functions ---------------------------------------------------------*/
/** @brief event handler for incoming connection related ble events
 */
static void btleConnEvtHandler(btle_event_t * pEvt)
{
    switch (pEvt->subEvt.conn)
    {
    case BTLE_EVT_CONN_CENTRAL_CONNECTED:
        state.btle.isCentralConnected = pEvt->connHandle;
        break;
    case BTLE_EVT_CONN_CENTRAL_DISCONNECTED:
        state.btle.isCentralConnected = BTLE_CONN_HANDLE_INVALID;
        break;
    case BTLE_EVT_CONN_PERIPH_CONNECTED:
        state.btle.isPeriphConnected = pEvt->connHandle;
        break;
    case BTLE_EVT_CONN_PERIPH_DISCONNECTED:
        state.btle.isPeriphConnected = BTLE_CONN_HANDLE_INVALID;
    default:
        break;
    }
}

/** @brief event handler for incoming hid related ble events
 */
static void btleHidEvtHandler(btle_event_t * pEvt)
{
    switch (pEvt->subEvt.hid)
    {
    case BTLE_EVT_HID_VOL_UP_SHORT:
        buttonVolUp = HMI_BUTTONSHORT;
        break;
    case BTLE_EVT_HID_VOL_UP_LONG:
        buttonVolUp = HMI_BUTTONLONG;
        break;
    case BTLE_EVT_HID_VOL_DOWN_SHORT:
        buttonVolDown = HMI_BUTTONSHORT;
        break;
    case BTLE_EVT_HID_VOL_DOWN_LONG:
        buttonVolDown = HMI_BUTTONLONG;
        break;
    default:
        break;
    }
}

/** @brief function to update the actual button configuration to flash
 *
 * @note    the update is not stored immediately, when finished, the fds event
 *          handler is called
 */
static uint32_t updateModeConfig()
{
    uint32_t errCode;
    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_chunk_t chunk;

    fds_record_key_t key = {.type = FDSMODECONFIG, .instance = FDSINSTANCE};
    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode != NRF_SUCCESS)
        return errCode;

    chunk.p_data = &modeConfig;
    chunk.length_words = SIZE_IN_WORDS(modeConfig);
    errCode = fds_update(&descriptor, key, 1, &chunk);
    APP_ERROR_CHECK(errCode);
    if (errCode == NRF_ERROR_NO_MEM)    // run garbage collection if no memory available, write operations will be tried again in fds event handler
    {
        errCode = fds_gc();
    }
    APP_ERROR_CHECK(errCode);
    return errCode;
}

/** @brief function to check if configuration is a valid power of 2
 */
static bool isGroupConfigValid(uint8_t groupConfig)
{
    for(uint_fast8_t i = 1; i <= MAX_NUM_OF_MODES; i <<= 1)
    {
        if (groupConfig == i)
            return true;
    }
    return false;
}

/** @brief function to check if mode configuration is valid
 */
static bool isModeConfigValid(const helenaMode_t * pMode)
{
    // cloned mode is not possible if both drivers are on
    if (pMode->setup.lightFlood && pMode->setup.lightSpot && pMode->setup.lightCloned)
        return false;
    // if no light is on, cloned and pitch flags must not be set and output must be 0
    if (!pMode->setup.lightFlood && !pMode->setup.lightSpot &&
        (pMode->setup.lightCloned || pMode->setup.lightPitchCompensation || pMode->intensity != 0))
        return false;
    // if any output is set, output must not be 0
    if ((pMode->setup.lightFlood || pMode->setup.lightSpot) && pMode->intensity == 0)
        return false;
    return true;
}

/** @brief helper function to convert device modes to btle modes
 */
static void convertModetoBtleMode(helenaMode_t const* pMode, btle_LcsModeConfig_t* pBtleMode, uint8_t numOfModes)
{
    while (numOfModes--)
    {
        memset(pBtleMode, 0, sizeof(btle_LcsModeConfig_t));

        pBtleMode->setup.flood = pMode->setup.lightFlood;
        pBtleMode->setup.spot = pMode->setup.lightSpot;
        pBtleMode->setup.pitchCompensation = pMode->setup.lightPitchCompensation;
        pBtleMode->setup.cloned = pMode->setup.lightCloned;
        pBtleMode->setup.taillight = pMode->setup.comTaillight;
        pBtleMode->setup.brakelight = pMode->setup.comBrakelight;
        if (pMode->setup.lightPitchCompensation)
            pBtleMode->illuminanceInLux = pMode->illuminanceInLux;
        else
            pBtleMode->intensityInPercent = (pMode->intensity * 100) / 256;

        pBtleMode++;
        pMode++;
    }
}

/** @brief helper function to convert btle modes to device modes
 */
static bool convertBtleModeToMode(btle_LcsModeConfig_t const* pBtleMode, helenaMode_t* pMode, uint8_t numOfModes)
{
    while (numOfModes--)
    {
        memset(pMode, 0, sizeof(helenaMode_t));

        pMode->setup.lightFlood = pBtleMode->setup.flood;
        pMode->setup.lightSpot = pBtleMode->setup.spot;
        pMode->setup.lightPitchCompensation = pBtleMode->setup.pitchCompensation;
        pMode->setup.lightCloned = pBtleMode->setup.cloned;
        pMode->setup.comTaillight = pBtleMode->setup.taillight;
        pMode->setup.comBrakelight = pBtleMode->setup.brakelight;

        if (pMode->setup.lightPitchCompensation)
            pMode->illuminanceInLux = pBtleMode->illuminanceInLux;
        else
            pMode->intensity = (pBtleMode->intensityInPercent * 256) / 100;

        if (!isModeConfigValid(pMode))
            return false;

        pBtleMode++;
        pMode++;
    }
    return true;
}

/** @brief function to enter a specific mode
 *
 * @param[in]   mode        the new mode
 * @param[in]   connHandle  the connection handle this mode shall be relayed to, BTLE_CONN_HANDLE_ALL to send to all connected peers
 */
static void enterMode(uint8_t mode, uint16_t connHandle)
{
    state.pMode->currentMode = mode;
    (void)btle_SetMode(mode, connHandle);
    (void)cmh_EnableTaillight(mode == MODE_OFF ? false : modeConfig.mode[mode].setup.comTaillight);
}

/** @brief function to check if a mode is a light mode (spot and/or flood are/is enabled)
 */
static bool isLightMode(helenaMode_t const* const pMode)
{
    if (pMode != NULL &&
        pMode->intensity && (pMode->setup.lightFlood || pMode->setup.lightSpot))
        return true;
    else
        return false;
}

/** @brief event handler for incoming ble events related to the light control service
 */
static void btleLcscpEventHandler(btle_event_t * pEvt)
{
    static union
    {
        btle_LcsModeConfig_t btle[MAX_NUM_OF_MODES];
        helenaMode_t main[MAX_NUM_OF_MODES];
    } modes;
    btle_LcscpEventResponse_t rsp;
    uint32_t errCode;

    memset(&rsp, 0, sizeof(btle_LcscpEventResponse_t));
    rsp.evt = pEvt->subEvt.lcscp;

    switch (pEvt->subEvt.lcscp)
    {
    case BTLE_EVT_LCSCP_REQ_MODE_CNT:       // mode count request
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.modeCnt = MAX_NUM_OF_MODES;
        break;
    case BTLE_EVT_LCSCP_REQ_GROUP_CNT:      // group count request
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.groupCnt = modeConfig.groups;
        break;
    case BTLE_EVT_LCSCP_REQ_MODE_CONFIG:    // mode configuration request
    {
        uint8_t start = pEvt->lcscpEventParams.modeConfigStart;
        uint8_t numOfModes = MAX_NUM_OF_MODES - start;
        if (start >= MAX_NUM_OF_MODES)
        {
            rsp.retCode = BTLE_RET_INVALID;
            break;
        }
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.modeList.pList = modes.btle;
        rsp.responseParams.modeList.listEntries = numOfModes;
        convertModetoBtleMode(&modeConfig.mode[start], modes.btle, numOfModes);
        break;
    }
    case BTLE_EVT_LCSCP_SET_MODE:           // mode set request
    {   uint8_t newMode;
        uint16_t connHandle = BTLE_CONN_HANDLE_INVALID;

        if (pEvt->lcscpEventParams.modeToSet >= MAX_NUM_OF_MODES)
            newMode = MODE_OFF;
        else
            newMode = pEvt->lcscpEventParams.modeToSet;

        if (pEvt->connHandle == state.btle.isPeriphConnected)
            connHandle = state.btle.isCentralConnected;
        else if (pEvt->connHandle == state.btle.isCentralConnected)
            connHandle = state.btle.isPeriphConnected;

        enterMode(newMode, connHandle);
        rsp.retCode = BTLE_RET_SUCCESS;
    }   break;
    case BTLE_EVT_LCSCP_CONFIG_MODE:        // mode configuration request
    {
        if (state.btle.isGroupCountPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isModeConfigWritePending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isLedConfigCheckPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isSensorCalibrationPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isPrefModeWritePending != BTLE_CONN_HANDLE_INVALID)
        {
            rsp.retCode = BTLE_RET_FAILED;
            break;
        }

        uint8_t start = pEvt->lcscpEventParams.modeToConfig.modeNumber;
        uint8_t numOfModes = pEvt->lcscpEventParams.modeToConfig.listEntries;
        if (numOfModes > MAX_NUM_OF_MODES - start ||
            !convertBtleModeToMode(pEvt->lcscpEventParams.modeToConfig.pConfig, modes.main, numOfModes))
        {
            rsp.retCode = BTLE_RET_INVALID;
            break;
        }

        memcpy(&modeConfig.mode[start], modes.main, sizeof(helenaMode_t) * numOfModes);
        errCode = updateModeConfig();
        if (errCode == NRF_SUCCESS)
        {
            state.btle.isModeConfigWritePending = pEvt->connHandle;
            return;
        }
        else
            rsp.retCode = BTLE_RET_FAILED;
    }   break;
    case BTLE_EVT_LCSCP_CONFIG_GROUP:       // group configuration request
        if (state.btle.isGroupCountPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isModeConfigWritePending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isLedConfigCheckPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isSensorCalibrationPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isPrefModeWritePending != BTLE_CONN_HANDLE_INVALID)
        {
            rsp.retCode = BTLE_RET_FAILED;
            break;
        }
        if (!isGroupConfigValid(pEvt->lcscpEventParams.groupConfig))
        {
            rsp.retCode = BTLE_RET_INVALID;
            break;
        }

        modeConfig.groups = pEvt->lcscpEventParams.groupConfig;
        errCode = updateModeConfig();
        if (errCode == NRF_SUCCESS)
        {
            state.btle.isGroupCountPending = pEvt->connHandle;
            return;
        }
        else
            rsp.retCode = BTLE_RET_FAILED;
        break;
    case BTLE_EVT_LCSCP_REQ_LED_CONFIG:
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.ledConfig.floodCnt = ledConfiguration.floodCount;
        rsp.responseParams.ledConfig.spotCnt = ledConfiguration.spotCount;
        break;
    case BTLE_EVT_LCSCP_CHECK_LED_CONFIG:
        if (state.btle.isGroupCountPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isModeConfigWritePending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isLedConfigCheckPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isSensorCalibrationPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isPrefModeWritePending != BTLE_CONN_HANDLE_INVALID)
        {
            rsp.retCode = BTLE_RET_FAILED;
            break;
        }

        if (state.power != POWER_ON)
        {
            state.btle.isLedConfigCheckPending = pEvt->connHandle;
            return;
        }
        else
            rsp.retCode = BTLE_RET_INVALID;
    case BTLE_EVT_LCSCP_REQ_SENS_OFFSET:
        if (ms_GetSensorOffset((ms_accelerationData_t*)&rsp.responseParams.sensOffset) == NRF_SUCCESS)
            rsp.retCode = BTLE_RET_SUCCESS;
        else
            rsp.retCode = BTLE_RET_INVALID;
        break;
    case BTLE_EVT_LCSCP_CALIB_SENS_OFFSET:
        if (state.btle.isGroupCountPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isModeConfigWritePending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isLedConfigCheckPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isSensorCalibrationPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isPrefModeWritePending != BTLE_CONN_HANDLE_INVALID)
        {
            rsp.retCode = BTLE_RET_FAILED;
            break;
        }
        state.btle.isSensorCalibrationPending = pEvt->connHandle;
        return;
    case BTLE_EVT_LCSCP_REQ_LIMITS:
    {
        q8_t floodLimit, spotLimit;
        if (light_GetLimits(&floodLimit, &spotLimit) == NRF_SUCCESS)
        {
            rsp.retCode = BTLE_RET_SUCCESS;
            rsp.responseParams.currentLimits.floodInPercent = (floodLimit * 100) >> 8;
            rsp.responseParams.currentLimits.spotInPercent = (spotLimit * 100) >> 8;
        }
        else
            rsp.retCode = BTLE_RET_FAILED;
    }   break;
    case BTLE_EVT_LCSCP_SET_LIMITS:
        if (light_SetLimits((q8_t)((pEvt->lcscpEventParams.currentLimits.floodInPercent << 8) / 100),
                            (q8_t)((pEvt->lcscpEventParams.currentLimits.spotInPercent << 8) / 100)) == NRF_SUCCESS)
            rsp.retCode = BTLE_RET_SUCCESS;
        else
            rsp.retCode = BTLE_RET_FAILED;
    case BTLE_EVT_LCSCP_REQ_PREF_MODE:
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.prefMode = modeConfig.prefMode;
        break;
    case BTLE_EVT_LCSCP_SET_PREF_MODE:
        if (state.btle.isGroupCountPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isModeConfigWritePending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isLedConfigCheckPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isSensorCalibrationPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isPrefModeWritePending != BTLE_CONN_HANDLE_INVALID)
        {
            rsp.retCode = BTLE_RET_FAILED;
            break;
        }
        if (isLightMode(&modeConfig.mode[pEvt->lcscpEventParams.prefMode]) ||
            pEvt->lcscpEventParams.prefMode >= MAX_NUM_OF_MODES)
        {
            if (pEvt->lcscpEventParams.prefMode >= MAX_NUM_OF_MODES)
                modeConfig.prefMode = MODE_INVALID;
            else
                modeConfig.prefMode = pEvt->lcscpEventParams.prefMode;
            errCode = updateModeConfig();
            if (errCode == NRF_SUCCESS)
            {
                state.btle.isPrefModeWritePending = pEvt->connHandle;
                return;
            }
        }
        rsp.retCode = BTLE_RET_FAILED;
        break;
    default:
        break;
    }
    // if this point is reached, send response immediately, otherwise it is not necessary or will be done later.
    APP_ERROR_CHECK(btle_SendEventResponse(&rsp, pEvt->connHandle));
}

/** @brief event handler for incoming ble events
 */
static void btleEventHandler(btle_event_t * pEvt)
{
    switch (pEvt->evt)
    {
    case BTLE_EVT_CONNECTION:
        btleConnEvtHandler(pEvt);
        break;
    case BTLE_EVT_HID:
        btleHidEvtHandler(pEvt);
        break;
    case BTLE_EVT_LCS_CTRL_POINT:
        btleLcscpEventHandler(pEvt);
        break;
    default:
        break;
    }
}

/** @brief event handler for incoming com messages
 */
static void lightMasterHandler(cmh_lightMasterData_t const* pMasterData)
{
    // messages from com will set the mode directly,
    // no relaying to other lights or taillight necessary

    // com mode city/fog low
    if (pMasterData->mainBeam == cmh_LIGHTLOW &&
        pMasterData->highBeam == cmh_LIGHTOFF &&
        pMasterData->helmetBeam == cmh_LIGHTOFF)
        state.pMode->currentMode = 0;
    // com mode city/fog high
    if (pMasterData->mainBeam == cmh_LIGHTFULL &&
        pMasterData->highBeam == cmh_LIGHTFULL &&
        pMasterData->helmetBeam == cmh_LIGHTOFF)
        state.pMode->currentMode = 1;
    // com mode trail uphill
    if (pMasterData->mainBeam == cmh_LIGHTLOW &&
        pMasterData->highBeam == cmh_LIGHTOFF &&
        pMasterData->helmetBeam == cmh_LIGHTLOW)
        state.pMode->currentMode = 2;
    // com mode trail downhill
    else if (pMasterData->mainBeam == cmh_LIGHTOFF &&
        pMasterData->highBeam == cmh_LIGHTFULL &&
        pMasterData->helmetBeam == cmh_LIGHTFULL)
        state.pMode->currentMode = 3;
    else
        state.pMode->currentMode = MODE_OFF;
}

/** @brief main timer event handler
 *
 * @details this function just sets a flag, corresponding actions are done in main loop
 */
static void timerHandler(void* pContext)
{
    (void)pContext;

    if (state.idleTimeout)
    {
        state.idleTimeout--;
        state.timeoutFlag = true;
    }
}

/** @brief fds event handler
 *
 * @details this function checks the results of file operations and sends the
 *          appropriate event response to the ble modules light control service
 */
static void fdsEventHandler(ret_code_t errCode, fds_cmd_id_t cmd, fds_record_id_t recordId, fds_record_key_t recordKey)
{
    // check only events of interest
    if (recordKey.instance == FDSINSTANCE && recordKey.type == FDSMODECONFIG)
    {
        // check if this is an event related to update or write operations
        if (cmd == FDS_CMD_UPDATE || cmd == FDS_CMD_WRITE)
        {
            // send notification if necessary
            if (state.btle.isGroupCountPending != BTLE_CONN_HANDLE_INVALID)
            {
                btle_LcscpEventResponse_t rsp;
                rsp.evt = BTLE_EVT_LCSCP_CONFIG_GROUP;
                rsp.retCode = errCode == NRF_SUCCESS ? BTLE_RET_SUCCESS : BTLE_RET_FAILED;
                APP_ERROR_CHECK(btle_SendEventResponse(&rsp, state.btle.isGroupCountPending));

                state.btle.isGroupCountPending = BTLE_CONN_HANDLE_INVALID;
            }

            if (state.btle.isModeConfigWritePending != BTLE_CONN_HANDLE_INVALID)
            {
                btle_LcscpEventResponse_t rsp;
                rsp.evt = BTLE_EVT_LCSCP_CONFIG_MODE;
                rsp.retCode = errCode == NRF_SUCCESS ? BTLE_RET_SUCCESS : BTLE_RET_FAILED;
                APP_ERROR_CHECK(btle_SendEventResponse(&rsp, state.btle.isModeConfigWritePending));

                state.btle.isModeConfigWritePending = BTLE_CONN_HANDLE_INVALID;
            }

            if (state.btle.isPrefModeWritePending != BTLE_CONN_HANDLE_INVALID)
            {
                btle_LcscpEventResponse_t rsp;
                rsp.evt = BTLE_EVT_LCSCP_SET_PREF_MODE;
                rsp.retCode = errCode == NRF_SUCCESS ? BTLE_RET_SUCCESS : BTLE_RET_FAILED;
                APP_ERROR_CHECK(btle_SendEventResponse(&rsp, state.btle.isPrefModeWritePending));
                state.btle.isPrefModeWritePending = BTLE_CONN_HANDLE_INVALID;
            }

            // run garbage collection if necessary
            if (errCode == NRF_ERROR_NO_MEM)
            {
                errCode = fds_gc();
                if (errCode != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(errCode);
                }
            }
        }
    }

    if (cmd == FDS_CMD_GC) // garbage collection finished, check if write operations are pending
    {
        if (state.btle.isGroupCountPending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isModeConfigWritePending != BTLE_CONN_HANDLE_INVALID ||
            state.btle.isPrefModeWritePending != BTLE_CONN_HANDLE_INVALID)
            APP_ERROR_CHECK(updateModeConfig());
    }
}

static void movementEventHandler()
{
    state.idleTimeout = IDLE_TIMEOUT;
}

/** @brief function to convert old configurations (<= v0.13) to new configuration structure
 */
static void convertV13ToMode(helenaConfig_t* pNew, helenaConfigV13_t const* pOld)
{
    uint8_t numOfModes = MAX_NUM_OF_MODES_V13 < MAX_NUM_OF_MODES ? MAX_NUM_OF_MODES_V13 : MAX_NUM_OF_MODES;
    helenaMode_t* pModeNew = pNew->mode;
    helenaModeV13_t const* pModeOld = pOld->modes;

    while(numOfModes--)
    {
        uint8_t mode = pModeOld->mode;
        if (mode < LIGHTMODEV13_CNT)
        {
            if (mode >= 9) mode += 4;
            else if (mode >= 7) mode += 2;
            else if (mode >= 4) mode += 1;

            if (mode & 1) pModeNew->setup.lightFlood = true;
            if (mode & 2) pModeNew->setup.lightSpot = true;
            if (mode & 4) pModeNew->setup.lightPitchCompensation = true;
            if (mode & 8) pModeNew->setup.lightCloned = true;

            if (pModeNew->setup.lightPitchCompensation)
                pModeNew->illuminanceInLux = pModeOld->intensity;
            else
                pModeNew->intensity = (pModeOld->intensity * 256) / 100;
        }

        pModeOld++;
        pModeNew++;
    }

    pNew->groups = pOld->modeGroups;
    pNew->prefMode = MODE_INVALID;
}

/** @brief function to load the mode configuration
 *
 * @details this function tries to load previous configuration out of flash memory,
 *          if no configuration is stored yet or if configuration is not valid,
 *          it loads and stores the default configuration
 */
static void loadModeConfig()
{
    uint32_t errCode;

    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_key_t key;
    helenaConfig_t modeCfg;

    memset(&modeCfg, 0, sizeof(modeCfg));

    key.instance = FDSINSTANCE;
    key.type = FDSMODECONFIG;
    errCode = fds_find(key.type, key.instance, &descriptor, &token);    // search for configuration in flash
    if (errCode == NRF_ERROR_NOT_FOUND)
    {
        key.type = FDSMODECONFIG_V13;
        errCode = fds_find(key.type, key.instance, &descriptor, &token);// search for old configuration if new one not found
    }
    if (errCode == NRF_SUCCESS)                                         // data found, copy (or convert) into temporary buffer
    {
        fds_record_t record;

        if (fds_open(&descriptor, &record) == NRF_SUCCESS)
        {
            if (key.type == FDSMODECONFIG)
                memcpy(&modeCfg, (helenaConfig_t*)record.p_data, record.header.tl.length_words * 4);
            else if (key.type == FDSMODECONFIG_V13)
            {
                convertV13ToMode(&modeCfg, (helenaConfigV13_t*)record.p_data);
                //APP_ERROR_CHECK(fds_clear(&descriptor));/// TODO: delete old config from flash
                errCode = NRF_ERROR_NOT_FOUND;  // set to not found to make sure converted data will be saved to flash
            }
            APP_ERROR_CHECK(fds_close(&descriptor));
        }
    }
    else if (errCode != NRF_ERROR_NOT_FOUND)
    {
        APP_ERROR_HANDLER(errCode);
    }
    // check if data is valid (this will fail if no data was found, because group config will be 0)
    bool isValid = true;
    if (!isGroupConfigValid(modeCfg.groups))
        isValid = false;
    for (int_fast8_t i = 0; i < MAX_NUM_OF_MODES; i++)
    {
        if (!isModeConfigValid(&modeCfg.mode[i]))
        {
            isValid = false;
            break;
        }
    }
    if (isValid)
        modeConfig = modeCfg;                                   // copy into configuration structure is data is valid
    if (!isValid || errCode == NRF_ERROR_NOT_FOUND)             // if data is not valid or old data has been converted, save values to flash
    {
        fds_record_chunk_t chunk;
        chunk.p_data = &modeConfig;
        chunk.length_words = SIZE_IN_WORDS(modeConfig);
        key.type = FDSMODECONFIG;
        if (errCode == NRF_ERROR_NOT_FOUND)
            errCode = fds_write(&descriptor, key, 1, &chunk);   // write record
        else //if(!isValid)
            errCode = fds_update(&descriptor, key, 1, &chunk);  // or update
        APP_ERROR_CHECK(errCode);
    }
}

/** @brief function to change helena power state
 */
static uint32_t setHelenaState(powerState_t newState)
{
    if (state.power == newState)
        return NRF_SUCCESS;

    switch (newState)
    {
    case POWER_SHUTDOWN:
        // shut down everything, there will be no return from this state, power cycle needed
        if (state.power >= POWER_IDLE)
        {
            APP_ERROR_CHECK(light_Enable(false));
            APP_ERROR_CHECK(ms_Enable(false));
        }
        hmi_SetLed(HMI_LEDBLUE, HMI_LEDOFF);
        hmi_SetLed(HMI_LEDRED, HMI_LEDOFF);
        APP_ERROR_CHECK(app_timer_stop(mainTimerId));
        nrf_gpio_cfg_default(pBoardConfig->bdepRX); // default config for com pin and button
        nrf_gpio_cfg_default(pBoardConfig->button); // to disable DETECT signal
        APP_ERROR_CHECK(sd_power_system_off());
        break;
    case POWER_STANDBY:
        // shut down light and motion sensor
        APP_ERROR_CHECK(light_Enable(false));
        APP_ERROR_CHECK(ms_Enable(false));
        hmi_SetLed(HMI_LEDBLUE, HMI_LEDOFF);
        hmi_SetLed(HMI_LEDRED, HMI_LEDOFF);
        btle_SetScanConfig(BTLE_SCAN_MODE_LOW_POWER);
        (void)app_timer_stop(mainTimerId);
        APP_ERROR_CHECK(app_timer_start(mainTimerId, LIGHT_UPDATE_PERIOD_LONG, NULL));  // used to keep timer running, otherwise RTC will be stopped.
        break;
    case POWER_IDLE:
        // if helena was on, nothing need to be changed, otherwise timer needs to be started
        if (state.power != POWER_ON)
        {
            APP_ERROR_CHECK(light_Enable(true));
            APP_ERROR_CHECK(ms_Enable(true));
            btle_SetScanConfig(BTLE_SCAN_MODE_LOW_LATENCY);
            (void)app_timer_stop(mainTimerId);
            APP_ERROR_CHECK(app_timer_start(mainTimerId, LIGHT_UPDATE_PERIOD_SHORT, NULL));
        }
        state.timeoutFlag = true;   // set timeout flag to trigger changes
        break;
    case POWER_ON:
        // if helena was in idle mode, nothing has to be changed, otherwise everything needs to be started
        if (state.power != POWER_IDLE)
        {
            APP_ERROR_CHECK(light_Enable(true));
            APP_ERROR_CHECK(ms_Enable(true));
            btle_SetScanConfig(BTLE_SCAN_MODE_LOW_LATENCY);
            (void)app_timer_stop(mainTimerId);
            APP_ERROR_CHECK(app_timer_start(mainTimerId, LIGHT_UPDATE_PERIOD_SHORT, NULL));
        }
        state.timeoutFlag = true; // set timeout flag to trigger changes
        break;
    }

    state.power = newState;

    return NRF_SUCCESS;
}

/**@brief Function for initialization the main module.
 */
static void mainInit(void)
{
    uint32_t errCode;

    // initialize timer
    errCode = app_timer_create(&mainTimerId, APP_TIMER_MODE_REPEATED, timerHandler);
    APP_ERROR_CHECK(errCode);

    // initialize mode state if content is not valid
    if (1 &&    /// TODO: reset survive
        state.pMode->magicnumber != MAGICNUMBER &&
        state.pMode->crc != crc16_compute((const uint8_t*)&state.pMode->magicnumber, sizeof(state.pMode->magicnumber), NULL))
    {
        state.pMode->currentMode = MODE_OFF;
        state.pMode->magicnumber = MAGICNUMBER;
        state.pMode->crc = crc16_compute((const uint8_t*)&state.pMode->magicnumber, sizeof(state.pMode->magicnumber), NULL);
    }

    // register to fds module
    errCode = fds_register(fdsEventHandler);
    APP_ERROR_CHECK(errCode);
    errCode = fds_init();
    APP_ERROR_CHECK(errCode);

    // load button mode configuration from flash
    loadModeConfig();

    // set minimum input voltage
    state.minInputVoltage = state.supplyCellCnt * MINIMUM_INPUT_VOLTAGE;

    APP_ERROR_CHECK(setHelenaState(POWER_IDLE));
    state.idleTimeout = IDLE_TIMEOUT;

    state.btle.isPeriphConnected = BTLE_CONN_HANDLE_INVALID;
    state.btle.isCentralConnected = BTLE_CONN_HANDLE_INVALID;
    state.btle.isGroupCountPending = BTLE_CONN_HANDLE_INVALID;
    state.btle.isModeConfigWritePending = BTLE_CONN_HANDLE_INVALID;
    state.btle.isLedConfigCheckPending = BTLE_CONN_HANDLE_INVALID;
    state.btle.isSensorCalibrationPending = BTLE_CONN_HANDLE_INVALID;
    state.btle.isPrefModeWritePending = BTLE_CONN_HANDLE_INVALID;
}

/** @brief function to get the next valid light mode
 *
 * @param[in]   mode    themode to start with
 * @return      the first valid mode found, MODE_OFF if no valid mode
 *
 * @note        This function will first start searching for valid mode within
 *              the group, if nothing is found it will jump to the next
 *              group(s). If no valid mode is found at all, MODE_OFF-will be
 *              returned.
 */
static uint8_t getNextValidMode(uint8_t mode)
{
    int_fast8_t groups = modeConfig.groups;
    int_fast8_t modesInGroup = MAX_NUM_OF_MODES / groups;
    for (int_fast8_t i = 0; i < groups; i++)
    {
        for (int_fast8_t j = 0; j < modesInGroup; j++)
        {
            if (isLightMode(&modeConfig.mode[mode]))        // if mode is enabled, return immediately
                return mode;

            if (++mode % modesInGroup)                      // otherwise loop through group
                mode -= modesInGroup;
        }

        mode += modesInGroup;                               // if no valid mode in group is found, loop through next group
        mode %= MAX_NUM_OF_MODES;
    }

    return MODE_OFF;                                        // no valid mode was found
}

/** @brief function to change modes
 *
 * @param[in]   mode    number of modes to in-/decrease, will not leave group
 * @param[in]   group   number of groups to in-/decrease
 * @return      new mode
 */
static uint8_t changeMode(int8_t mode, int8_t group)
{
    uint8_t groups = modeConfig.groups;
    uint8_t modesPerGroup = MAX_NUM_OF_MODES / groups;
    uint8_t currentGroup = state.pMode->currentMode / modesPerGroup;
    uint8_t modeInGroup = state.pMode->currentMode % modesPerGroup;

    modeInGroup += mode;
    modeInGroup %= modesPerGroup;

    currentGroup += group;
    currentGroup %= groups;

    return currentGroup * modesPerGroup + modeInGroup;
}

/** @brief function for handling button events
 */
static void processButtons(hmi_buttonState_t* pInternal, hmi_buttonState_t* pVolumeUp, hmi_buttonState_t* pVolumeDown)
{
    do
    {
        uint_fast8_t newMode;

        // ultra long press of internal button
        if (*pInternal == HMI_BUTTONULTRALONG)
        {
            if (state.pMode->currentMode == MODE_OFF)       // either search for remote
            {
                APP_ERROR_CHECK(btle_DeleteBonds());
                APP_ERROR_CHECK(btle_SearchForRemote());
            }
            else                                            // or shut off
                enterMode(MODE_OFF, BTLE_CONN_HANDLE_ALL);
            break;
        }

        // button press of external volume down button
        if (*pVolumeDown != HMI_BUTTONNOPRESS)
        {
            if (modeConfig.prefMode != MODE_INVALID &&
                (state.pMode->currentMode == modeConfig.prefMode || !isLightMode(&modeConfig.mode[modeConfig.prefMode])))
                enterMode(MODE_OFF, BTLE_CONN_HANDLE_ALL);
            else
                enterMode(modeConfig.prefMode, BTLE_CONN_HANDLE_ALL);
            break;
        }

        // increase to next mode for short button press
        if (*pInternal == HMI_BUTTONSHORT || *pVolumeUp == HMI_BUTTONSHORT)
        {
            if (state.pMode->currentMode == MODE_OFF)
            {
                if (modeConfig.prefMode != MODE_INVALID && isLightMode(&modeConfig.mode[modeConfig.prefMode]))
                    newMode = modeConfig.prefMode;
                else
                    newMode = 0;
            }
            else
                newMode = changeMode(1, 0);
        }
        // switch to next group for long button press
        else if (state.pMode->currentMode != MODE_OFF && (*pInternal == HMI_BUTTONLONG || *pVolumeUp == HMI_BUTTONLONG))
            newMode = changeMode(0, 1);
        else
            break;

        newMode = getNextValidMode(newMode);
        enterMode(newMode, BTLE_CONN_HANDLE_ALL);
    } while(0);

    // clear buttons
    *pInternal = HMI_BUTTONNOPRESS;
    *pVolumeDown = HMI_BUTTONNOPRESS;
    *pVolumeUp = HMI_BUTTONNOPRESS;
}

/** @brief function to handle the status led
 */
static void ledHandling(light_limiterActive_t flood, light_limiterActive_t spot, btleStatus_t *pBtleData)
{
    // red led indicating if any voltage or temperature limiting is active
    if (state.power >= POWER_IDLE &&
        (flood.voltage || flood.temperature || spot.voltage || spot.temperature))
        hmi_SetLed(HMI_LEDRED, HMI_LEDON);
    else
        hmi_SetLed(HMI_LEDRED, HMI_LEDOFF);
    // blue led indicating if a central connection (e.g. to a remote) is established
    if (state.power >= POWER_IDLE &&
        (pBtleData->isCentralConnected != BTLE_CONN_HANDLE_INVALID))
        hmi_SetLed(HMI_LEDBLUE, HMI_LEDON);
    else
        hmi_SetLed(HMI_LEDBLUE, HMI_LEDOFF);
}

/** @brief function to update the light information data for the com module
 */
static void updateLightMsgData(const light_status_t * pStatus)
{
    cmh_helmetLight_t helmetBeam;

    memset(&helmetBeam, 0, sizeof(cmh_helmetLight_t));
    if (pStatus->flood.current || pStatus->spot.current)
        helmetBeam.overcurrentError = 1;
    if (pStatus->flood.voltage || pStatus->spot.voltage)
        helmetBeam.voltageError = 1;
    if (pStatus->flood.temperature || pStatus->spot.temperature)
        helmetBeam.temperatureError = 1;
    if (pStatus->flood.dutycycle || pStatus->spot.dutycycle)
        helmetBeam.directdriveError = 1;
    helmetBeam.mode = state.pMode->currentMode == 0 ? cmh_LIGHTLOW :
                      state.pMode->currentMode == 1 ? cmh_LIGHTFULL :
                      cmh_LIGHTOFF;
    helmetBeam.current = ((pStatus->currentFlood + pStatus->currentSpot) * 1000ul) >> 10;
    helmetBeam.temperature = (pStatus->temperature * 10l) >> 4;
    helmetBeam.voltage = (pStatus->inputVoltage * 1000ul) >> 10;

    // ignore error codes
    (void)cmh_UpdateHelmetLight(&helmetBeam);
}

/** @brief function to update the light informations for the ble module
 */
static void updateLightBtleData(helenaMode_t const * const pMode, light_status_t const* pStatus, q15_t pitch)
{
    btle_lcsMeasurement_t helmetBeam;
    uint16_t powerFlood, powerSpot;
    helenaMode_t const* pM;
    uint16_t ledVoltFlood, ledVoltSpot;

    ledVoltFlood = ledConfiguration.floodCount == LIGHT_LEDCONFIG_UNKNOWN ? LEDVOLTAGE :
                   ledConfiguration.floodCount * LEDVOLTAGE;
    ledVoltSpot = ledConfiguration.spotCount == LIGHT_LEDCONFIG_UNKNOWN ? LEDVOLTAGE :
                   ledConfiguration.spotCount * LEDVOLTAGE;

    powerFlood = (pStatus->currentFlood * ledVoltFlood * 1000ul) >> 10;
    powerSpot = (pStatus->currentSpot * ledVoltSpot * 1000ul) >> 10;

    pM = pMode;     // copy pointer, convert function will increase pointer
    if (pM != NULL)
        convertModetoBtleMode(pM, &helmetBeam.mode, 1);
    else
        memset(&helmetBeam.mode, 0, sizeof(helmetBeam.mode));

    helmetBeam.statusFlood.overcurrent = pStatus->flood.current;
    helmetBeam.statusFlood.inputVoltage = pStatus->flood.voltage;
    helmetBeam.statusFlood.temperature = pStatus->flood.temperature;
    helmetBeam.statusFlood.dutyCycleLimit = pStatus->flood.dutycycle;
    helmetBeam.statusSpot.overcurrent = pStatus->spot.current;
    helmetBeam.statusSpot.inputVoltage = pStatus->spot.voltage;
    helmetBeam.statusSpot.temperature = pStatus->spot.temperature;
    helmetBeam.statusSpot.dutyCycleLimit = pStatus->spot.dutycycle;
    helmetBeam.temperature = (int16_t)(pStatus->temperature >> 4) - 273;
    helmetBeam.inputVoltage = (pStatus->inputVoltage * 1000ul) >> 10;
    helmetBeam.powerFlood = powerFlood;
    helmetBeam.powerSpot = powerSpot;
    if (pMode->setup.lightCloned && pMode->setup.lightFlood)
        helmetBeam.powerFlood += powerSpot;
    if (pMode->setup.lightCloned && pMode->setup.lightSpot)
        helmetBeam.powerSpot += powerFlood;
    if (pitch < 0)
        pitch = INT8_MIN;
    else
    {
        if (pitch >= (1 << 14))             // convert 0°..360° to -180°..180°
            pitch -= (1l << 15);
        if (pitch > (1 << 13))              // flip 90°..180° to 90°..0°
            pitch = (1 << 14) - pitch;
        else if (-pitch > (1 << 13))        // flip -90°..-180° to -90°..0°
            pitch = -1 * (1 << 14) - pitch;
        pitch = (pitch * 360l) >> 15;       // convert into degree
        helmetBeam.pitch = pitch;
    }

   (void)btle_UpdateLcsMeasurements(&helmetBeam);
}

static void brakeDetection(bool indicate)
{
    if (modeConfig.mode[state.pMode->currentMode].setup.comBrakelight && indicate)
        cmh_UpdateBrakeIndicator(true);
}

static void powerStateCheck()
{
    uint32_t errCode;
    uint8_t currentMode = state.pMode->currentMode;
    helenaMode_t* pMode = currentMode == MODE_OFF ? NULL : &modeConfig.mode[currentMode];
    powerState_t newPowerState = state.power;

    // change to ON mode necessary ?
    if (state.power != POWER_ON &&                                                      //  power on
        currentMode != MODE_OFF && isLightMode(pMode))                                  // if light is on
        newPowerState = POWER_ON;

    // change to IDLE mode possible ?
    if (state.power != POWER_IDLE &&                                                    //  idle
        ((currentMode == MODE_OFF && state.idleTimeout) ||                              // if mode is off but timeout still counting
         (currentMode != MODE_OFF && pMode->setup.comBrakelight && !isLightMode(pMode))))// or a non light mode with brake detection
        newPowerState = POWER_IDLE;

    // change to STANDBY ?
    if (state.power != POWER_STANDBY &&                                                     //  standby
        state.idleTimeout == 0 &&                                                       // if idle timeout expired and
        (currentMode == MODE_OFF ||                                                     // mode is of
         (currentMode != MODE_OFF && !pMode->setup.comBrakelight && !isLightMode(pMode))))// or neither light nor brake detection is used
        newPowerState = POWER_STANDBY;

    // change to SHUTDOWN ?
    if (state.power > POWER_STANDBY)                                                       /// TODO: do this checking in STANDBY mode, too?
    {
        static pwr_inputVoltage_t voltageLast;
        pwr_inputVoltage_t voltageNow;
        errCode = pwr_GetInputVoltage(&voltageNow);
        if (errCode == NRF_SUCCESS && voltageNow.timestamp != 0 && state.supplyCellCnt != 0)
        {                                                           // perform checks only if
            if (voltageLast.timestamp != 0 &&                       // valid values are available and
                voltageLast.inputVoltage < state.minInputVoltage && // both value (actual and last stored)
                voltageNow.inputVoltage < state.minInputVoltage)    // are below threshold
            {
                uint32_t timediff;
                (void)app_timer_cnt_diff_compute(voltageNow.timestamp, voltageLast.timestamp, &timediff);
                if (timediff > SHUTDOWN_DELAY)                      // shutdown helena if input voltage has been
                    newPowerState = POWER_SHUTDOWN;                 //below threshold for specific time period
            }
                                                                    // store actual values if
            if (voltageNow.inputVoltage >= state.minInputVoltage || // input voltage is above threshold or
                voltageLast.timestamp == 0 ||                       // no valid values stored yet or
                voltageLast.inputVoltage >= state.minInputVoltage)  // last values was above threshold
                voltageLast = voltageNow;
        }
    }

    if (newPowerState != state.power)
    {
        errCode = setHelenaState(newPowerState);
        APP_ERROR_CHECK(errCode);
    }

    if (state.pMode->currentMode != MODE_OFF && (pMode->setup.comBrakelight || isLightMode(pMode)))
        state.idleTimeout = IDLE_TIMEOUT;
}

static void ledConfigCheck()
{
    if (state.btle.isLedConfigCheckPending != BTLE_CONN_HANDLE_INVALID)
    {
        btle_LcscpEventResponse_t rsp;

        rsp.evt = BTLE_EVT_LCSCP_CHECK_LED_CONFIG;
        if (light_CheckLedConfig(&ledConfiguration) == NRF_SUCCESS)
            rsp.retCode = BTLE_RET_SUCCESS;
        else
            rsp.retCode = BTLE_RET_FAILED;
        rsp.responseParams.ledConfig.floodCnt = ledConfiguration.floodCount;
        rsp.responseParams.ledConfig.spotCnt = ledConfiguration.spotCount;
        APP_ERROR_CHECK(btle_SendEventResponse(&rsp, state.btle.isLedConfigCheckPending));

        state.btle.isLedConfigCheckPending = BTLE_CONN_HANDLE_INVALID;
    }
}

static void sensorCalibrationCheck()
{
    if (state.btle.isSensorCalibrationPending != BTLE_CONN_HANDLE_INVALID)
    {
        btle_LcscpEventResponse_t rsp;

        rsp.evt = BTLE_EVT_LCSCP_CALIB_SENS_OFFSET;
        if (ms_CalibrateSensorOffset((ms_accelerationData_t*)&rsp.responseParams.sensOffset) == NRF_SUCCESS)
            rsp.retCode = BTLE_RET_SUCCESS;
        else
            rsp.retCode = BTLE_RET_FAILED;
        APP_ERROR_CHECK(btle_SendEventResponse(&rsp, state.btle.isSensorCalibrationPending));

        state.btle.isSensorCalibrationPending = BTLE_CONN_HANDLE_INVALID;
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    btle_StackInit();
    debug_Init();
    APP_ERROR_CHECK(board_Init());
    hmi_Init();
    state.supplyCellCnt = pwr_Init();
    light_Init(state.supplyCellCnt, &ledConfiguration);
    ms_Init(movementEventHandler);

    btle_lcsFeature_t features;
    features.pitchSupported = 1;
    features.floodSupported = ledConfiguration.floodCount ? 1 : 0;  // supported if unknown
    features.spotSupported = ledConfiguration.spotCount ? 1 : 0;    // supported if unknown
    btle_Init(false, &features, btleEventHandler);

    com_Init();
    cmh_Init(&lightMasterHandler);
#if defined HELENA_DEBUG_FIELD_TESTING
    debug_FieldTestingInit();
#endif
    mainInit();

    while (1)
    {
        const helenaMode_t* pLightMode;
        const light_status_t* pLightStatus;

        // communication check
        com_MessageStruct const* pMessageIn = com_Check();
        if (pMessageIn != NULL) {
            (void)btle_ComGatewayCheck(pMessageIn);             // relay message to bluetooth gateway
            cmh_ComMessageCheck(pMessageIn);                    // process message using the com message handler
        }

        // button checks
        hmi_buttonState_t buttonInt = hmi_Debounce();           // run debounce routine and get internal button state
        processButtons(&buttonInt, &buttonVolUp, &buttonVolDown);  // process button events

        // determine current light mode
        if (state.pMode->currentMode != MODE_OFF)
            pLightMode = &modeConfig.mode[state.pMode->currentMode];
        else
            pLightMode = NULL;

        // check if state has to be changed
        powerStateCheck();

        // check if led config check is pending
        ledConfigCheck();

        // check if sensor calibration is pending
        sensorCalibrationCheck();

        // periodic timebase checks
        if (state.timeoutFlag && (state.power == POWER_IDLE || state.power == POWER_ON))
        {
            ms_data_t msData;

            // get motion sensor data
            uint32_t errCode = ms_GetData(&msData);
            if (errCode != NRF_ERROR_INVALID_DATA)
            {
                APP_ERROR_CHECK(errCode);
            }

            // update light
            light_mode_t lightMode = {{0}};
            if (pLightMode != NULL)
            {
                lightMode.setup.flood = pLightMode->setup.lightFlood;
                lightMode.setup.spot = pLightMode->setup.lightSpot;
                lightMode.setup.pitchCompensation = pLightMode->setup.lightPitchCompensation;
                lightMode.setup.cloned = pLightMode->setup.lightCloned;
                if (lightMode.setup.pitchCompensation)
                    lightMode.illuminanceInLux = pLightMode->intensity;
                else
                    lightMode.intensity = pLightMode->intensity;
            }

            errCode = light_UpdateTargets(&lightMode, msData.pitch, &pLightStatus);
            APP_ERROR_CHECK(errCode);

            // update brake indicator
            brakeDetection(msData.isBraking);

            // update com related message data
            updateLightMsgData(pLightStatus);

            // update ble related message data
            updateLightBtleData(pLightMode, pLightStatus, state.power >= POWER_IDLE ? msData.pitch : INT16_MIN);

            // reset idle timeout counter if moving
            if (msData.isMoving)
                state.idleTimeout = IDLE_TIMEOUT;
        }

        // set status leds
        ledHandling(pLightStatus->flood, pLightStatus->spot, &state.btle);

        light_Execute();
        debug_Execute();
        cmh_Execute();
        pwr_SleepManagement();
    }
}
/* Public functions ----------------------------------------------------------*/

/**END OF FILE*****************************************************************/
