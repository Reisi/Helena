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
#define APP_TIMER_OP_QUEUE_SIZE 2           /**< Size of timer operation queues. */

#define MAGICNUMBER             0x83731714  /**< magic number to verify data in no init section */
#define FDSMODECONFIG           0x600D      /**< number identifying mode configuration fds data */
#define FDSINSTANCE             0xCAFE      /**< number identifying fds data for main main module */

#define LEDVOLTAGE              3

#define LIGHT_UPDATE_PERIOD_SHORT   (APP_TIMER_TICKS(10, APP_TIMER_PRESCALER))
#define LIGHT_UPDATE_PERIOD_LONG    (APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER))
#define IDLE_TIMEOUT                (APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)/LIGHT_UPDATE_PERIOD_SHORT)

#define MINIMUM_INPUT_VOLTAGE   6000
#define SHUTDOWN_DELAY          (APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER))

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    HELENA_SHUTDOWN = 0, /**< complete shutdown, minimum power consumption, power cycle needed to leave */
    HELENA_OFF,          /**< light and motion processing off, com and hmi working, ble working with low power scanning */
    HELENA_IDLE,         /**< light off but data read out on a regular basis, motion processing on, com and hmi working, ble working with low latency scanning */
    HELENA_ON            /**< light on and data read out on a regular basis, motion processing on, com and hmi working, ble working with low latency scanning */
} helenaStateEnum;

typedef enum
{
    BUTTONMODE0 = 0,
    BUTTONMODE1,
    BUTTONMODE2,
    BUTTONMODE3,
    BUTTONMODE4,
    BUTTONMODE5,
    BUTTONMODE6,
    BUTTONMODE7,
    BUTTONMODECOM
} lightButtonModesEnum;

typedef enum
{
    COMMODEOFF = 0,
    COMMODEDIM,
    COMMODELOW,
    COMMODEFULL,
    COMMODECNT
} lightComModesEnum;

typedef struct
{
    lightButtonModesEnum button;
    lightComModesEnum com;
    uint32_t magicnumber;
    uint16_t crc;
} lightStateStruct;

typedef struct
{
    uint8_t modeGroups;
    uint8_t padding[3];                     /**< padding needed, structure is stored in flash and needs to be word aligned */
    light_ModeStruct modes[BUTTONMODECOM];
} buttonLightModeStruct;

typedef struct
{
    bool isPeriphConnected;                 /**< indicating if in a connection as peripheral */
    bool isCentralConnected;                /**< indicating if in a connection as central */
    bool isModeConfigWritePending;          /**< indicating if a mode configure write response is pending */
    bool isGroupCountPending;               /**< indicating if a group configuration write response is pending */
    bool isLedConfigCheckPending;           /**< indication if a led check procedure was requested */
    bool isSensorCalibrationPending;        /**< indication if sensor offset calibration was requested */
} btleStatusStruct;

/* Private variables ---------------------------------------------------------*/
static const light_ModeStruct comModes[COMMODECNT] =
{
    {LIGHT_MODEOFF, 0},
    {LIGHT_MODEFLOOD, 10},
    {LIGHT_MODEFULLAPC, 12},
    {LIGHT_MODEFLOODAPC, 75}
};
static buttonLightModeStruct buttonModes __attribute__ ((aligned (4))) =
{
    .modeGroups = 2,
    .modes =
    {
        {LIGHT_MODESPOTAPC, 10},
        {LIGHT_MODESPOTAPC, 30},
        {LIGHT_MODEOFF, 0},
        {LIGHT_MODEOFF, 0},
        {LIGHT_MODESPOT, 35},
        {LIGHT_MODESPOT, 80},
        {LIGHT_MODEOFF, 0},
        {LIGHT_MODEOFF, 0}
    }
};
static lightStateStruct lightState __attribute__((section(".noinit"))); /**< actual light state */
static btleStatusStruct btleStatus;                                     /**< actual ble status */
APP_TIMER_DEF(mainTimerId);                                             /**< timer instance for main timer */
static bool timeoutFlag;                                                /**< flag indication main timer timeout */
static uint32_t idleTimeout;                                            /**< idle countdown */
static hmi_ButtonEnum buttonInt, buttonVolUp, buttonVolDown;            /**< button states */
static helenaStateEnum helenaState;                                     /**< helena power state */
static light_DriverConfigStruct ledConfiguration;

/* Private macros ------------------------------------------------------------*/
#define COUNT_OF(x)             (sizeof(x)/sizeof(x[0]))

/* Private functions ---------------------------------------------------------*/

/** @brief event handler for incoming connection related ble events
 */
static void btleConnEvtHandler(btle_EventStruct * pEvt)
{
    switch (pEvt->subEvt.conn)
    {
    case BTLE_EVT_CONN_CENTRAL_CONNECTED:
        btleStatus.isCentralConnected = true;
        break;
    case BTLE_EVT_CONN_CENTRAL_DISCONNECTED:
        btleStatus.isCentralConnected = false;
        break;
    default:
        break;
    }
}

/** @brief event handler for incoming hid related ble events
 */
static void btleHidEvtHandler(btle_EventStruct * pEvt)
{
    switch (pEvt->subEvt.hid)
    {
    case BTLE_EVT_HID_VOL_UP_SHORT:
        buttonVolUp = hmi_BUTTONSHORT;
        break;
    case BTLE_EVT_HID_VOL_UP_LONG:
        buttonVolUp = hmi_BUTTONLONG;
        break;
    case BTLE_EVT_HID_VOL_DOWN_SHORT:
        buttonVolDown = hmi_BUTTONSHORT;
        break;
    case BTLE_EVT_HID_VOL_DOWN_LONG:
        buttonVolDown = hmi_BUTTONLONG;
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
static uint32_t updateButtonModes()
{
    uint32_t errCode;
    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_chunk_t chunk;

    fds_record_key_t key = {.type = FDSMODECONFIG, .instance = FDSINSTANCE};
    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode != NRF_SUCCESS)
        return errCode;
    chunk.p_data = &buttonModes;
    chunk.length_words = SIZEOF_WORDS(buttonModes);
    return fds_update(&descriptor, key, 1, &chunk);
}

/** @brief function to check if configuration is a valid power of 2
 */
static bool isGroupConfigValid(uint8_t groupConfig)
{
    for(uint_fast8_t i = 1; i <= BUTTONMODECOM; i <<= 1)
    {
        if (groupConfig == i)
            return true;
    }
    return false;
}

/** @brief function to check if mode configuration is valid
 *
 * @note for now this function only checks if the mode number is valid, this may
 *       be extended, when light module supports led configuration recognition.
 */
static bool isModeConfigValid(const light_ModeStruct * pMode)
{
    if (pMode->mode < LIGHT_MODECNT)
        return true;
    return false;
}

/** @brief event handler for incoming ble events related to the light control service
 */
static void btleLcscpEventHandler(btle_EventStruct * pEvt)
{
    btle_LcscpEventResponseStruct rsp;
    uint32_t errCode;

    memset(&rsp, 0, sizeof(btle_LcscpEventResponseStruct));
    rsp.evt = pEvt->subEvt.lcscp;

    switch (pEvt->subEvt.lcscp)
    {
    case BTLE_EVT_LCSCP_REQ_MODE_CNT:       // mode count request
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.modeCnt = BUTTONMODECOM;
        break;
    case BTLE_EVT_LCSCP_REQ_GROUP_CNT:      // group count request
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.groupCnt = buttonModes.modeGroups;
        break;
    case BTLE_EVT_LCSCP_REQ_MODE_CONFIG:    // mode configuration request
    {
        uint8_t start = pEvt->lcscpEventParams.modeConfigStart;
        if (start >= BUTTONMODECOM)
        {
            rsp.retCode = BTLE_RET_INVALID;
            break;
        }
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.modeList.pList = (btle_LightModeConfig*)&buttonModes.modes[start];
        rsp.responseParams.modeList.listEntries = BUTTONMODECOM - start;
        break;
    }
    case BTLE_EVT_LCSCP_SET_MODE:           // mode set request
        if (pEvt->lcscpEventParams.modeToSet >= BUTTONMODECOM)
            lightState.button = BUTTONMODECOM;
        else
            lightState.button = pEvt->lcscpEventParams.modeToSet;
        rsp.retCode = BTLE_RET_SUCCESS;
        break;
    case BTLE_EVT_LCSCP_CONFIG_MODE:        // mode configuration request
        for (uint_fast8_t i = 0; i < pEvt->lcscpEventParams.modeToConfig.listEntries; i++)
        {
            if (pEvt->lcscpEventParams.modeToConfig.modeNumber + i >= BUTTONMODECOM ||
                !isModeConfigValid((light_ModeStruct*)&pEvt->lcscpEventParams.modeToConfig.pConfig[i]))
                rsp.retCode = BTLE_RET_INVALID;
        }
        for (uint_fast8_t i = 0; i < pEvt->lcscpEventParams.modeToConfig.listEntries; i++)
        {
            buttonModes.modes[pEvt->lcscpEventParams.modeToConfig.modeNumber + i].mode = pEvt->lcscpEventParams.modeToConfig.pConfig[i].modeType;
            buttonModes.modes[pEvt->lcscpEventParams.modeToConfig.modeNumber + i].intensity = pEvt->lcscpEventParams.modeToConfig.pConfig[i].intensity;
        }
        errCode = updateButtonModes();
        if (errCode == NRF_SUCCESS)
        {
            btleStatus.isModeConfigWritePending = true;
            return;
        }
        else
            rsp.retCode = BTLE_RET_FAILED;
        break;
    case BTLE_EVT_LCSCP_CONFIG_GROUP:       // group configuration request
        if (!isGroupConfigValid(pEvt->lcscpEventParams.groupConfig))
            rsp.retCode = BTLE_RET_INVALID;
        else
        {
            buttonModes.modeGroups = pEvt->lcscpEventParams.groupConfig;
            errCode = updateButtonModes();
            if (errCode == NRF_SUCCESS)
            {
                btleStatus.isGroupCountPending = true;
                return;
            }
            else
                rsp.retCode = BTLE_RET_FAILED;
        }
        break;
    case BTLE_EVT_LCSCP_REQ_LED_CONFIG:
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.ledConfig.floodCnt = ledConfiguration.floodCount;
        rsp.responseParams.ledConfig.spotCnt = ledConfiguration.spotCount;
        break;
    case BTLE_EVT_LCSCP_CHECK_LED_CONFIG:
        if (helenaState != HELENA_ON)
        {
            btleStatus.isLedConfigCheckPending = true;
            return;
        }
        else
            rsp.retCode = BTLE_RET_INVALID;
    case BTLE_EVT_LCSCP_REQ_SENS_OFFSET:
        if (ms_GetSensorOffset((ms_AccelerationStruct*)&rsp.responseParams.sensOffset) == NRF_SUCCESS)
            rsp.retCode = BTLE_RET_SUCCESS;
        else
            rsp.retCode = BTLE_RET_INVALID;
        break;
    case BTLE_EVT_LCSCP_CALIB_SENS_OFFSET:
        btleStatus.isSensorCalibrationPending = true;
        return;
    default:
        break;
    }
    // if this point is reached, send response immediately, otherwise it is not necessary or will be done later.
    APP_ERROR_CHECK(btle_SendEventResponse(&rsp));
}

/** @brief event handler for incoming ble events
 *
 * @details this function relays the incoming events to the corresponding
 *          event handlers
 */
static void btleEventHandler(btle_EventStruct * pEvt)
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
static void lightMasterHandler(const cmh_LightMasterDataStruct * pMasterData)
{
    lightState.com = pMasterData->helmetBeam;
}

/** @brief main timer event handler
 *
 * @details this function just sets a flag, corresponding actions are done in main loop
 */
static void timerHandler(void* pContext)
{
    (void)pContext;

    if (idleTimeout)
    {
        idleTimeout--;
        timeoutFlag = true;
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
            if (btleStatus.isGroupCountPending || btleStatus.isModeConfigWritePending)
            {
                btle_LcscpEventResponseStruct rsp;
                if (btleStatus.isGroupCountPending)
                {
                    rsp.evt = BTLE_EVT_LCSCP_CONFIG_GROUP;
                    btleStatus.isGroupCountPending = false;
                }
                if (btleStatus.isModeConfigWritePending)
                {
                    rsp.evt = BTLE_EVT_LCSCP_CONFIG_MODE;
                    btleStatus.isModeConfigWritePending = false;
                }
                if (errCode == NRF_SUCCESS)
                    rsp.retCode = BTLE_RET_SUCCESS;
                else
                    rsp.retCode = BTLE_RET_FAILED;
                APP_ERROR_CHECK(btle_SendEventResponse(&rsp));
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
        else
            APP_ERROR_CHECK(errCode);
    }
}

/** @brief function to load the button mode configuration
 *
 * @details this function tries to load previous configuration out of flash memory,
 *          if no configuration is stored yet or if configuration is not valid,
 *          it loads and stores the default configuration
 */
static void loadButtonModeConfig()
{
    uint32_t errCode;

    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_key_t key = {.type = FDSMODECONFIG, .instance = FDSINSTANCE};

    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode == NRF_SUCCESS)
    {
        // check if data is valid
        fds_record_t record;
        buttonLightModeStruct * pData;
        errCode = fds_open(&descriptor, &record);
        if (errCode == NRF_SUCCESS)
        {
            bool dataValid = true;
            pData = (buttonLightModeStruct*)record.p_data;
            if (!isGroupConfigValid(pData->modeGroups))
                dataValid = false;
            for (uint_fast8_t i = 0; i < BUTTONMODECOM; i++)
            {
                if (!isModeConfigValid(&pData->modes[i]))
                {
                    dataValid = false;
                    break;
                }
            }
            // if data is valid, copy configuration
            if (dataValid)
                buttonModes = *pData;
            errCode = fds_close(&descriptor);
            APP_ERROR_CHECK(errCode);
            // if data is not valid, update it with defaults values
            if (!dataValid)
            {
                fds_record_chunk_t chunk;
                chunk.p_data = &buttonModes;
                chunk.length_words = SIZEOF_WORDS(buttonModes);
                errCode = fds_update(&descriptor, key, 1, &chunk);
                APP_ERROR_CHECK(errCode);
            }
        }
        else
        {
            APP_ERROR_HANDLER(errCode);
        }
    }
    else if (errCode == NRF_ERROR_NOT_FOUND)
    {
        fds_record_chunk_t chunk;
        chunk.p_data = &buttonModes;
        chunk.length_words = SIZEOF_WORDS(buttonModes);
        errCode = fds_write(&descriptor, key, 1, &chunk);
        APP_ERROR_CHECK(errCode);
    }
    else
    {
        APP_ERROR_HANDLER(errCode);
    }
}

/** @brief function to change helena power state
 */
static uint32_t setHelenaState(helenaStateEnum newState)
{
    if (helenaState == newState)
        return NRF_SUCCESS;

    switch (newState)
    {
    case HELENA_SHUTDOWN:
        // shut down everything, there will be no return from this state, power cycle needed
        if (helenaState >= HELENA_IDLE)
        {
            APP_ERROR_CHECK(light_Enable(false));
            APP_ERROR_CHECK(ms_Enable(false));
        }
        hmi_SetLed(hmi_LEDBLUE, hmi_LEDOFF);
        hmi_SetLed(hmi_LEDRED, hmi_LEDOFF);
        APP_ERROR_CHECK(app_timer_stop(mainTimerId));
        nrf_gpio_cfg_default(pBoardConfig->bdepRX); // default config for com pin and button
        nrf_gpio_cfg_default(pBoardConfig->button); // to disable DETECT signal
        APP_ERROR_CHECK(sd_power_system_off());
        break;
    case HELENA_OFF:
        // shut down light and motion sensor
        APP_ERROR_CHECK(light_Enable(false));
        APP_ERROR_CHECK(ms_Enable(false));
        btle_SetScanConfig(BTLE_SCAN_MODE_LOW_POWER);
        (void)app_timer_stop(mainTimerId);
        APP_ERROR_CHECK(app_timer_start(mainTimerId, LIGHT_UPDATE_PERIOD_LONG, NULL));  // used to keep timer running, otherwise RTC will be stopped.
        break;
    case HELENA_IDLE:
        // if helena was on, nothing need to be changed, otherwise timer needs to be started
        if (helenaState != HELENA_ON)
        {
            APP_ERROR_CHECK(light_Enable(true));
            APP_ERROR_CHECK(ms_Enable(true));
            btle_SetScanConfig(BTLE_SCAN_MODE_LOW_LATENCY);
            (void)app_timer_stop(mainTimerId);
            APP_ERROR_CHECK(app_timer_start(mainTimerId, LIGHT_UPDATE_PERIOD_SHORT, NULL));
        }
        timeoutFlag = true;
        break;
    case HELENA_ON:
        // if helena was in idle mode, nothing has to be changed, otherwise everything needs to be started
        if (helenaState != HELENA_IDLE)
        {
            APP_ERROR_CHECK(light_Enable(true));
            APP_ERROR_CHECK(ms_Enable(true));
            btle_SetScanConfig(BTLE_SCAN_MODE_LOW_LATENCY);
            (void)app_timer_stop(mainTimerId);
            APP_ERROR_CHECK(app_timer_start(mainTimerId, LIGHT_UPDATE_PERIOD_SHORT, NULL));
        }
        timeoutFlag = true;
        break;
    }

    helenaState = newState;

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

    // initialize light state if content is not valid
    if (lightState.magicnumber != MAGICNUMBER &&
        lightState.crc != crc16_compute((const uint8_t*)&lightState.magicnumber, sizeof(lightState.magicnumber), NULL))
    {
        lightState.button = BUTTONMODECOM;
        lightState.com = COMMODEOFF;
        lightState.magicnumber = MAGICNUMBER;
        lightState.crc = crc16_compute((const uint8_t*)&lightState.magicnumber, sizeof(lightState.magicnumber), NULL);
    }

    // register to fds module
    errCode = fds_register(fdsEventHandler);
    APP_ERROR_CHECK(errCode);
    errCode = fds_init();
    APP_ERROR_CHECK(errCode);

    // load button mode configuration from flash
    loadButtonModeConfig();

    APP_ERROR_CHECK(setHelenaState(HELENA_IDLE));
    idleTimeout = IDLE_TIMEOUT;
}

/** @brief function for handling button events
 */
static void buttonHandling(hmi_ButtonEnum internal, hmi_ButtonEnum volumeUp, hmi_ButtonEnum volumeDown)
{
    // ultra long press while off -> start searching for remote
    if (lightState.button == BUTTONMODECOM && lightState.com == COMMODEOFF && internal == hmi_BUTTONULTRALONG)
    {
        APP_ERROR_CHECK(btle_DeleteBonds());
        APP_ERROR_CHECK(btle_SearchForRemote());
    }

    // ultra long press or volume down button while light is in button mode and on -> turn off
    if (lightState.button != BUTTONMODECOM && (volumeDown != hmi_BUTTONNOPRESS || internal == hmi_BUTTONULTRALONG))
    {
        lightState.button = BUTTONMODECOM;
        lightState.com = COMMODEOFF;
    }

    // short press -> increase to next state
    if (internal == hmi_BUTTONSHORT || volumeUp == hmi_BUTTONSHORT)
    {
        if (lightState.button == BUTTONMODECOM)
            lightState.button = BUTTONMODE0;
        else
        {
            uint_fast8_t modesPerGroup = BUTTONMODECOM / buttonModes.modeGroups;
            do
            {
                if (++lightState.button % modesPerGroup == 0)
                    lightState.button -= modesPerGroup;
            } while (buttonModes.modes[lightState.button].mode == LIGHT_MODEOFF);
        }
    }

    // long press while light is not off -> increase to next group
    if (lightState.button != BUTTONMODECOM && (internal == hmi_BUTTONLONG || volumeUp == hmi_BUTTONLONG))
    {
        uint_fast8_t modesPerGroup = BUTTONMODECOM / buttonModes.modeGroups;
        do
        {
            lightState.button += modesPerGroup;
            lightState.button %= BUTTONMODECOM;
            for (uint_fast8_t i = 0; i < modesPerGroup; i++)
            {
                if (buttonModes.modes[lightState.button].mode == LIGHT_MODEOFF)
                {
                    if (++lightState.button % modesPerGroup == 0)
                        lightState.button -= modesPerGroup;
                }
                else
                    break;
            }
        } while (buttonModes.modes[lightState.button].mode == LIGHT_MODEOFF);
    }
}

/** @brief function to handle the status led
 */
static void ledHandling(light_Status flood, light_Status spot, btleStatusStruct *pBtleData)
{
    // mask out duty-cycle and overcurrent flags
    flood &= LIGHT_STATUS_VOLTAGELIMIT | LIGHT_STATUS_TEMPERATURELIMIT;
    spot &= LIGHT_STATUS_VOLTAGELIMIT | LIGHT_STATUS_TEMPERATURELIMIT;

    if (flood | spot)
        hmi_SetLed(hmi_LEDRED, hmi_LEDON);
    else
        hmi_SetLed(hmi_LEDRED, hmi_LEDOFF);
    if (pBtleData->isCentralConnected)
        hmi_SetLed(hmi_LEDBLUE, hmi_LEDON);
    else
        hmi_SetLed(hmi_LEDBLUE, hmi_LEDOFF);
}

/** @brief function to update the light information data for the com module
 */
static void updateLightMsgData(const light_StatusStruct * pStatus)
{
    cmh_HelmetLightStruct helmetBeam;
    light_Status lightStatus;

    memset(&helmetBeam, 0, sizeof(cmh_HelmetLightStruct));
    lightStatus = pStatus->flood | pStatus->spot;
    if (lightStatus & LIGHT_STATUS_OVERCURRENT)
        helmetBeam.overcurrentError = 1;
    if (lightStatus & LIGHT_STATUS_VOLTAGELIMIT)
        helmetBeam.voltageError = 1;
    if (lightStatus & LIGHT_STATUS_TEMPERATURELIMIT)
        helmetBeam.temperatureError = 1;
    if (lightStatus & LIGHT_STATUS_DUTYCYCLELIMIT)
        helmetBeam.directdriveError = 1;
    helmetBeam.mode = lightState.com;
    helmetBeam.current = pStatus->currentFlood + pStatus->currentSpot;
    helmetBeam.temperature = pStatus->temperature;
    helmetBeam.voltage = pStatus->inputVoltage;

    // ignore error codes
    (void)cmh_UpdateHelmetLight(&helmetBeam);
}

/** @brief function to update the light informations for the ble module
 */
static void updateLightBtleData(const light_ModeStruct* pLightMode, const light_StatusStruct * pStatus)
{
    btle_lightDataStruct helmetBeam;
    uint16_t powerFlood, powerSpot;

    powerFlood = pStatus->currentFlood * LEDVOLTAGE * ledConfiguration.floodCount;
    powerSpot = pStatus->currentSpot * LEDVOLTAGE * ledConfiguration.spotCount;

    helmetBeam.mode = pLightMode->mode;
    helmetBeam.intensity = pLightMode->intensity;
    helmetBeam.statusFlood.overcurrent = pStatus->flood & LIGHT_STATUS_OVERCURRENT ? 1 : 0;
    helmetBeam.statusFlood.inputVoltage = pStatus->flood & LIGHT_STATUS_VOLTAGELIMIT ? 1 : 0;
    helmetBeam.statusFlood.temperature = pStatus->flood & LIGHT_STATUS_TEMPERATURELIMIT ? 1 : 0;
    helmetBeam.statusFlood.dutyCycleLimit = pStatus->flood & LIGHT_STATUS_DUTYCYCLELIMIT ? 1 : 0;
    helmetBeam.statusSpot.overcurrent = pStatus->spot & LIGHT_STATUS_OVERCURRENT ? 1 : 0;
    helmetBeam.statusSpot.inputVoltage = pStatus->spot & LIGHT_STATUS_VOLTAGELIMIT ? 1 : 0;
    helmetBeam.statusSpot.temperature = pStatus->spot & LIGHT_STATUS_TEMPERATURELIMIT ? 1 : 0;
    helmetBeam.statusSpot.dutyCycleLimit = pStatus->spot & LIGHT_STATUS_DUTYCYCLELIMIT ? 1 : 0;
    helmetBeam.temperature = pStatus->temperature / 10 - 273;
    helmetBeam.inputVoltage = pStatus->inputVoltage;
    helmetBeam.powerFlood = powerFlood;
    helmetBeam.powerSpot = powerSpot;
    if (pLightMode->mode == LIGHT_MODEFLOODAPCCLONED || pLightMode->mode == LIGHT_MODEFLOODCLONED)
        helmetBeam.powerFlood += powerSpot;
    if (pLightMode->mode == LIGHT_MODESPOTAPCCLONED || pLightMode->mode == LIGHT_MODESPOTCLONED)
        helmetBeam.powerSpot += powerFlood;

   (void)btle_UpdateLightMeasurements(&helmetBeam);
}

static void brakeDetection(bool indicate)
{
    if (indicate)
        cmh_UpdateBrakeIndicator(true);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t errCode;
    btle_LightFeatureStruct features;

    // Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    btle_StackInit();
    debug_Init();
    APP_ERROR_CHECK(board_Init());
    hmi_Init();
    pwr_Init();
    // wait for led driver module to leave bootloader and start up
    nrf_delay_ms(250);
    light_Init(&ledConfiguration);
    ms_Init();
    features.pitchSupported = 1;
    features.floodSupported = ledConfiguration.floodCount ? 1 : 0;
    features.spotSupported = ledConfiguration.spotCount ? 1 : 0;
    btle_Init(false, &features, btleEventHandler);
    com_Init();
    cmh_Init(&lightMasterHandler);
#if defined HELENA_DEBUG_FIELD_TESTING
    debug_FieldTestingInit();
#endif
    mainInit();

    while (1)
    {
        const com_MessageStruct *pMessageIn;
        const light_ModeStruct* pLightMode;
        const light_StatusStruct* pLightStatus;

        // communication check
        pMessageIn = com_Check();
        if (pMessageIn != NULL) {
            (void)btle_ComGatewayCheck(pMessageIn);
            cmh_ComMessageCheck(pMessageIn);
        }

        // button checks
        buttonInt = hmi_Debounce();
        buttonHandling(buttonInt, buttonVolUp, buttonVolDown);
        buttonVolUp = hmi_BUTTONNOPRESS;
        buttonVolDown = hmi_BUTTONNOPRESS;

        // determine actual light mode
        if (lightState.button == BUTTONMODECOM)
            pLightMode = &comModes[lightState.com];
        else
            pLightMode = &buttonModes.modes[lightState.button];

        // check if state has to be changed
        if (pLightMode->mode == LIGHT_MODEOFF && helenaState == HELENA_ON)
        {
            APP_ERROR_CHECK(setHelenaState(HELENA_IDLE));
            idleTimeout = IDLE_TIMEOUT;
        }
        if (pLightMode->mode != LIGHT_MODEOFF && helenaState != HELENA_ON)
        {
            APP_ERROR_CHECK(setHelenaState(HELENA_ON));
            idleTimeout = IDLE_TIMEOUT;
        }
        if (idleTimeout == 0 && helenaState != HELENA_OFF)
        {
            APP_ERROR_CHECK(setHelenaState(HELENA_OFF));
        }
        if (helenaState != HELENA_OFF)
        {
            static pwr_VoltageStruct voltageLast;
            pwr_VoltageStruct voltageNow;
            errCode = pwr_GetInputVoltage(&voltageNow);
            if (errCode == NRF_SUCCESS && voltageNow.timestamp != 0)
            {                                                           // perform checks only if
                if (voltageLast.timestamp != 0 &&                       // valid values are available and
                    voltageLast.inputVoltage < MINIMUM_INPUT_VOLTAGE && // both value (actual and last stored)
                    voltageNow.inputVoltage < MINIMUM_INPUT_VOLTAGE)    // are below threshold
                {
                    uint32_t timediff;
                    (void)app_timer_cnt_diff_compute(voltageNow.timestamp, voltageLast.timestamp, &timediff);
                    if (timediff > SHUTDOWN_DELAY)                      // shutdown helena if input voltage has been
                        APP_ERROR_CHECK(setHelenaState(HELENA_SHUTDOWN));//below threshold for specific time period
                }
                                                                        // store actual values if
                if (voltageNow.inputVoltage >= MINIMUM_INPUT_VOLTAGE || // input voltage is above threshold or
                    voltageLast.timestamp == 0 ||                       // no valid values stored yet or
                    voltageLast.inputVoltage >= MINIMUM_INPUT_VOLTAGE)  // last values was above threshold
                    voltageLast = voltageNow;
            }
        }

        // check if led config check is pending
        if (btleStatus.isLedConfigCheckPending)
        {
            btle_LcscpEventResponseStruct rsp;

            btleStatus.isLedConfigCheckPending = false;
            rsp.evt = BTLE_EVT_LCSCP_CHECK_LED_CONFIG;
            if (light_CheckLedConfig(&ledConfiguration) == NRF_SUCCESS)
                rsp.retCode = BTLE_RET_SUCCESS;
            else
                rsp.retCode = BTLE_RET_FAILED;
            rsp.responseParams.ledConfig.floodCnt = ledConfiguration.floodCount;
            rsp.responseParams.ledConfig.spotCnt = ledConfiguration.spotCount;
            APP_ERROR_CHECK(btle_SendEventResponse(&rsp));
        }

        // check if sensor calibration is pending
        if (btleStatus.isSensorCalibrationPending)
        {
            btle_LcscpEventResponseStruct rsp;

            btleStatus.isSensorCalibrationPending = false;
            rsp.evt = BTLE_EVT_LCSCP_CALIB_SENS_OFFSET;
            if (ms_CalibrateSensorOffset((ms_AccelerationStruct*)&rsp.responseParams.sensOffset) == NRF_SUCCESS)
                rsp.retCode = BTLE_RET_SUCCESS;
            else
                rsp.retCode = BTLE_RET_FAILED;
            APP_ERROR_CHECK(btle_SendEventResponse(&rsp));
        }

        // periodic timebase checks
        if (timeoutFlag && (helenaState == HELENA_IDLE || helenaState == HELENA_ON))
        {
            ms_DataStruct msData;

            // get motion sensor data
            errCode = ms_FetchData(&msData);
            if (errCode != NRF_ERROR_INVALID_DATA)
            {
                APP_ERROR_CHECK(errCode);
            }

            // update light
            errCode = light_UpdateTargets(pLightMode, msData.pitch, &pLightStatus);
            APP_ERROR_CHECK(errCode);

            // update brake indicator
            brakeDetection(msData.isBraking);
            // update com related message data
            updateLightMsgData(pLightStatus);
            // update ble related message data
            updateLightBtleData(pLightMode, pLightStatus);

            // reset idle timeout counter if necessary
            if (pLightMode->mode != LIGHT_MODEOFF || msData.isMoving)
                idleTimeout = IDLE_TIMEOUT;
            if (msData.isMoving)
                idleTimeout = IDLE_TIMEOUT;
        }

        // set status leds
        ledHandling(pLightStatus->flood, pLightStatus->spot, &btleStatus);

        light_Execute();
        debug_Execute();
        cmh_Execute();
        pwr_SleepManagement();
    }
}
/* Public functions ----------------------------------------------------------*/

/**END OF FILE*****************************************************************/
