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
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "ble_conn_state.h"
#include "app_error.h"
#include "fds.h"
#include "motion_sensor.h"
#include "custom_board.h"
#include "nrf_delay.h"

/* Private defines -----------------------------------------------------------*/
#define APP_TIMER_OP_QUEUE_SIZE 2           /**< Size of timer operation queues. */

#define MAGICNUMBER             0x83731714  /**< magic number to verify data in no init section */
#define FDSMODECONFIG           0x600D      /**< number identifying mode configuration fds data */
#define FDSINSTANCE             0xCAFE      /**< number identifying fds data for main main module */

#define LEDVOLTAGEFLOOD         6
#define LEDVOLTAGESPOT          3

#define LIGHT_UPDATE_PERIOD_SHORT   (APP_TIMER_TICKS(10, APP_TIMER_PRESCALER))
#define LIGHT_UPDATE_PERIOD_LONG    (APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER))

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    BUTTONMODE0 = 0, BUTTONMODE1, BUTTONMODE2, BUTTONMODE3,
    BUTTONMODE4, BUTTONMODE5, BUTTONMODE6, BUTTONMODE7, BUTTONMODECOM
} lightButtonModes;

typedef enum
{
    COMMODEOFF = 0, COMMODEDIM, COMMODELOW, COMMODEFULL, COMMODECNT
} lightComModes;

typedef struct
{
    lightButtonModes button;
    lightButtonModes lastMode;
    lightComModes com;
    uint32_t magicnumber;
    uint16_t crc;
} lightStateStruct;

typedef struct
{
    uint8_t modeGroups;
    uint8_t padding[3];
    light_ModeStruct modes[BUTTONMODECOM];
} buttonLightModeStruct;

typedef struct
{
    bool isPeriphConnected;
    bool isCentralConnected;
    bool isModeConfigWritePending;
    bool isGroupCountPending;
} btleStatusStruct;

/* Private variables ---------------------------------------------------------*/
static const light_ModeStruct comModes[COMMODECNT] =
{
    {LIGHT_MODEOFF, 0},
    {LIGHT_MODEFLOOD, 10},
    {LIGHT_MODEADAPTIVEBOTH, 12},
    {LIGHT_MODEADAPTIVEFLOOD, 75}
};
static buttonLightModeStruct buttonModes __attribute__ ((aligned (4))) =
{
    .modeGroups = 2,
    .modes =
    {
        {LIGHT_MODEADAPTIVESPOT, 10},
        {LIGHT_MODEADAPTIVESPOT, 30},
        {LIGHT_MODEOFF, 0},
        {LIGHT_MODEOFF, 0},
        {LIGHT_MODESPOT, 35},
        {LIGHT_MODESPOT, 80},
        {LIGHT_MODEOFF, 0},
        {LIGHT_MODEOFF, 0}
    }
};
static lightStateStruct lightState __attribute__((section(".noinit")));
static btleStatusStruct btleStatus;
APP_TIMER_DEF(lightUpdateTimerId);
static uint32_t currentTimebase;
static bool lightUpdateFlag;
static hmi_ButtonEnum buttonInt, buttonVolUp, buttonVolDown;

/* Private macros ------------------------------------------------------------*/
#define COUNT_OF(x)             (sizeof(x)/sizeof(x[0]))

/* Private functions ---------------------------------------------------------*/
static void timerHandler(void* pContext)
{
    (void)pContext;

    lightUpdateFlag = true;
}

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

static bool isGroupConfigValid(uint8_t groupConfig)
{
    for(uint_fast8_t i = 1; i <= BUTTONMODECOM; i <<= 1)
    {
        if (groupConfig == i)
            return true;
    }
    return false;
}

static bool isModeConfigValid(const light_ModeStruct * pMode)
{
    if (pMode->mode < LIGHT_MODECNT)
        return true;
    return false;
}

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

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void mainInit(void)
{
    uint32_t errCode;

    errCode = app_timer_create(&lightUpdateTimerId, APP_TIMER_MODE_REPEATED, timerHandler);
    APP_ERROR_CHECK(errCode);

    // initialize light state if content is not valid
    if (lightState.magicnumber != MAGICNUMBER &&
        lightState.crc != crc16_compute((const uint8_t*)&lightState.magicnumber, sizeof(lightState.magicnumber), NULL))
    {
        lightState.button = BUTTONMODECOM;
        lightState.lastMode = BUTTONMODECOM;
        lightState.com = COMMODEOFF;
        lightState.magicnumber = MAGICNUMBER;
        lightState.crc = crc16_compute((const uint8_t*)&lightState.magicnumber, sizeof(lightState.magicnumber), NULL);
    }

    errCode = fds_register(fdsEventHandler);
    APP_ERROR_CHECK(errCode);
    errCode = fds_init();
    APP_ERROR_CHECK(errCode);

    // load button mode configuration from flash
    loadButtonModeConfig();
}

static void lightMasterHandler(const cmh_LightMasterDataStruct * pMasterData)
{
    lightState.com = pMasterData->helmetBeam;
}

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

static void btleLcscpEventHandler(btle_EventStruct * pEvt)
{
    btle_LcscpEventResponseStruct rsp;
    uint32_t errCode;

    memset(&rsp, 0, sizeof(btle_LcscpEventResponseStruct));
    rsp.evt = pEvt->subEvt.lcscp;

    switch (pEvt->subEvt.lcscp)
    {
    case BTLE_EVT_LCSCP_REQ_MODE_CNT:
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.modeCnt = BUTTONMODECOM;
        break;
    case BTLE_EVT_LCSCP_REQ_GROUP_CNT:
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.groupCnt = buttonModes.modeGroups;
        break;
    case BTLE_EVT_LCSCP_REQ_MODE_CONFIG:
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
    case BTLE_EVT_LCSCP_SET_MODE:
        if (pEvt->lcscpEventParams.modeToSet >= BUTTONMODECOM)
            lightState.button = BUTTONMODECOM;
        else
        {
            lightState.button = pEvt->lcscpEventParams.modeToSet;
            lightState.lastMode = lightState.button;
        }
        rsp.retCode = BTLE_RET_SUCCESS;
        break;
    case BTLE_EVT_LCSCP_CONFIG_MODE:
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
    case BTLE_EVT_LCSCP_CONFIG_GROUP:
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
    default:
        rsp.retCode = BTLE_RET_NOT_SUPPORTED;
        break;
    }
    // if this point is reached, send response immediately, otherwise it is not necessary or will be done later.
    APP_ERROR_CHECK(btle_SendEventResponse(&rsp));
}

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

  /*  btle_LcscpEventResponseStruct rsp;
    uint32_t errCode;

    memset(&rsp, 0, sizeof(btle_LcscpEventResponseStruct));
    rsp.evt = pEvt->evt;

    switch (pEvt->evt)
    {
    case BTLE_EVT_PERIPH_CONNECTED:
        btleStatus.isPeriphConnected = true;
        return;
    case BTLE_EVT_PERIPH_DISCONNECTED:
        btleStatus.isPeriphConnected = false;
        return;
    case BTLE_EVT_CENTRAL_CONNECTED:
        btleStatus.isCentralConnected = true;
        return;
    case BTLE_EVT_CENTRAL_DISCONNECTED:
        btleStatus.isCentralConnected = false;
        return;
    case BTLE_EVT_VOL_UP_SHORT:
        buttonVolUp = hmi_BUTTONSHORT;
        return;
    case BTLE_EVT_VOL_UP_LONG:
        buttonVolUp = hmi_BUTTONLONG;
        return;
    case BTLE_EVT_VOL_DOWN_SHORT:
        buttonVolDown = hmi_BUTTONSHORT;
        return;
    case BTLE_EVT_VOL_DOWN_LONG:
        buttonVolDown = hmi_BUTTONLONG;
        return;
    case BTLE_EVT_REQ_MODE_CNT:
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.modeCnt = BUTTONMODECOM;
        break;
    case BTLE_EVT_REQ_GROUP_CNT:
        rsp.retCode = BTLE_RET_SUCCESS;
        rsp.responseParams.groupCnt = buttonModes.modeGroups;
        break;
    case BTLE_EVT_REQ_MODE_CONFIG:
    {
        uint8_t start = pEvt->eventParams.modeConfigStart;
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
    case BTLE_EVT_SET_MODE:
        if (pEvt->eventParams.modeToSet >= BUTTONMODECOM)
            lightState.button = BUTTONMODECOM;
        else
        {
            lightState.button = pEvt->eventParams.modeToSet;
            lightState.lastMode = lightState.button;
        }
        rsp.retCode = BTLE_RET_SUCCESS;
        break;
    case BTLE_EVT_CONFIG_MODE:
        if (pEvt->eventParams.modeToConfig.modeNumber >= BUTTONMODECOM ||
            !isModeConfigValid((light_ModeStruct*)&pEvt->eventParams.modeToConfig.config))
            rsp.retCode = BTLE_RET_INVALID;
        else
        {
            buttonModes.modes[pEvt->eventParams.modeToConfig.modeNumber].mode = pEvt->eventParams.modeToConfig.config.modeType;
            buttonModes.modes[pEvt->eventParams.modeToConfig.modeNumber].intensity = pEvt->eventParams.modeToConfig.config.intensity;
            errCode = updateButtonModes();
            if (errCode == NRF_SUCCESS)
            {
                btleStatus.isModeConfigWritePending = true;
                return;
            }
            else
                rsp.retCode = BTLE_RET_FAILED;
        }
        break;
    case BTLE_EVT_CONFIG_GROUP:
        if (!isGroupConfigValid(pEvt->eventParams.groupConfig))
            rsp.retCode = BTLE_RET_INVALID;
        else
        {
            buttonModes.modeGroups = pEvt->eventParams.groupConfig;
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
    default:
        rsp.retCode = BTLE_RET_NOT_SUPPORTED;
        break;
    }
    // if this point is reached, send response immediately, otherwise it is not necessary or will be done later.
    APP_ERROR_CHECK(btle_SendEventResponse(&rsp));*/
}

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

static void updateLightBtleData(const light_ModeStruct* pLightMode, const light_StatusStruct * pStatus)
{
    btle_lightDataStruct helmetBeam;

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
    helmetBeam.powerFlood = pStatus->currentFlood * LEDVOLTAGEFLOOD;
    helmetBeam.powerSpot = pStatus->currentSpot * LEDVOLTAGESPOT;
    helmetBeam.temperature = pStatus->temperature / 10 - 273;
    helmetBeam.inputVoltage = pStatus->inputVoltage;

   (void)btle_UpdateLightMeasurements(&helmetBeam);
}

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
        // use last mode if light was off an available
        if (lightState.button == BUTTONMODECOM)
        {
            if (lightState.lastMode != BUTTONMODECOM)
                lightState.button = lightState.lastMode;
            else
                lightState.button = BUTTONMODE0;
        }
        else
        {
            uint_fast8_t modesPerGroup = BUTTONMODECOM / buttonModes.modeGroups;
            do
            {
                if (++lightState.button % modesPerGroup == 0)
                    lightState.button -= modesPerGroup;
            } while (buttonModes.modes[lightState.button].mode == LIGHT_MODEOFF);
            lightState.lastMode = lightState.button;
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
        lightState.lastMode = lightState.button;
    }
}

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

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t errCode;

    // Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    debug_Init();
    APP_ERROR_CHECK(board_Init());
    hmi_Init();
    btle_Init(false, btleEventHandler);
    light_Init();
    ms_Init();
    com_Init();
    cmh_Init(&lightMasterHandler);
#if defined HELENA_DEBUG_FIELD_TESTING
    debug_FieldTestingInit();
#endif
    mainInit();

    sd_clock_hfclk_request();
    currentTimebase = LIGHT_UPDATE_PERIOD_LONG;
    errCode = app_timer_start(lightUpdateTimerId, currentTimebase, NULL);
    APP_ERROR_CHECK(errCode);
    do  // led driver need 250ms to start up, due to bootloader, so just try  again and again.
    {
        errCode = light_Enable(true);
    } while (errCode == NRF_ERROR_INTERNAL);
    APP_ERROR_CHECK(errCode);

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

        // check if timebase has to be changed
        if (pLightMode->mode == LIGHT_MODEOFF && currentTimebase != LIGHT_UPDATE_PERIOD_LONG)
        {
            APP_ERROR_CHECK(app_timer_stop(lightUpdateTimerId));
            lightUpdateFlag = true;
            currentTimebase = LIGHT_UPDATE_PERIOD_LONG;
            APP_ERROR_CHECK(app_timer_start(lightUpdateTimerId, currentTimebase, NULL));
            APP_ERROR_CHECK(ms_Enable(false));
        }
        else if (pLightMode->mode != LIGHT_MODEOFF && currentTimebase != LIGHT_UPDATE_PERIOD_SHORT)
        {
            APP_ERROR_CHECK(app_timer_stop(lightUpdateTimerId));
            lightUpdateFlag = true;
            currentTimebase = LIGHT_UPDATE_PERIOD_SHORT;
            APP_ERROR_CHECK(app_timer_start(lightUpdateTimerId, currentTimebase, NULL));
            APP_ERROR_CHECK(ms_Enable(true));
        }

        if (lightUpdateFlag)
        {
            ms_DataStruct msData;

            lightUpdateFlag = false;
            if (currentTimebase == LIGHT_UPDATE_PERIOD_SHORT)
            {
                errCode = ms_FetchData(&msData);
                if (errCode == NRF_SUCCESS)
                {
                    errCode = light_UpdateTargets(pLightMode, msData.rot.pitch, &pLightStatus);
                    APP_ERROR_CHECK(errCode);
                }
                else if (errCode != NRF_ERROR_INVALID_DATA)
                {
                    APP_ERROR_HANDLER(errCode);
                }

            }
            else
            {
                APP_ERROR_CHECK(light_UpdateTargets(pLightMode, 24576, &pLightStatus));
            }
            ledHandling(pLightStatus->flood, pLightStatus->spot, &btleStatus);
            updateLightMsgData(pLightStatus);
            updateLightBtleData(pLightMode, pLightStatus);
        }

        light_Execute();
        debug_Execute();
        cmh_Execute();
        //app_sched_execute();
        pwr_SleepManagement();
    }
}
/* Public functions ----------------------------------------------------------*/

/**END OF FILE*****************************************************************/
