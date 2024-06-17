/**
  ******************************************************************************
  * @file    main.c
  * @author  Thomas Reisnecker
  * @brief   main module for Helena
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define MAIN_LOG_ENABLED

#ifdef MAIN_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // MAIN_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "debug.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "btle.h"
#include "link_management.h"
#include "mode_management.h"
#include "app_timer.h"
#include "hmi.h"
#include "string.h"
#include "board.h"
#include "com_message_handling.h"

/* Private defines -----------------------------------------------------------*/
#define APP_TIMER_PRESCALER     MAIN_TIMER_PRESCALER
#define APP_TIMER_OP_QUEUE_SIZE 2

#define IDLE_TIMEOUT            MAIN_TIMER_TICKS(180000)
#define IDLE_PERIOD             MAIN_TIMER_TICKS(10000)

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    MAINSTATE_OFF,      // device enters system off mode
    MAINSTATE_STANDBY,
    MAINSTATE_IDLE
} mState_t;

typedef enum
{
    FLOPSCR_NONE,   // no source
    FLOPSCR_HMI,    // flash operation request from hmi module
    /// TODO: why is there nothing from Helen Project Service?
} pendingFlashOpSource_t;

typedef struct
{
    volatile uint8_t pendingResponses;  // the number of responses to be expected
    pendingFlashOpSource_t source;
    uint32_t firstErrorCode;
    union
    {
        ds_reportHandler_t hmiResponse; // response handler for events related to the hmi module
    };
} pendingFlashOp_t;

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)           (sizeof(x)/sizeof(x[0]))
#define SYSEVENT_CNT()          NRF_SECTION_VARS_COUNT(main_SysEventHandler_t, sys_events)
#define SYSEVENT_GET(cnt)       NRF_SECTION_VARS_GET(cnt, main_SysEventHandler_t, sys_events)

/* Private variables ---------------------------------------------------------*/
static mState_t mainState;
static uint32_t idleCounter;
APP_TIMER_DEF(idleTimer);
static pendingFlashOp_t pendingFlashOp;

/* Private read only variables -----------------------------------------------*/
/// TODO: this must be configured at compile time
static nrf_clock_lf_cfg_t const clockConfig =
{
    .source = NRF_CLOCK_LF_SRC_RC,
    .rc_ctiv = 16,
    .rc_temp_ctiv = 1
};

/* Private functions ---------------------------------------------------------*/
static void modeChanged(uint8_t newMode)
{
    //LOG_INFO("[main]: new mode %d", newMode);

    if (newMode == MM_MODE_OFF)
        newMode = mm_GetOffMode();
    ret_code_t errCode = brd_SetLightMode(newMode);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("setting light mode error %d", errCode);
    }
}

static void sysEventDispatch(uint32_t sysEvt)
{
    for (uint_fast8_t i = 0; i < SYSEVENT_CNT(); i++)
    {
        main_SysEventHandler_t const* pHandler = SYSEVENT_GET(i);
        if (pHandler->pHandler != NULL)
            pHandler->pHandler(sysEvt);
    }
}

void idleTimerCallback(void * pContext)
{
    (void)pContext;

    if (idleCounter == 0 || (--idleCounter != 0))
        return;

    mainState = MAINSTATE_STANDBY;
    LOG_INFO("[main]: going to standby mode");

    ret_code_t errCode;
    errCode = lm_SetExposureMode(LM_EXP_LOW_POWER);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[main]: exposure mode set error %d", errCode);
    }

    errCode = brd_SetPowerMode(BRD_PM_STANDBY);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[main]: board power mode set error %d", errCode);
    }
}

static ret_code_t timerInit()
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    ret_code_t errCode = app_timer_create(&idleTimer, APP_TIMER_MODE_REPEATED, &idleTimerCallback);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[main]: idle timer create error %d", errCode);
        return errCode;
    }

    mainState = MAINSTATE_IDLE;
    idleCounter = IDLE_TIMEOUT / IDLE_PERIOD;
    errCode = app_timer_start(idleTimer, IDLE_PERIOD, NULL);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[main]: starting idle timer error %d", errCode);
    }

    return errCode;
}

static ret_code_t sdhInit(void)
{
    nrf_clock_lf_cfg_t clockConfig = CLOCK_CONFIG;

    SOFTDEVICE_HANDLER_INIT(&clockConfig, NULL);

    ret_code_t errCode = softdevice_sys_evt_handler_set(sysEventDispatch);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[main]: error %d on setting system event handler", errCode);
    }

    return errCode;
}

static void flashOpResultHandler(ret_code_t errCode)
{
    if (pendingFlashOp.pendingResponses == 0)
        return; // not expecting anything

    // preserve the first error code
    if (pendingFlashOp.firstErrorCode == NRF_SUCCESS)
        pendingFlashOp.firstErrorCode = errCode;

    if (--pendingFlashOp.pendingResponses == 0)
    {
        switch (pendingFlashOp.source)
        {
        case FLOPSCR_HMI:
            if (pendingFlashOp.hmiResponse != NULL)
                pendingFlashOp.hmiResponse(pendingFlashOp.firstErrorCode);
            break;

        default:
            break;
        }

        memset(&pendingFlashOp, 0, sizeof(pendingFlashOp));
    }
}

static bool isBlockZero(void const* pBlock, unsigned int size)
{
    unsigned char* pMem = (unsigned char*)pBlock;
    for (unsigned int i = 0; i < size; i++)
    {
        if(pMem[i] != 0)
            return false;
    }
    return true;
}

ret_code_t hmiEventHandler(hmi_evt_t const* pEvent)
{
    ret_code_t errCode = NRF_SUCCESS;
    (void)main_ResetIdleTimer();

    switch (pEvent->type)
    {
    case HMI_EVT_SEARCHREMOTE:
        LOG_INFO("[main]: searching remote");
        errCode = lm_SetExposureMode(LM_EXP_SEARCHING);
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[main]: setting exposure mode error %d", errCode);
        }
        break;

    case HMI_EVT_FACTORYRESET:
        LOG_INFO("[main]: initiating factory reset");
        if (pendingFlashOp.pendingResponses)
            return NRF_ERROR_INVALID_STATE;

        pendingFlashOp.pendingResponses = 3;
        pendingFlashOp.source = FLOPSCR_HMI;
        pendingFlashOp.hmiResponse = pEvent->params.resultHandler;

        errCode = mm_FactoryReset(flashOpResultHandler);
        if (errCode != NRF_SUCCESS)
        {
            pendingFlashOp.pendingResponses--;
            if (pendingFlashOp.firstErrorCode != NRF_SUCCESS)
                pendingFlashOp.firstErrorCode = errCode;
            LOG_ERROR("[main]: mode factory reset error %d", errCode);
        }

        errCode = brd_FactoryReset(flashOpResultHandler);
        if (errCode != NRF_SUCCESS)
        {
            pendingFlashOp.pendingResponses--;
            if (pendingFlashOp.firstErrorCode != NRF_SUCCESS)
                pendingFlashOp.firstErrorCode = errCode;
            LOG_ERROR("[main]: board factory reset error %d", errCode);
        }

        errCode = lm_DeleteBonds(flashOpResultHandler);
        if (errCode != NRF_SUCCESS)
        {
            pendingFlashOp.pendingResponses--;
            if (pendingFlashOp.firstErrorCode != NRF_SUCCESS)
                pendingFlashOp.firstErrorCode = errCode;
            LOG_ERROR("[main]: deleting bonds error %d", errCode);
        }
        // if all requests failed, result might to be sent manually
        if (pendingFlashOp.hmiResponse == 0 && !isBlockZero(&pendingFlashOp, sizeof(pendingFlashOp)))
        {
            pEvent->params.resultHandler(pendingFlashOp.firstErrorCode);
            memset(&pendingFlashOp, 0, sizeof(pendingFlashOp));
        }
        break;

    case HMI_EVT_DELETEBONDS:
        LOG_INFO("[main]: deleting bonds");
        if (pendingFlashOp.pendingResponses)
            return NRF_ERROR_INVALID_STATE;

        pendingFlashOp.pendingResponses = 1;
        pendingFlashOp.source = FLOPSCR_HMI;
        pendingFlashOp.hmiResponse = pEvent->params.resultHandler;

        errCode = lm_DeleteBonds(flashOpResultHandler);
        if (errCode != NRF_SUCCESS)
        {
            pendingFlashOp.pendingResponses--;
            if (pendingFlashOp.firstErrorCode != NRF_SUCCESS)
                pendingFlashOp.firstErrorCode = errCode;
            LOG_ERROR("[main]: deleting bonds error %d", errCode);
        }
        // if request failed, result might to be sent manually
        if (pendingFlashOp.hmiResponse == 0 && !isBlockZero(&pendingFlashOp, sizeof(pendingFlashOp)))
        {
            pEvent->params.resultHandler(pendingFlashOp.firstErrorCode);
            memset(&pendingFlashOp, 0, sizeof(pendingFlashOp));
        }
        break;

    default:    // the other events are handled by the hmi module
        return mm_HmiEventHandler(pEvent);
    }

    return errCode;
}

static bool hpsEventHandler(ble_hps_evt_t const * pEvt)
{
    if (pEvt->evt_type == BLE_HPS_EVT_MODE_CONFIG_CHANGED)
    {
        LOG_INFO("[main]: new mode config recieved");

        ret_code_t errCode;
        uint16_t size;

        size = (uint32_t)pEvt->params.modes.p_channel_config - (uint32_t)pEvt->params.modes.p_mode_config;

        errCode = mm_CheckModeConfig((mm_modeConfig_t const*)(pEvt->params.modes.p_mode_config), size);
        if (errCode != NRF_SUCCESS)
            return false;

        errCode = brd_SetChannelConfig(pEvt->params.modes.p_channel_config, pEvt->params.modes.total_size - size, NULL);
        if (errCode == NRF_ERROR_INVALID_LENGTH || errCode == NRF_ERROR_INVALID_PARAM)
            return false;

        if (errCode != NRF_SUCCESS)
            LOG_ERROR("[main]: setting channel config error %d", errCode);

        errCode = mm_SetModeConfig((mm_modeConfig_t const*)pEvt->params.modes.p_mode_config, size, NULL);
        if (errCode != NRF_SUCCESS)
            LOG_ERROR("[main]: setting mode config error %d", errCode);
    }
    else if (pEvt->evt_type == BLE_HPS_EVT_CP_WRITE)
    {
        hmi_evtParams_t hmiParam;

        switch (pEvt->params.ctrl_pt.opcode)
        {
        /*case BLE_HPS_CP_REQ_MODE:
            break;*/

        case BLE_HPS_CP_SET_MODE:
            hmiParam.mode.modeNumber = pEvt->params.ctrl_pt.mode_to_set;
            hmiParam.mode.connHandle = pEvt->p_client->conn_handle;
            (void)hmi_RequestEvent(HMI_EVT_MODENUMBER, &hmiParam);
            break;

        case BLE_HPS_CP_REQ_SEARCH:
            (void)hmi_RequestEvent(HMI_EVT_SEARCHREMOTE, NULL);
            break;

        case BLE_HPS_CP_REQ_FACTORY_RESET:
            hmiParam.resultHandler = NULL;
            (void)hmi_RequestEvent(HMI_EVT_FACTORYRESET, &hmiParam);
            break;

        case BLE_HPS_CP_SET_MODE_OVERRIDE:
        {
            ret_code_t errCode = brd_OverrideMode(&pEvt->params.ctrl_pt.channel_config);
            return errCode == NRF_SUCCESS;
        }

        default:
            return false;
        }
    }
    return true;
}

static void btleEventHandler(btle_event_t const * pEvt)
{
    if (pEvt->type == BTLE_EVT_NAME_CHANGE)
    {
        LOG_INFO("[main]: new device name received: %s", pEvt->pNewDeviceName);

        ret_code_t errCode;
        errCode = brd_SetDeviceName(pEvt->pNewDeviceName, NULL);
        APP_ERROR_CHECK(errCode);
    }
}

static ret_code_t bluetoothInit(brd_info_t const* pBrdInfo)
{
    ble_hps_init_t hpsInit = {{0}};

    //ble_hps_f_ch_size_t chSize[pBrdInfo->pFeatures->channelCount];
    hpsInit.features.mode_count = MM_NUM_OF_MODES;
    hpsInit.features.channel_count = pBrdInfo->pFeatures->channelCount;
    /// TODO this cannot be hardcoded here, it should be shifted to board specific code
    /*for (uint8_t i = 0; i < ARRAY_SIZE(chSize); i++)
    {
        chSize[i].channel_bitsize = 8;
        chSize[i].sp_ftr_bitsize = 3;
        chSize[i].channel_description = pBrdInfo->pFeatures->pChannelTypes[i];
    }*/
    hpsInit.features.p_channel_size = pBrdInfo->pFeatures->pChannelTypes;//chSize;
    hpsInit.features.flags.mode_set_supported = true;
    hpsInit.features.flags.search_request_supported = true;
    hpsInit.features.flags.factory_reset_supported = true;
    hpsInit.features.flags.mode_override_supported = true;
    hpsInit.modes = *(pBrdInfo->pModes);
    hpsInit.evt_handler = hpsEventHandler;
    hpsInit.p_qwr_buf = pBrdInfo->pBuf;

    btle_init_t btleInit = {0};
    btleInit.advType = ADV_TYPE;
    btleInit.maxDeviceNameLength = MAX_NAME_LENGTH;
    btleInit.pInfo = pBrdInfo->pInfo;
    btleInit.eventHandler = btleEventHandler;
    btleInit.pHpsInit = &hpsInit;

    return btle_Init(&btleInit);
}

static void statusLedHandling(bool limiterActive)
{
    static uint32_t const processRate = MAIN_TIMER_TICKS(20);
    static uint32_t lastCheck;
    uint32_t timestamp, timediff;
    hmi_ledState_t ledState;

    (void)app_timer_cnt_get(&timestamp);
    (void)app_timer_cnt_diff_compute(timestamp, lastCheck, &timediff);
    if (timediff < processRate)
        return;

    lastCheck = timestamp;

    // in standby and off mode deactivate leds to save energy
    if (mainState <= MAINSTATE_STANDBY)
    {
        hmi_SetLed(HMI_LT_RED, HMI_LS_OFF);
        hmi_SetLed(HMI_LT_GREEN, HMI_LS_OFF);
        hmi_SetLed(HMI_LT_BLUE, HMI_LS_OFF);
        return;
    }

    // red indicates limiter status
    ledState = limiterActive ? HMI_LS_ON : HMI_LS_OFF;
    hmi_SetLed(HMI_LT_RED, ledState);

    // green indicates a peripheral connection
    /// TODO: maybe indicate advertising with blinking?
    uint8_t periphCnt = lm_GetPeriphCnt();
    ledState = periphCnt ? HMI_LS_ON : HMI_LS_OFF;
    hmi_SetLed(HMI_LT_GREEN, ledState);

    // blue indicates scanner and central connection
    lm_scanState_t scanMode = lm_GetScanningState();
    uint8_t centralCnt = lm_GetCentralCnt();
    ledState = scanMode == LM_SCAN_SEARCH ? HMI_LS_BLINKFAST :        // blink fast if search (highest priority
               centralCnt ? HMI_LS_ON :                               // on if at least one central is connected
               scanMode == LM_SCAN_LOW_LATENCY ? HMI_LS_BLINKSLOW :   // slow blinking if searching for known devices
               HMI_LS_OFF;                                            // or off
    hmi_SetLed(HMI_LT_BLUE, ledState);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
/*static void execute(bool limiting)
{
    statusLedHandling(limiting);
}*/

static void comHandler(cmh_event_t const* pEvent)
{
    hmi_evtParams_t req;

    switch (pEvent->type)
    {
    case CMH_EVENT_MODE_REQUEST:
        req.mode.connHandle = BLE_CONN_HANDLE_INVALID;
        req.mode.modeNumber = pEvent->mode;
        (void)hmi_RequestEvent(HMI_EVT_MODENUMBER, &req);
        break;
    default:
        break;
    }
}

static ret_code_t comInit(brd_comPins_t const* pPins)
{
    if (pPins == NULL)
        return NRF_ERROR_NULL;

    ret_code_t errCode;
    cmh_init_t init;

    init.rxPin = pPins->rx;
    init.txPin = pPins->tx;
    init.handler = comHandler;

    errCode = cmh_Init(&init);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[main]: com init error %d", errCode);

    return errCode;
}

static void startupCheck(ret_code_t errCode, uint8_t step)
{
    (void)step;

    while (errCode != NRF_SUCCESS)
    {
        uint32_t timestamp;
        (void)app_timer_cnt_get(&timestamp);

        hmi_ledState_t state = timestamp & (0x4000 >> MAIN_TIMER_PRESCALER) ? HMI_LS_ON : HMI_LS_OFF;
        hmi_SetLed(HMI_LT_RED, state);

        dbg_Execute();
    }
}

int main()
{
    brd_info_t const* pBrdInfos;

    ret_code_t errCode;//, criticalError = NRF_SUCCESS;
    uint8_t initMode;

    errCode = dbg_Init();
    startupCheck(errCode, 0);

    errCode = timerInit();
    startupCheck(errCode, 0);

    errCode = sdhInit();
    startupCheck(errCode, 0);

    errCode = brd_Init(&pBrdInfos);
    startupCheck(errCode, 1);

    mm_init_t mmInit =
    {
        .pModesMem = (mm_modeConfig_t*)pBrdInfos->pModes->p_mode_config,
        .pBuf = pBrdInfos->pBuf,
        .modeHandler = modeChanged
    };
    errCode = mm_Init(&mmInit, &initMode);
    startupCheck(errCode, 2);

    hmi_init_t hmiInit = {.eventHandler = hmiEventHandler};
    errCode = hmi_Init(&hmiInit);
    startupCheck(errCode, 3);

    errCode = bluetoothInit(pBrdInfos);
    startupCheck(errCode, 4);

    errCode = comInit(pBrdInfos->pFeatures->pComSupported);
    if (errCode == NRF_ERROR_NULL)  // com not supported at all
        errCode = NRF_SUCCESS;
    startupCheck(errCode, 5);

    errCode = brd_SetPowerMode(BRD_PM_IDLE);
    APP_ERROR_CHECK(errCode);

    if (initMode == MM_MODE_OFF)
        initMode = mm_GetOffMode();
    errCode = brd_SetLightMode(initMode);
    APP_ERROR_CHECK(errCode);

    errCode = lm_SetExposureMode(LM_EXP_LOW_LATENCY);
    APP_ERROR_CHECK(errCode);

    while(1)
    {
        bool limiting = brd_Execute();
        mm_Execute();
        hmi_Execute();
        dbg_Execute();
        statusLedHandling(limiting);
        APP_ERROR_CHECK(sd_app_evt_wait());
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t main_ResetIdleTimer()
{
    ret_code_t errCode = NRF_SUCCESS;

    if (mainState <= MAINSTATE_STANDBY)
    {
        mainState = MAINSTATE_IDLE;
        LOG_INFO("[main]: going to idle mode");

        errCode = lm_SetExposureMode(LM_EXP_LOW_LATENCY);
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[main]: exposure mode set error %d", errCode);
        }

        errCode = brd_SetPowerMode(BRD_PM_IDLE);
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[main]: board power mode set error %d", errCode);
        }
    }

    idleCounter = IDLE_TIMEOUT / IDLE_PERIOD;

    return errCode;
}

/**END OF FILE*****************************************************************/
