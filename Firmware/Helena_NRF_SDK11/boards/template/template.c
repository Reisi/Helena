/**
  ******************************************************************************
  * @file    template.c
  * @author  Thomas Reisnecker
  * @brief   board managment module (template)
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define BRD_LOG_ENABLED

#ifdef BRD_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BRD_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "template_hmi.h"
#include "mode_management.h"
#include "string.h"

/* External variables --------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define SIZE_OF_MODES       (MM_NUM_OF_MODES * sizeof(mm_modeConfig_t))
#define SIZE_OF_CHANNELS    (MM_NUM_OF_MODES * 0)

/* Private typedef -----------------------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/* Private read only variables -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t              modesBuffer[SIZE_OF_MODES + SIZE_OF_CHANNELS];
static uint8_t              qwrMemBuffer[BLE_HPS_MIN_BUF_SIZE(sizeof(modesBuffer) + 4)];    // this buffer is also used for storing new received modes and this storage buffer need 4 additional bytes. since this template does not implement any channels this additional 4 bytes need to be assigned manually here
static ble_user_mem_block_t const qwrMemBlock =
{
    .p_mem = qwrMemBuffer,
    .len = sizeof(qwrMemBuffer)
};
static ble_hps_qwr_buffer_t qwrBuffer = {.p_block = &qwrMemBlock};

/* Board information ---------------------------------------------------------*/
static btle_info_t const bleInfo =
{
    .pDevicename = "Helena",
    .pManufacturer = NULL,
    .pModelNumber = "template",
    .pBoardHWVersion = "n.a.",
    .deviceAppearance = BLE_APPEARANCE_GENERIC_CYCLING,
};

static brd_features_t const features =
{
    .channelCount = 0,
    .pChannelTypes = NULL
};

static ble_hps_modes_init_t const modes =
{
    .p_mode_config = (ble_hps_mode_config_t*)modesBuffer,
    .total_size = sizeof(modesBuffer)
};

static brd_info_t const brdInfo =
{
    .pFeatures = &features,
    .pInfo = &bleInfo,
    .pModes = &modes,
    .pBuf = &qwrBuffer
};

/* Private functions ---------------------------------------------------------*/
static void setModes()
{
    mm_modeConfig_t const* pModesConfig;

    /// TODO: this might be unnecessary if mode management uses modes buffer as well
    (void)mm_GetModeConfigs(&pModesConfig); // cannot fail unless fed with null-pointer
    memcpy(modesBuffer, pModesConfig, SIZE_OF_MODES);
    // add channels
}

/* Public functions ----------------------------------------------------------*/
ret_code_t brd_Init(brd_info_t const* *pInfo)
{
    if (pInfo == NULL)
        return NRF_ERROR_NULL;

    *pInfo = &brdInfo;

    setModes();

    return template_Hmi_Init();
}

ret_code_t brd_SetPowerMode(brd_powerMode_t newMode)
{
    (void)newMode;

    return NRF_SUCCESS;
}

ret_code_t brd_SetLightMode(uint8_t newMode)
{
    (void)newMode;

    return NRF_SUCCESS;
}

ret_code_t brd_OverrideMode(ble_hps_cp_channel_config_t const* pConfig)
{
    if (pConfig->size != features.channelCount)
        return NRF_ERROR_INVALID_LENGTH;

    return NRF_SUCCESS;
}

bool brd_Execute(void)
{
    template_Hmi_Execute();

    return false;
}

ret_code_t brd_GetChannelConfig(void const** ppData, uint16_t* pSize)
{
    if (ppData == NULL || pSize == NULL)
        return NRF_ERROR_NULL;

    *ppData = NULL;

    return NRF_SUCCESS;
}

ret_code_t brd_SetChannelConfig(void const* pData, uint16_t size, ds_reportHandler_t resultHandler)
{
    if (size != 0)
        return NRF_ERROR_INVALID_LENGTH;

    if (resultHandler != NULL)
        resultHandler(NRF_SUCCESS);

    return NRF_SUCCESS;
}

ret_code_t brd_SetDeviceName(char const* pNewName, ds_reportHandler_t resultHandler)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t brd_FactoryReset(ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    resultHandler(NRF_SUCCESS);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
