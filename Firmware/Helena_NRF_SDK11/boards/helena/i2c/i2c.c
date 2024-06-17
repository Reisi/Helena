/**
  ******************************************************************************
  * @file    i2c.c
  * @author  Thomas R.
  * @version V1.0
  * @date    15/11/15
  * @brief   i2c Driver
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_error.h"
#include "i2c.h"
#include "app_util_platform.h"
#include "string.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    XFER_DONE = NRF_DRV_TWI_EVT_DONE,
    XFER_ADDR_NACK = NRF_DRV_TWI_EVT_ADDRESS_NACK,
    XFER_DATA_NACK = NRF_DRV_TWI_EVT_DATA_NACK,
    XFER_PENDING
} xferStatus_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const nrf_drv_twi_t twiInst = NRF_DRV_TWI_INSTANCE(0);
static volatile xferStatus_t xferStatus;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void twiEventHandler(nrf_drv_twi_evt_t const* pEvt, void* pContext)
{
    (void)pContext;

    xferStatus = (xferStatus_t)pEvt->type;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t i2c_Init(uint32_t sda, uint32_t scl, nrf_twi_frequency_t frequency)
{
    nrf_drv_twi_config_t config =
    {
        .scl = scl,
        .sda = sda,
        .frequency = frequency,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    return nrf_drv_twi_init(&twiInst, &config, twiEventHandler, NULL);
}

int8_t i2c_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t len)
{
    /*ret_code_t errCode;

    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(devAddr, &regAddr, 1, pData, len);

    nrf_drv_twi_enable(&twiInst);

    xferStatus = XFER_PENDING;
    errCode = nrf_drv_twi_xfer(&twiInst, &xfer, 0);
    if (errCode == NRF_SUCCESS)
    {
        while (xferStatus == XFER_PENDING) {}
    }

    nrf_drv_twi_disable(&twiInst);

    if (xferStatus != XFER_DONE)
        errCode = NRF_ERROR_INTERNAL;

    return errCode == NRF_SUCCESS ? 0 : -1;*/

    ret_code_t errCode;

    nrf_drv_twi_enable(&twiInst);

    xferStatus = XFER_PENDING;
    errCode = nrf_drv_twi_tx(&twiInst, devAddr, &regAddr, 1, true);
    if (errCode == NRF_SUCCESS)
    {
        while (xferStatus == XFER_PENDING) {}
        if (xferStatus == XFER_DONE)
        {
            xferStatus = XFER_PENDING;
            errCode = nrf_drv_twi_rx(&twiInst, devAddr, pData, len);
        }
    }

    if (errCode == NRF_SUCCESS)
    {
        while (xferStatus == XFER_PENDING) {}
    }

    nrf_drv_twi_disable(&twiInst);

    if (xferStatus != XFER_DONE)
        errCode = NRF_ERROR_INTERNAL;

    return errCode == NRF_SUCCESS ? 0 : -1;


    /*ret_code_t errCode;

    nrf_drv_twi_enable(&twiInst);

    errCode = nrf_drv_twi_tx(&twiInst, devAddr, &regAddr, 1, true);
    if (errCode == NRF_SUCCESS)
    {
        errCode = nrf_drv_twi_rx(&twiInst, devAddr, pData, len);
    }

    nrf_drv_twi_disable(&twiInst);

    return errCode == NRF_SUCCESS ? 0 : -1;*/
}

int8_t i2c_Write(uint8_t devAddr, uint8_t regAddr, uint8_t const *pData, uint16_t len)
{
    /*ret_code_t errCode;

    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXTX(devAddr, &regAddr, 1, (uint8_t*)pData, len);

    nrf_drv_twi_enable(&twiInst);

    xferStatus = XFER_PENDING;
    errCode = nrf_drv_twi_xfer(&twiInst, &xfer, 0);
    if (errCode == NRF_SUCCESS)
    {
        while (xferStatus == XFER_PENDING) {}
    }

    nrf_drv_twi_disable(&twiInst);

    if (xferStatus != XFER_DONE)
        errCode = NRF_ERROR_INTERNAL;

    return errCode == NRF_SUCCESS ? 0 : -1;*/

    ret_code_t errCode;
    uint8_t buffer[len + 1];

    buffer[0] = regAddr;
    memcpy(&buffer[1], pData, len);

    nrf_drv_twi_enable(&twiInst);

    xferStatus = XFER_PENDING;
    errCode = nrf_drv_twi_tx(&twiInst, devAddr, buffer, len + 1, false);
    if (errCode == NRF_SUCCESS)
    {
        while (xferStatus == XFER_PENDING) {}
    }

    nrf_drv_twi_disable(&twiInst);

    if (xferStatus != XFER_DONE)
        errCode = NRF_ERROR_INTERNAL;

    return errCode == NRF_SUCCESS ? 0 : -1;


    /*ret_code_t errCode;
    uint8_t buffer[len + 1];

    buffer[0] = regAddr;
    memcpy(&buffer[1], pData, len);

    nrf_drv_twi_enable(&twiInst);

    errCode = nrf_drv_twi_tx(&twiInst, devAddr, buffer, len + 1, false);

    nrf_drv_twi_disable(&twiInst);

    return errCode == NRF_SUCCESS ? 0 : -1;*/
}

/**END OF FILE*****************************************************************/



