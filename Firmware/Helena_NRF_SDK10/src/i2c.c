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
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "app_trace.h"
#include "app_error.h"
#include "i2c.h"
#include "custom_board.h"

#if defined HELENA_DEBUG_RTT
#include "SEGGER_RTT.h"
#endif

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(0);
static nrf_drv_twi_config_t twi_config =
{
    .frequency              = NRF_TWI_FREQ_400K,
    .interrupt_priority     = NRF_APP_PRIORITY_LOW
};
static bool autoRecover = false;
static bool isInitialized = false;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static uint32_t i2c_Recover()
{
    isInitialized = false;
    nrf_drv_twi_uninit(&twi_instance);
    NRF_TWI0->EVENTS_ERROR = 0;
    NRF_TWI0->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    NRF_TWI0->POWER        = 0;
    nrf_delay_us(5);
    NRF_TWI0->POWER        = 1;
    NRF_TWI0->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
    app_trace_log("[I2C]: i2c recovered\r\n");
    return i2c_Init();
}

/* Public functions ----------------------------------------------------------*/
uint32_t i2c_Init()
{
    uint32_t err_code;

    if (isInitialized)
        return NRF_SUCCESS;
    twi_config.scl = pBoardConfig->i2cSCL;
    twi_config.sda = pBoardConfig->i2cSDA;
    err_code = nrf_drv_twi_init(&twi_instance, &twi_config, NULL, NULL);
    app_trace_log("[I2C]: i2c bus initialized\r\n");
    if (err_code == NRF_SUCCESS)
    {
        isInitialized = true;
        //nrf_drv_twi_enable(&twi_instance);
    }
    return err_code;
}

void i2c_EnableAutoRecover(bool enable)
{
    autoRecover = enable;
}

uint32_t i2c_read(uint8_t device_address, uint8_t register_address, uint8_t length, uint8_t *data)
{
    uint32_t errCode;

    nrf_drv_twi_enable(&twi_instance);
    errCode = nrf_drv_twi_tx(&twi_instance, device_address, &register_address, 1, true);
    if (autoRecover)
    {
        uint32_t loop = 1;
        while (errCode == NRF_ERROR_INTERNAL && loop--)
        {
            errCode = i2c_Recover();
            if (errCode == NRF_SUCCESS)
            {
                nrf_drv_twi_enable(&twi_instance);
                errCode = nrf_drv_twi_tx(&twi_instance, device_address, &register_address, 1, true);
            }
        }
    }

    /*if (errCode == NRF_ERROR_INTERNAL && autoRecover)
    {
        //APP_ERROR_HANDLER(0);
        errCode = i2c_Recover();
        if (errCode == NRF_SUCCESS)
        {
            nrf_drv_twi_enable(&twi_instance);
            errCode = nrf_drv_twi_tx(&twi_instance, device_address, &register_address, 1, true);
        }
    }*/

    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = nrf_drv_twi_rx(&twi_instance, device_address, data, length, false);
    if (autoRecover)
    {
        uint32_t loop = 1;
        while (errCode == NRF_ERROR_INTERNAL && loop--)
        {
            errCode = i2c_Recover();
            if (errCode == NRF_SUCCESS)
            {
                nrf_drv_twi_enable(&twi_instance);
                errCode = nrf_drv_twi_rx(&twi_instance, device_address, data, length, false);
            }
        }
    }

    /*if (errCode == NRF_ERROR_INTERNAL && autoRecover)
    {
        //APP_ERROR_HANDLER(0);
        errCode = i2c_Recover();
        if (errCode == NRF_SUCCESS)
        {
            nrf_drv_twi_enable(&twi_instance);
            errCode = nrf_drv_twi_tx(&twi_instance, device_address, &register_address, 1, true);
        }
    }*/
    nrf_drv_twi_disable(&twi_instance);
    return errCode;
}

uint32_t i2c_write(uint8_t device_address, uint8_t register_address, uint8_t length, uint8_t *data)
{
    uint8_t buffer[I2CBUFFERSIZE];//, retry_cnt = 1;
    uint32_t errCode;

    if (length > I2CBUFFERSIZE-1)
        return NRF_ERROR_INVALID_LENGTH;
    buffer[0] = register_address;
    memcpy(&buffer[1], data, length);

    nrf_drv_twi_enable(&twi_instance);
    errCode = nrf_drv_twi_tx(&twi_instance, device_address, buffer, length+1, false);
    if (autoRecover)
    {
        uint32_t loop = 1;
        while (errCode == NRF_ERROR_INTERNAL && loop--)
        {
            errCode = i2c_Recover();
            if (errCode == NRF_SUCCESS)
            {
                nrf_drv_twi_enable(&twi_instance);
                errCode = nrf_drv_twi_tx(&twi_instance, device_address, buffer, length+1, false);
            }
        }
    }

    /*if (errCode == NRF_ERROR_INTERNAL && autoRecover)
    {
        //APP_ERROR_HANDLER(0);
        errCode = i2c_Recover();
        if (errCode == NRF_SUCCESS)
        {
            nrf_drv_twi_enable(&twi_instance);
            errCode = nrf_drv_twi_tx(&twi_instance, device_address, &register_address, 1, true);
        }
    }*/
    nrf_drv_twi_disable(&twi_instance);
    return errCode;
}

/**END OF FILE*****************************************************************/



