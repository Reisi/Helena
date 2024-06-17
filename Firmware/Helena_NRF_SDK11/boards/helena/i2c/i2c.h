/**
  ******************************************************************************
  * @file    i2c.h
  * @author  Thomas Reisnecker
  * @brief   header file for i2c driver
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "sdk_errors.h"
#include "nrf_drv_twi.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
ret_code_t i2c_Init(uint32_t sda, uint32_t scl, nrf_twi_frequency_t frequency);

int8_t i2c_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t len);

int8_t i2c_Write(uint8_t devAddr, uint8_t regAddr, uint8_t const *pData, uint16_t len);

#endif /* I2C_H_INCLUDED */

/**END OF FILE*****************************************************************/
