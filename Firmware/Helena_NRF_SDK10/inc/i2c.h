/**
  ******************************************************************************
  * @file    i2c.h
  * @author  Thomas R.
  * @version V1.0
  * @date    15/11/15
  * @brief   header file for i2c driver
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define I2CERRORCOUNT_TO_RECOVER    5
#define I2CBUFFERSIZE               20

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
uint32_t i2c_Init(void);
void i2c_EnableAutoRecover(bool enable);
uint32_t i2c_read(uint8_t device_address, uint8_t register_address, uint8_t length, uint8_t *data);
uint32_t i2c_write(uint8_t device_address, uint8_t register_address, uint8_t length, uint8_t *data);

#endif /* I2C_H_INCLUDED */

/**END OF FILE*****************************************************************/
