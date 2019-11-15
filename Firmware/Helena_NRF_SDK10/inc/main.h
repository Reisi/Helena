/**
  ******************************************************************************
  * @file    main.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    15/10/26
  * @brief   Header for main.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define APP_TIMER_PRESCALER 0   /**< Value of the RTC1 PRESCALER register. */
#define DEAD_BEEF           0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);


#endif /* MAIN_H */

/**END OF FILE*****************************************************************/
