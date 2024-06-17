/**
  ******************************************************************************
  * @file    ble_gatt_queue_c.c
  * @author  Thomas Reisnecker
  * @brief   gatt queue module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "ble_gatt_queue.h"

/* Private defines -----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void tx_buffer_process(ble_gq_inst_t * p_gq_inst)
{
    ret_code_t err_code;

    while (p_gq_inst->tx_pending != 0)
    {
        ble_gq_message_t * p_message = &p_gq_inst->txBuffer[p_gq_inst->tx_index];

        if (p_message->type == BLE_GQ_READ_REQ)
        {
            err_code = sd_ble_gattc_read(p_message->conn_handle, p_message->req.read_handle, 0);
        }
        else
        {
            err_code = sd_ble_gattc_write(p_message->conn_handle, &p_message->req.write_req.gattc_params);
        }

        if (err_code == NRF_SUCCESS)
        {
            p_gq_inst->tx_pending--;
            p_gq_inst->tx_index++;
            p_gq_inst->tx_index %= BLE_GQ_TX_BUFFER_SIZE;
        }
        else
        {
            if (p_gq_inst->error_handler)
            {
                p_gq_inst->error_handler(err_code);
            }
            return;
        }
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t ble_gq_init(ble_gq_inst_t * p_gq_inst, ble_srv_error_handler_t error_handler)
{
    if (p_gq_inst == NULL)
    {
        return NRF_ERROR_NULL;
    }

    memset(p_gq_inst, 0, sizeof(ble_gq_inst_t));

    p_gq_inst->error_handler = error_handler;

    return NRF_SUCCESS;
}

void ble_gq_on_ble_evt(ble_gq_inst_t * p_gq_inst)
{
    tx_buffer_process(p_gq_inst);
}

ret_code_t ble_gq_message_add(ble_gq_inst_t * p_gq_inst, ble_gq_message_t const * p_message)
{
    if (p_gq_inst == NULL || p_message == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_gq_inst->tx_pending == BLE_GQ_TX_BUFFER_SIZE)
    {
        return NRF_ERROR_NO_MEM;
    }

    uint8_t tx_insert;
    tx_insert = p_gq_inst->tx_index + p_gq_inst->tx_pending;
    tx_insert %= BLE_GQ_TX_BUFFER_SIZE;

    memcpy(&p_gq_inst->txBuffer[tx_insert], p_message, sizeof(ble_gq_message_t));
    p_gq_inst->tx_pending++;

    tx_buffer_process(p_gq_inst);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
