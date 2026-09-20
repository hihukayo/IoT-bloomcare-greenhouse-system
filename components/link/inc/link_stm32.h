#ifndef __LINK_STM32_H
#define __LINK_STM32_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief  Initialise the link to the STM32F103: UART plus frame parser.
 */
void link_stm32_init(void);

/**
 * @brief  Poll the STM32 link once, decoded frames are logged.
 */
void link_stm32_poll(void);

/**
 * @brief  Send one CONTROL command to the node and wait for its answer, the
 *         SEQ + ACK handshake of the link. Only one transaction may be in
 *         flight, the sequence number is released again when the answer
 *         arrives or when every resend failed.
 * @param  id:    item ID to change, see LINK_ID_xxx.
 * @param  value: new value, scaled the same way as the reported items.
 * @retval ESP_OK when the node acknowledged, ESP_ERR_INVALID_STATE when
 *         another transaction is still in flight, ESP_ERR_INVALID_RESPONSE
 *         when the node refused with NAK, ESP_ERR_TIMEOUT when it stayed
 *         silent even after all resends.
 */
esp_err_t link_stm32_send_control(uint8_t id, int32_t value);

#endif /* __LINK_STM32_H */
