/**
  ******************************************************************************
  * @file    task_gateway.h
  * @brief   Task layer: gateway task.
  * @note    Outward face of the gateway. Right now everything below the gateway
  *          is the STM32 node, later this is what the OLED and the phone
  *          application talk to.
  ******************************************************************************
  */
#ifndef __TASK_GATEWAY_H
#define __TASK_GATEWAY_H

#include <stdint.h>
#include "esp_err.h"
#include "dev_stm32.h"

/* ------------------------------------------------------------------
 * Task layer: gateway.
 *
 * The sending side of the link is driven from here: this task owns the
 * CONTROL and the QUERY transactions, so every other user of the node
 * asks this task instead of touching the device layer.
 * ------------------------------------------------------------------ */

/**
 * @brief  Prepare the gateway task.
 */
void Task_Gateway_Init(void);

/**
 * @brief  Serve the gateway task once, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Gateway_Poll(uint32_t now_ms);

/**
 * @brief  Tell the node how often it has to report its values.
 * @note   A CONTROL frame, so the node answers with an ACK and this call
 *         returns once that answer arrived or the resends ran out.
 * @param  ms: period in milliseconds, accepted range 1000 .. 60000.
 * @retval ESP_OK on success, ESP_ERR_INVALID_ARG outside the range, the codes
 *         of dev_stm32_send_control() otherwise.
 */
esp_err_t Task_Gateway_SetReportPeriod(uint32_t ms);

/**
 * @brief  Ask the node for its values without waiting for the next report.
 * @param  id:  item ID to read, 0 asks for every item the node has.
 * @param  out: buffer that receives the items of the answer.
 * @param  max: capacity of that buffer in items.
 * @param  got: number of items received, may be NULL.
 * @retval see dev_stm32_send_query().
 */
esp_err_t Task_Gateway_RequestValues(uint8_t id, dev_item_t *out, uint8_t max, uint8_t *got);

#endif /* __TASK_GATEWAY_H */
