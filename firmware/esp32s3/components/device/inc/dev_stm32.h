/**
  ******************************************************************************
  * @file    dev_stm32.h
  * @brief   Device layer: the STM32F103 sensor node behind the gateway UART.
  * @note    One peer device. It pumps the UART, decodes the frames the node
  *          sends and runs the SEQ + ACK stop and wait machine for the frames
  *          this side sends.
  ******************************************************************************
  */
#ifndef __DEV_STM32_H
#define __DEV_STM32_H

#include <stdint.h>
#include "esp_err.h"
#include "link_protocol_defs.h"

/** One data item of the link: ID plus value, see LINK_ID_xxx. */
typedef struct
{
    uint8_t id;
    int32_t value;
} dev_item_t;

/** Highest number of items one frame of the link is expected to carry. */
#define DEV_STM32_MAX_ITEMS     16

/* One whole QUERY answer has to fit into the buffer of this layer. The node
   caps its answer at LINK_QUERY_MAX_ITEMS, so the two numbers can never
   drift apart in silence: too small a buffer here stops the build. */
#if DEV_STM32_MAX_ITEMS < LINK_QUERY_MAX_ITEMS
#error "DEV_STM32_MAX_ITEMS must not be smaller than LINK_QUERY_MAX_ITEMS"
#endif

/** Link counters, all of them restart at zero when the gateway boots. */
typedef struct
{
    uint32_t frame_ok;      /* frames received and CRC checked                 */
    uint32_t frame_bad;     /* frames the parser dropped                       */
    uint32_t tx_ctrl;       /* CONTROL frames sent                             */
    uint32_t tx_query;      /* QUERY frames sent                               */
    uint32_t tx_resend;     /* resends, one per try that stayed unanswered     */
    uint32_t tx_ok;         /* transactions the node acknowledged              */
    uint32_t tx_nak;        /* transactions the node refused                   */
    uint32_t tx_timeout;    /* transactions without any answer                 */
} dev_stm32_stats_t;

/**
 * @brief  Event handler: the node pushed a frame on its own.
 * @param  cmd:   command code, LINK_CMD_REPORT or LINK_CMD_HEARTBEAT.
 * @param  items: decoded data items, valid only during the call.
 * @param  count: number of items, 0 for a heartbeat.
 * @param  user:  context passed to dev_stm32_set_event_handler().
 */
typedef void (*dev_stm32_event_cb_t)(uint8_t cmd, const dev_item_t *items,
                                     uint8_t count, void *user);

/**
 * @brief  Bring up the link to the node: UART plus frame parser.
 */
void dev_stm32_init(void);

/**
 * @brief  Poll the node once, every decoded frame is logged.
 */
void dev_stm32_poll(void);

/**
 * @brief  Subscribe to the frames the node pushes, REPORT and HEARTBEAT.
 * @note   One subscriber at a time, that is the Tasks layer. The answer of a
 *         QUERY is returned by dev_stm32_send_query() instead and is never
 *         handed out as an event, so a query is never counted twice.
 * @param  cb:   handler, NULL unsubscribes.
 * @param  user: context forwarded to the handler.
 */
void dev_stm32_set_event_handler(dev_stm32_event_cb_t cb, void *user);

/**
 * @brief  Copy the link counters.
 * @param  out: destination, NULL is ignored.
 */
void dev_stm32_get_stats(dev_stm32_stats_t *out);

/**
 * @brief  Send CONTROL items and wait for the answer of the node.
 * @note   One acknowledged transaction at a time (stop and wait). A new
 *         transaction always takes a new SEQ, the retries of the same
 *         transaction keep the SEQ, so the node can recognise a resend.
 * @param  items: data items, may contain several system or actuator items.
 * @param  count: number of items.
 * @retval ESP_OK when the node acknowledged, ESP_ERR_INVALID_STATE when another
 *         transaction is still in flight, ESP_ERR_INVALID_ARG when the items do
 *         not fit into one frame, ESP_ERR_INVALID_SIZE when the item buffer is
 *         too small, ESP_ERR_INVALID_RESPONSE when the node refused with a NAK,
 *         ESP_ERR_TIMEOUT when it stayed silent even after every resend.
 */
esp_err_t dev_stm32_send_control_items(const dev_item_t *items, uint8_t count);

/**
 * @brief  Send one CONTROL item and wait for the answer of the node.
 * @param  id:    item ID to change, see LINK_ID_xxx.
 * @param  value: new value, scaled the same way as the reported items.
 * @retval see dev_stm32_send_control_items().
 */
esp_err_t dev_stm32_send_control(uint8_t id, int32_t value);

/**
 * @brief  Ask the node for one item, or for every item it has.
 * @note   The node answers with a REPORT frame that carries the SEQ of this
 *         query, which is how request and answer are paired here.
 * @param  id:  item ID to read, 0 asks for everything the node has.
 * @param  out: buffer that receives the items of the answer.
 * @param  max: capacity of that buffer in items.
 * @param  got: number of items written, may be NULL.
 * @retval ESP_OK when items arrived, ESP_ERR_NOT_FOUND when the answer held none
 *         of the asked items, the other codes as in
 *         dev_stm32_send_control_items().
 */
esp_err_t dev_stm32_send_query(uint8_t id, dev_item_t *out, uint8_t max, uint8_t *got);

#endif /* __DEV_STM32_H */
