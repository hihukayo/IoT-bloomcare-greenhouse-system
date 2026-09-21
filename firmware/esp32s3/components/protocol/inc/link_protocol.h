/**
  ******************************************************************************
  * @file    link_protocol.h
  * @brief   Protocol layer: frame format and (de)serialisation of the
  *          STM32F103 node <-> ESP32-S3 gateway link.
  * @note    Pure software: no register, no pin and no driver, so this component
  *          is portable and stays the mirror image of the node side protocol.
  ******************************************************************************
  */
#ifndef __LINK_PROTOCOL_H
#define __LINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The wire contract: frame layout, handshake rules, addresses, command
   codes, item IDs and timing, shared with the node side. */
#include "link_protocol_defs.h"

/** Frame parser context. */
typedef struct
{
    uint8_t  buf[LINK_MAX_FRAME * 2];
    size_t   len;
    uint32_t frame_ok;
    uint32_t frame_bad;
} link_parser_t;

/**
 * @brief  Callback invoked once for every valid frame.
 * @param  frame: pointer to the whole frame, starting with 0xAA 0x55.
 * @param  user:  user context passed to link_parser_feed().
 */
typedef void (*link_frame_cb_t)(const uint8_t *frame, void *user);

/* ---------------- receiving side ---------------- */

/**
 * @brief  CRC16/MODBUS checksum.
 * @param  data: input buffer.
 * @param  len:  input length in bytes.
 * @retval 16-bit checksum.
 */
uint16_t link_crc16(const uint8_t *data, size_t len);

/**
 * @brief  Decode a little-endian int32 from an item value field.
 * @param  p: pointer to the 4 value bytes.
 * @retval decoded value.
 */
int32_t link_get_i32(const uint8_t *p);

/**
 * @brief  Reset a parser context.
 * @param  p: parser context.
 */
void link_parser_init(link_parser_t *p);

/**
 * @brief  Feed received bytes into the parser, valid frames are reported
 *         through the callback. Bytes without a frame are dropped.
 * @param  p:    parser context.
 * @param  data: newly received bytes.
 * @param  n:    number of received bytes.
 * @param  cb:   callback for every valid frame, may be NULL.
 * @param  user: user context forwarded to the callback.
 */
void link_parser_feed(link_parser_t *p, const uint8_t *data, size_t n,
                      link_frame_cb_t cb, void *user);

/* ---------------- sending side ---------------- */

/**
 * @brief  Build one frame (header, payload, CRC) into a caller buffer.
 * @param  out:     buffer that receives the frame.
 * @param  cap:     capacity of that buffer, at least LINK_HDR_LEN + len + LINK_CRC_LEN.
 * @param  addr:    address field, see LINK_ADDR_xxx.
 * @param  cmd:     command code, see LINK_CMD_xxx.
 * @param  seq:     sequence number to put into the frame.
 * @param  payload: payload bytes, may be NULL when len is 0.
 * @param  len:     payload length in bytes, at most LINK_MAX_PAYLOAD.
 * @retval total frame length in bytes, 0 when the arguments do not fit.
 */
size_t link_frame_build(uint8_t *out, size_t cap, uint8_t addr, uint8_t cmd,
                        uint8_t seq, const uint8_t *payload, uint8_t len);

/**
 * @brief  Append one data item (ID plus int32 value) to a payload buffer.
 * @param  payload: payload buffer, at least cap bytes.
 * @param  cap:     capacity of that buffer.
 * @param  len:     current payload length, updated when the item was added.
 * @param  id:      item ID, see LINK_ID_xxx.
 * @param  value:   item value, scaled by 100 like the reported values.
 * @retval true when the item was appended, false when the buffer is too small.
 */
bool link_item_put(uint8_t *payload, uint8_t cap, uint8_t *len, uint8_t id, int32_t value);

/**
 * @brief  Number of whole items inside a payload.
 * @param  len: payload length in bytes.
 * @retval number of items.
 */
uint8_t link_item_count(uint8_t len);

#endif /* __LINK_PROTOCOL_H */
