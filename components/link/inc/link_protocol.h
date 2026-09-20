#ifndef __LINK_PROTOCOL_H
#define __LINK_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* ------------------------------------------------------------------
 * STM32F103 <-> ESP32-S3 link, frame format (little endian):
 *
 *   AA 55 | VER | ADDR | LEN | CMD | SEQ | PAYLOAD(LEN) | CRC_L CRC_H
 *     0 1     2     3     4     5     6         7 ...
 *
 *   VER     : protocol version, currently 0x01
 *   ADDR    : node this frame belongs to, see LINK_ADDR_xxx
 *   LEN     : payload length in bytes, 0 ~ 250
 *   CMD     : command code, see LINK_CMD_xxx
 *   SEQ     : sequence number, an ACK or NAK echoes the SEQ it answers
 *   PAYLOAD : data items, each 5 bytes = ID(1B) + int32(4B)
 *   CRC16   : MODBUS, over VER .. end of PAYLOAD, low byte first
 *
 * Handshake rules:
 *   - REPORT / HEARTBEAT are fire and forget, they are never acknowledged.
 *   - CONTROL must be answered with ACK or NAK carrying the same SEQ.
 * ------------------------------------------------------------------ */
#define LINK_SOF0           0xAA
#define LINK_SOF1           0x55
#define LINK_VER            0x01
#define LINK_OFF_VER        2
#define LINK_OFF_ADDR       3
#define LINK_OFF_LEN        4
#define LINK_OFF_CMD        5
#define LINK_OFF_SEQ        6
#define LINK_OFF_PAYLOAD    7
#define LINK_HDR_LEN        7
#define LINK_CRC_LEN        2
#define LINK_MAX_PAYLOAD    250
#define LINK_MAX_FRAME      (LINK_HDR_LEN + LINK_MAX_PAYLOAD + LINK_CRC_LEN)
#define LINK_ITEM_LEN       5
#define LINK_ACK_TIMEOUT_MS 200     /* wait for one answer before a resend */
#define LINK_ACK_RETRY      3       /* resends before the transaction fails */

/* Addresses: 0x00 gateway, 0x01..0x0F nodes, 0xFF broadcast. */
#define LINK_ADDR_GATEWAY   0x00
#define LINK_ADDR_NODE1     0x01
#define LINK_ADDR_BROADCAST 0xFF
#define LINK_ADDR_STM32     LINK_ADDR_NODE1

/* Command codes */
#define LINK_CMD_REPORT     0x01
#define LINK_CMD_CONTROL    0x02
#define LINK_CMD_HEARTBEAT  0x03
#define LINK_CMD_QUERY      0x04
#define LINK_CMD_ACK        0x80
#define LINK_CMD_NAK        0x81

/* Command handler result and NAK reason codes */
#define LINK_OK             0x00
#define LINK_NAK_UNSUPPORTED 0x01
#define LINK_NAK_BAD_PARAM  0x02
#define LINK_NAK_EXEC_FAIL  0x03

/* Item IDs: 0x01~0x7F sensors, 0x80~0xEF actuators, 0xF0~0xFF system */
#define LINK_ID_TEMP        0x01
#define LINK_ID_HUMI        0x02
#define LINK_ID_REPORT_MS   0xF1    /* CONTROL: report period in ms */
#define LINK_ID_ERRCODE     0xF2

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

#endif /* __LINK_PROTOCOL_H */
