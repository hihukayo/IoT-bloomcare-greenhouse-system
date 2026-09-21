/**
  ******************************************************************************
  * @file    link_protocol_defs.h
  * @brief   The wire contract of the STM32F103 node <-> ESP32-S3 gateway link.
  * @note    Pure C, no platform header at all: both ends include this very file,
  *          so the frame format, the command codes and the item IDs cannot drift
  *          apart. Only constants live here.
  * @note    The two implementations deliberately stay apart: the node runs the
  *          transport callback plus the whole acknowledged transaction, the
  *          gateway keeps the stop and wait in its device layer. So there is no
  *          code in this file and therefore nothing to build.
  * @note    Included from
  *            firmware/stm32/Components/Inc/comp_link.h
  *              (Keil reaches it through the IncludePath entry ../../shared)
  *            firmware/esp32s3/components/protocol/inc/link_protocol.h
  *              (ESP-IDF reaches it through the shared component)
  ******************************************************************************
  */
#ifndef __LINK_PROTOCOL_DEFS_H
#define __LINK_PROTOCOL_DEFS_H

/* ------------------------------------------------------------------
 * Frame layout (little endian):
 *
 *   AA 55 | VER | ADDR | LEN | CMD | SEQ | PAYLOAD(LEN) | CRC_L CRC_H
 *    0  1     2      3     4     5     6          7 ...
 *
 *   VER     : protocol version, 0x01
 *   ADDR    : node this frame belongs to, see LINK_ADDR_xxx. A node fills in
 *             its own address, the gateway fills in the node it talks to, so
 *             one gateway can serve several identical sensor nodes later on.
 *   LEN     : payload length in bytes, 0 ~ 250
 *   CMD     : command code, see LINK_CMD_xxx
 *   SEQ     : sequence number, an ACK or NAK echoes the SEQ it answers
 *   PAYLOAD : data items, each 5 bytes = ID(1B) + int32(4B, little endian).
 *             Values are scaled by 100: 2340 means 23.40 of the unit.
 *   CRC16   : MODBUS (poly 0xA001) over VER .. end of PAYLOAD, low byte first
 *
 * One frame is LINK_HDR_LEN + LEN + LINK_CRC_LEN bytes long.
 *
 * Handshake rules:
 *   - REPORT and HEARTBEAT are fire and forget, they are never acknowledged.
 *   - CONTROL must be answered with ACK or NAK carrying the same SEQ.
 *   - QUERY is answered with a REPORT that echoes the SEQ, so the asking side
 *     can pair request and answer, and with a NAK when the node has no value.
 *   - ACK payload = the CMD it confirms (1 byte).
 *   - NAK payload = the CMD (1 byte) + reason code (1 byte), see LINK_NAK_xxx.
 *   - A resent transaction keeps its SEQ, so the receiver can tell a resend from
 *     a new command and must not run it twice, see LINK_DUP_WINDOW_MS.
 * ------------------------------------------------------------------ */

/* Frame header, one byte each. */
#define LINK_SOF0               0xAAU
#define LINK_SOF1               0x55U
#define LINK_VER                0x01U
#define LINK_MAX_PAYLOAD        250U    /* LEN is one byte, keep 5 bytes spare */
#define LINK_CRC_LEN            2U
#define LINK_HDR_LEN            7U      /* SOF0 SOF1 VER ADDR LEN CMD SEQ     */
#define LINK_MAX_FRAME          (LINK_HDR_LEN + LINK_MAX_PAYLOAD + LINK_CRC_LEN)
#define LINK_ITEM_LEN           5U      /* ID(1B) + int32(4B)                 */

/* Byte offsets inside one frame. */
#define LINK_OFF_VER            2U
#define LINK_OFF_ADDR           3U
#define LINK_OFF_LEN            4U
#define LINK_OFF_CMD            5U
#define LINK_OFF_SEQ            6U
#define LINK_OFF_PAYLOAD        7U

/* Addresses: 0x00 gateway, 0x01..0x0F nodes, 0xFF broadcast. */
#define LINK_ADDR_GATEWAY       0x00U
#define LINK_ADDR_NODE1         0x01U
#define LINK_ADDR_BROADCAST     0xFFU

/* Command codes. */
#define LINK_CMD_REPORT         0x01U
#define LINK_CMD_CONTROL        0x02U
#define LINK_CMD_HEARTBEAT      0x03U
#define LINK_CMD_QUERY          0x04U
#define LINK_CMD_ACK            0x80U
#define LINK_CMD_NAK            0x81U

/* Command handler result and NAK reason codes. */
#define LINK_OK                 0x00U
#define LINK_NAK_UNSUPPORTED    0x01U
#define LINK_NAK_BAD_PARAM      0x02U
#define LINK_NAK_EXEC_FAIL      0x03U

/* Item IDs: 0x01~0x7F sensors, 0x80~0xEF actuators, 0xF0~0xFF system. */
#define LINK_ID_TEMP            0x01U
#define LINK_ID_HUMI            0x02U
#define LINK_ID_REPORT_MS       0xF1U   /* CONTROL: report period in ms       */
#define LINK_ID_ERRCODE         0xF2U

/* Fault codes carried in LINK_ID_ERRCODE. The node side log says which sensor
   failed, the code only tells that one of them did. */
#define LINK_ERR_SENSOR         1U

/* Timing of an acknowledged transaction. LINK_ACK_* are held by the side that
   starts the transaction, LINK_DUP_WINDOW_MS by the side that answers it, but
   both numbers are part of the contract, so both ends see them. */
#define LINK_ACK_TIMEOUT_MS     200U    /* wait for one answer before a resend  */
#define LINK_ACK_RETRY          3U      /* resends before the transaction fails */
#define LINK_DUP_WINDOW_MS      3000U   /* inside this window the same SEQ is a resend */

/* How many items one QUERY answer may carry. The node caps its answer here, so
   the asking side has to size its receive buffer for at least this many. */
#define LINK_QUERY_MAX_ITEMS    8U

#endif /* __LINK_PROTOCOL_DEFS_H */
