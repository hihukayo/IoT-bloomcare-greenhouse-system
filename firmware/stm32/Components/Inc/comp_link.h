/**
  ******************************************************************************
  * @file    comp_link.h
  * @brief   Protocol layer: frame format of the STM32 node <-> ESP32-S3 gateway link.
  * @note    Pure software: no register, no pin and no HAL reference, so this file runs
  *          on any platform as soon as the board layer provides a Link_Transport_t.
  ******************************************************************************
  */
#ifndef __COMP_LINK_H
#define __COMP_LINK_H

#include <stdint.h>

/* ------------------------------------------------------------------
 * STM32F103 node <-> ESP32-S3 gateway link protocol.
 *
 * This is a pure software protocol layer: it never touches a register,
 * a pin or a HAL handle, it only builds and parses bytes. The bytes are
 * moved by the transport passed to Link_Init() (the board layer fills in
 * a BSP implementation), so the very same file runs on any platform.
 *
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
 *   PAYLOAD : data items, each 5 bytes = ID(1B) + int32(4B, little endian)
 *   CRC16   : MODBUS (poly 0xA001) over VER .. end of PAYLOAD, low byte first
 *
 * One frame is LINK_HDR_LEN + LEN + LINK_CRC_LEN bytes long.
 *
 * Handshake rules:
 *   - REPORT and HEARTBEAT are fire and forget, they are never acknowledged.
 *   - CONTROL must be answered with ACK or NAK carrying the same SEQ.
 *   - ACK payload = the CMD it confirms (1 byte).
 *   - NAK payload = the CMD (1 byte) + reason code (1 byte), see LINK_NAK_xxx.
 * ------------------------------------------------------------------ */

/* Comment out the next line to drop every debug counter and dump routine. */
#define LINK_DEBUG

#define LINK_SOF0               0xAAU
#define LINK_SOF1               0x55U
#define LINK_VER                0x01U
#define LINK_MAX_PAYLOAD        250U    /* LEN is one byte, keep 5 bytes spare */
#define LINK_CRC_LEN            2U
#define LINK_HDR_LEN            7U      /* SOF0 SOF1 VER ADDR LEN CMD SEQ    */
#define LINK_MAX_FRAME          (LINK_HDR_LEN + LINK_MAX_PAYLOAD + LINK_CRC_LEN)
#define LINK_ITEM_LEN           5U      /* ID(1B) + int32(4B)                */
#define LINK_ACK_TIMEOUT_MS     200U    /* wait for one answer before resend */
#define LINK_ACK_RETRY          3U      /* resends before the transaction fails */
#define LINK_QUERY_MAX_ITEMS    8U      /* items one QUERY answer may carry      */
#define LINK_DUP_WINDOW_MS      3000U   /* inside this window the same SEQ is a resend */


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
#define LINK_ADDR_LOCAL         LINK_ADDR_NODE1

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

/* Return codes of the API. */
#define LINK_RET_OK             0x00U
#define LINK_RET_FAIL           0x01U
#define LINK_RET_BUSY           0x02U

/* Item IDs: 0x01~0x7F sensors, 0x80~0xEF actuators, 0xF0~0xFF system. */
#define LINK_ID_TEMP            0x01U
#define LINK_ID_HUMI            0x02U
#define LINK_ID_REPORT_MS       0xF1U   /* CONTROL: report period in ms      */
#define LINK_ID_ERRCODE         0xF2U
/* Fault codes carried in LINK_ID_ERRCODE: the node side log says which
   sensor failed, the code only tells that one of them did. */
#define LINK_ERR_SENSOR         1

/** One data item: ID plus int32 value, values are scaled by 100. */
typedef struct
{
    uint8_t id;
    int32_t value;
} Link_Item_t;

/**
 * @brief  Handler for the CONTROL frames that arrive from the gateway.
 * @param  cmd:     command code of the frame.
 * @param  payload: payload bytes, each item is LINK_ITEM_LEN bytes long.
 * @param  len:     payload length in bytes.
 * @retval LINK_OK when the command was executed, otherwise a LINK_NAK_xxx reason.
 */
typedef uint8_t (*Link_CmdHandler_t)(uint8_t cmd, const uint8_t *payload, uint8_t len);

/**
 * @brief  Handler for the QUERY frames of the gateway.
 * @note   The answer is a REPORT frame that echoes the SEQ of the query, so the
 *         gateway can pair request and answer; a NAK is sent when the id is
 *         unknown or no value is ready yet.
 * @param  id:  item id asked for, 0 means every item the node has.
 * @param  out: buffer that receives the answer items.
 * @param  max: capacity of that buffer in items.
 * @retval number of items written, 0 when nothing can be answered.
 */
typedef uint8_t (*Link_QueryHandler_t)(uint8_t id, Link_Item_t *out, uint8_t max);


/**
 * @brief  Everything this protocol needs from the platform underneath.
 * @note   The board layer (BSP/bsp_uart.c) provides the implementation, so
 *         this file stays portable: move it to another MCU and only the
 *         three function pointers have to be provided again.
 */
typedef struct
{
    void     (*start_rx)(void);                          /* arm the receiver   */
    uint16_t (*rx_take)(uint8_t *dst, uint16_t max);     /* pending bytes, 0..max */
    uint8_t  (*idle_take)(void);                         /* 1 = frame ended on the wire */
    uint8_t  (*tx)(const uint8_t *data, uint16_t len);    /* 0 = sent           */
} Link_Transport_t;

/**
 * @brief  Reset the protocol state and start reception.
 * @param  transport: platform functions described above, must stay valid.
 */
void Link_Init(const Link_Transport_t *transport);

/**
 * @brief  Serve the link once, call it from the main loop.
 * @param  now_ms: current millisecond tick, it drives the ACK timeout.
 */
void Link_Poll(uint32_t now_ms);

/**
 * @brief  Install the command handler.
 * @param  handler: handler for CONTROL frames, NULL disables the support.
 */
void Link_SetCmdHandler(Link_CmdHandler_t handler);

/**
 * @brief  Install the query handler.
 * @param  handler: handler for QUERY frames, NULL answers them with a NAK.
 */
void Link_SetQueryHandler(Link_QueryHandler_t handler);


/**
 * @brief  Report a list of data items to the gateway.
 * @param  items: data item array.
 * @param  count: number of items.
 * @retval LINK_RET_OK or LINK_RET_FAIL.
 */
uint8_t Link_SendReport(const Link_Item_t *items, uint8_t count);

/**
 * @brief  Send one heartbeat frame.
 * @retval LINK_RET_OK or LINK_RET_FAIL.
 */
uint8_t Link_SendHeartbeat(void);

/**
 * @brief  Send a frame that must be acknowledged (stop and wait).
 * @param  cmd:     command code.
 * @param  payload: payload bytes, may be NULL when len is 0.
 * @param  len:     payload length in bytes.
 * @retval LINK_RET_OK, LINK_RET_BUSY or LINK_RET_FAIL.
 */
uint8_t Link_SendReliable(uint8_t cmd, const uint8_t *payload, uint8_t len);

/**
 * @brief  Tell whether an acknowledged transaction is still in flight.
 * @retval 1 when busy, 0 when free.
 */
uint8_t Link_TxBusy(void);

/**
 * @brief  Result of the last acknowledged transaction.
 * @retval 1 when it timed out, 0 when it was confirmed.
 */
uint8_t Link_LastTxFailed(void);

#ifdef LINK_DEBUG
/* Plain globals instead of a struct: they only serve debugging on USART1. */
extern volatile uint32_t g_link_frame_ok;       /* frames accepted               */
extern volatile uint32_t g_link_frame_bad;      /* wrong length, VER or CRC      */
extern volatile uint32_t g_link_frame_other;    /* addressed to another node     */
extern volatile uint32_t g_link_cmd_rx;         /* CONTROL frames executed       */
extern volatile uint32_t g_link_dup_rx;         /* CONTROL resends answered only */

extern volatile uint32_t g_link_ack_tx;         /* ACK frames sent               */
extern volatile uint32_t g_link_nak_tx;         /* NAK frames sent               */
extern volatile uint32_t g_link_ack_rx;         /* ACK frames received           */
extern volatile uint32_t g_link_nak_rx;         /* NAK frames received           */
extern volatile uint32_t g_link_retry_tx;       /* frames resent                 */
extern volatile uint32_t g_link_timeout_tx;     /* transactions given up         */
#endif /* LINK_DEBUG */

#endif /* __COMP_LINK_H */
