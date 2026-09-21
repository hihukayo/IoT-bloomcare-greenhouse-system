/**
  ******************************************************************************
  * @file    comp_link.c
  * @brief   Implementation of the link protocol: framing, CRC, parser and ACK/NAK.
  * @note    State kept here: the byte stream being reassembled and the stop and wait
  *          transmit machine. The tick arrives through Link_Poll().
  ******************************************************************************
  */
#include "comp_link.h"
#include <string.h>

/* ------------------------------------------------------------------
 * Protocol state. Nothing in this file knows about USARTs: every byte
 * leaves or enters through the transport installed by Link_Init().
 * ------------------------------------------------------------------ */
static const Link_Transport_t *s_transport;
static uint32_t s_now_ms;                         /* tick sampled by Link_Poll */

#ifdef LINK_DEBUG
volatile uint32_t g_link_frame_ok;
volatile uint32_t g_link_frame_bad;
volatile uint32_t g_link_frame_other;
volatile uint32_t g_link_cmd_rx;
volatile uint32_t g_link_dup_rx;

volatile uint32_t g_link_ack_tx;
volatile uint32_t g_link_nak_tx;
volatile uint32_t g_link_ack_rx;
volatile uint32_t g_link_nak_rx;
volatile uint32_t g_link_retry_tx;
volatile uint32_t g_link_timeout_tx;
#endif

static uint8_t  s_asm_buf[LINK_MAX_FRAME * 2U];  /* frames being reassembled   */
static uint16_t s_asm_len;

static uint8_t  s_tx_frame[LINK_MAX_FRAME];
static uint8_t  s_item_payload[LINK_MAX_PAYLOAD];
static uint8_t  s_tx_seq;                        /* global sequence number     */

/* ---------------- one acknowledged transaction (stop and wait) ------------- */
static uint8_t  s_tx_pend;                       /* 1 = waiting for one answer */
static uint8_t  s_tx_pend_cmd;
static uint8_t  s_tx_pend_seq;
static uint8_t  s_tx_pend_payload[LINK_MAX_PAYLOAD];
static uint8_t  s_tx_pend_len;
static uint8_t  s_tx_retry;
static uint32_t s_tx_deadline;
static uint8_t  s_tx_failed;                     /* 1 = last transaction failed */

static Link_CmdHandler_t s_cmd_handler;
static Link_QueryHandler_t s_query_handler;

static Link_Item_t s_query_items[LINK_QUERY_MAX_ITEMS];

/* Last CONTROL that ran. The same SEQ arriving again inside LINK_DUP_WINDOW_MS
   means the gateway lost our answer and resent the frame: it is answered again
   with the same result, but the command itself is not executed a second time. */
static uint8_t  s_last_ctrl_seq;
static uint8_t  s_last_ctrl_reason;
static uint8_t  s_last_ctrl_valid;
static uint32_t s_last_ctrl_ms;


/**
 * @brief  CRC16/MODBUS checksum, polynomial 0xA001, initial value 0xFFFF.
 * @param  data: input buffer.
 * @param  len:  input length in bytes.
 * @retval 16-bit checksum.
 */
static uint16_t Link_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  bit;
    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x0001U) != 0U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/**
 * @brief  Build one frame and hand it to the transport.
 * @param  cmd:         command code.
 * @param  seq:         sequence number to put into the frame.
 * @param  payload:     payload bytes, may be NULL when payload_len is 0.
 * @param  payload_len: payload length in bytes.
 * @retval LINK_RET_OK or LINK_RET_FAIL.
 */
static uint8_t Link_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t payload_len)
{
    uint16_t crc;
    uint16_t total;

    if ((s_transport == NULL) || (s_transport->tx == NULL))
    {
        return LINK_RET_FAIL;
    }
    if (payload_len > LINK_MAX_PAYLOAD)
    {
        return LINK_RET_FAIL;
    }
    if ((payload_len > 0U) && (payload == NULL))
    {
        return LINK_RET_FAIL;
    }
    s_tx_frame[0] = LINK_SOF0;
    s_tx_frame[1] = LINK_SOF1;
    s_tx_frame[LINK_OFF_VER] = LINK_VER;
    s_tx_frame[LINK_OFF_ADDR] = LINK_ADDR_NODE1;
    s_tx_frame[LINK_OFF_LEN] = payload_len;
    s_tx_frame[LINK_OFF_CMD] = cmd;
    s_tx_frame[LINK_OFF_SEQ] = seq;
    if (payload_len > 0U)
    {
        memcpy(&s_tx_frame[LINK_OFF_PAYLOAD], payload, payload_len);
    }
    crc = Link_Crc16(&s_tx_frame[LINK_OFF_VER],
                     (uint16_t)((LINK_HDR_LEN - LINK_OFF_VER) + payload_len));
    total = (uint16_t)(LINK_HDR_LEN + payload_len);
    s_tx_frame[total] = (uint8_t)(crc & 0x00FFU);
    total++;
    s_tx_frame[total] = (uint8_t)((crc >> 8) & 0x00FFU);
    total++;
    return (s_transport->tx(s_tx_frame, total) == 0U) ? LINK_RET_OK : LINK_RET_FAIL;
}

/**
 * @brief  Pack data items into a payload and send them as one frame.
 * @param  cmd:   command code to send.
 * @param  seq:   sequence number to put into the frame.
 * @param  items: data item array.
 * @param  count: number of items, at most LINK_MAX_PAYLOAD / LINK_ITEM_LEN.
 * @retval LINK_RET_OK or LINK_RET_FAIL.
 */
static uint8_t Link_SendItems(uint8_t cmd, uint8_t seq, const Link_Item_t *items, uint8_t count)
{
    uint8_t i;
    uint8_t n = 0U;

    if ((items == NULL) || (count == 0U))
    {
        return LINK_RET_FAIL;
    }
    if (((uint16_t)count * LINK_ITEM_LEN) > LINK_MAX_PAYLOAD)
    {
        return LINK_RET_FAIL;
    }
    for (i = 0U; i < count; i++)
    {
        s_item_payload[n] = items[i].id;
        n++;
        s_item_payload[n] = (uint8_t)((uint32_t)items[i].value & 0xFFU);
        n++;
        s_item_payload[n] = (uint8_t)(((uint32_t)items[i].value >> 8) & 0xFFU);
        n++;
        s_item_payload[n] = (uint8_t)(((uint32_t)items[i].value >> 16) & 0xFFU);
        n++;
        s_item_payload[n] = (uint8_t)(((uint32_t)items[i].value >> 24) & 0xFFU);
        n++;
    }
    return Link_SendFrame(cmd, seq, s_item_payload, n);
}

/**
 * @brief  Answer one command with ACK (reason == LINK_OK) or with NAK.

 * @param  seq:    sequence number of the command being answered.
 * @param  cmd:    command code being answered.
 * @param  reason: LINK_OK or a LINK_NAK_xxx code.
 */
static void Link_SendAnswer(uint8_t seq, uint8_t cmd, uint8_t reason)
{
    uint8_t payload[2];

    payload[0] = cmd;
    if (reason == LINK_OK)
    {
        (void)Link_SendFrame(LINK_CMD_ACK, seq, payload, 1U);
#ifdef LINK_DEBUG
        g_link_ack_tx++;
#endif
    }
    else
    {
        payload[1] = reason;
        (void)Link_SendFrame(LINK_CMD_NAK, seq, payload, 2U);
#ifdef LINK_DEBUG
        g_link_nak_tx++;
#endif
    }
}

/**
 * @brief  Hand one complete and CRC-checked frame to the command layer.
 * @param  frame: the whole frame, starting with 0xAA 0x55.
 */
static void Link_OnFrame(const uint8_t *frame)
{
    const uint8_t *payload = &frame[LINK_OFF_PAYLOAD];
    uint8_t addr = frame[LINK_OFF_ADDR];
    uint8_t len  = frame[LINK_OFF_LEN];
    uint8_t cmd  = frame[LINK_OFF_CMD];
    uint8_t seq  = frame[LINK_OFF_SEQ];
    uint8_t reason;

    if ((addr != LINK_ADDR_NODE1) && (addr != LINK_ADDR_BROADCAST))
    {
#ifdef LINK_DEBUG
        g_link_frame_other++;
#endif
        return;                                   /* meant for another node */
    }

    switch (cmd)
    {
    case LINK_CMD_CONTROL:
#ifdef LINK_DEBUG
        g_link_cmd_rx++;
#endif
        if ((s_last_ctrl_valid != 0U) && (s_last_ctrl_seq == seq) &&
            ((uint32_t)(s_now_ms - s_last_ctrl_ms) < LINK_DUP_WINDOW_MS))
        {
            /* The gateway resent this command, its answer must have been lost.
               Answer with the very same result, run nothing a second time. */
#ifdef LINK_DEBUG
            g_link_dup_rx++;
#endif
            Link_SendAnswer(seq, cmd, s_last_ctrl_reason);
            break;
        }
        if (s_cmd_handler == NULL)
        {
            reason = LINK_NAK_UNSUPPORTED;
        }
        else
        {
            reason = s_cmd_handler(cmd, payload, len);
        }
        s_last_ctrl_seq = seq;
        s_last_ctrl_reason = reason;
        s_last_ctrl_ms = s_now_ms;
        s_last_ctrl_valid = 1U;
        Link_SendAnswer(seq, cmd, reason);        /* a command is always answered */
        break;


    case LINK_CMD_QUERY:
    {
        /* A query is answered with a data frame instead of an ACK, and that
           frame echoes the SEQ of the query so the gateway can pair them. */
        uint8_t asked = (len > 0U) ? payload[0] : 0U;
        uint8_t got = 0U;

        if (s_query_handler != NULL)
        {
            got = s_query_handler(asked, s_query_items, (uint8_t)LINK_QUERY_MAX_ITEMS);
        }
        if (got == 0U)
        {
            Link_SendAnswer(seq, cmd, LINK_NAK_UNSUPPORTED);
        }
        else
        {
            (void)Link_SendItems(LINK_CMD_REPORT, seq, s_query_items, got);
        }
        break;
    }


    case LINK_CMD_ACK:
        if ((s_tx_pend != 0U) && (seq == s_tx_pend_seq))
        {
            s_tx_pend = 0U;                       /* done, the SEQ is released   */
            s_tx_failed = 0U;
#ifdef LINK_DEBUG
            g_link_ack_rx++;
#endif
        }
        break;

    case LINK_CMD_NAK:
        if ((s_tx_pend != 0U) && (seq == s_tx_pend_seq))
        {
            s_tx_pend = 0U;                       /* refused, SEQ released again */
            s_tx_failed = 1U;
#ifdef LINK_DEBUG
            g_link_nak_rx++;
#endif
        }
        break;

    case LINK_CMD_REPORT:
    case LINK_CMD_HEARTBEAT:
    default:
        break;                                    /* never acknowledged */
    }
}

/**
 * @brief  Cut every complete frame out of the reassembly buffer.
 */
static void Link_ParserRun(void)
{
    while (s_asm_len > 0U)
    {
        uint16_t start = 0xFFFFU;
        uint16_t i;
        uint8_t  payload_len;
        uint16_t total;
        uint16_t crc_calc;
        uint16_t crc_recv;

        for (i = 0U; (uint16_t)(i + 1U) < s_asm_len; i++)
        {
            if ((s_asm_buf[i] == LINK_SOF0) && (s_asm_buf[i + 1U] == LINK_SOF1))
            {
                start = i;
                break;
            }
        }
        if (start == 0xFFFFU)
        {
            /* No header left, but keep the last byte, it may be a lone 0xAA. */
            s_asm_buf[0] = s_asm_buf[s_asm_len - 1U];
            s_asm_len = 1U;
            return;
        }
        if (start > 0U)
        {
            memmove(s_asm_buf, &s_asm_buf[start], (size_t)(s_asm_len - start));
            s_asm_len = (uint16_t)(s_asm_len - start);
        }
        if (s_asm_len < LINK_HDR_LEN)
        {
            return;                               /* header still incomplete */
        }
        payload_len = s_asm_buf[LINK_OFF_LEN];
        if (payload_len > LINK_MAX_PAYLOAD)
        {
#ifdef LINK_DEBUG
            g_link_frame_bad++;
#endif
            memmove(s_asm_buf, &s_asm_buf[1], (size_t)(s_asm_len - 1U));
            s_asm_len--;
            continue;                             /* resync on the next 0xAA 55 */
        }
        total = (uint16_t)(LINK_HDR_LEN + payload_len + LINK_CRC_LEN);
        if (s_asm_len < total)
        {
            return;                               /* body still incomplete */
        }
        crc_calc = Link_Crc16(&s_asm_buf[LINK_OFF_VER],
                              (uint16_t)((LINK_HDR_LEN - LINK_OFF_VER) + payload_len));
        crc_recv = (uint16_t)((uint16_t)s_asm_buf[total - 2U] |
                              ((uint16_t)s_asm_buf[total - 1U] << 8));
        if ((s_asm_buf[LINK_OFF_VER] != LINK_VER) || (crc_calc != crc_recv))
        {
#ifdef LINK_DEBUG
            g_link_frame_bad++;
#endif
        }
        else
        {
#ifdef LINK_DEBUG
            g_link_frame_ok++;
#endif
            Link_OnFrame(s_asm_buf);
        }
        memmove(s_asm_buf, &s_asm_buf[total], (size_t)(s_asm_len - total));
        s_asm_len = (uint16_t)(s_asm_len - total);
    }
}

/**
 * @brief  Append freshly received bytes and parse everything that is complete.
 * @param  data: new bytes.
 * @param  n:    number of new bytes.
 */
static void Link_ParserFeed(const uint8_t *data, uint16_t n)
{
    if (((uint32_t)n + (uint32_t)s_asm_len) > (uint32_t)sizeof(s_asm_buf))
    {
        s_asm_len = 0U;                           /* must not happen, start over */
    }
    memcpy(&s_asm_buf[s_asm_len], data, n);
    s_asm_len = (uint16_t)(s_asm_len + n);
    Link_ParserRun();
}

/**
 * @brief  Resend or give up the pending acknowledged transaction.
 */
static void Link_TxTick(void)
{
    if (s_tx_pend == 0U)
    {
        return;
    }
    if ((int32_t)(s_now_ms - s_tx_deadline) < 0)
    {
        return;                                   /* still inside the timeout */
    }
    if (s_tx_retry < LINK_ACK_RETRY)
    {
        s_tx_retry++;
        s_tx_deadline = s_now_ms + LINK_ACK_TIMEOUT_MS;
        (void)Link_SendFrame(s_tx_pend_cmd, s_tx_pend_seq, s_tx_pend_payload, s_tx_pend_len);
#ifdef LINK_DEBUG
        g_link_retry_tx++;
#endif
        return;
    }
    /* Answer lost: mark the transaction as failed and release the SEQ. */
    s_tx_pend = 0U;
    s_tx_failed = 1U;
#ifdef LINK_DEBUG
    g_link_timeout_tx++;
#endif
}

/**
 * @brief  Reset the protocol state and start reception.
 */
void Link_Init(const Link_Transport_t *transport)
{
    s_transport = transport;
    s_now_ms = 0U;
    s_tx_seq = 0U;
    s_tx_pend = 0U;
    s_tx_failed = 0U;
    s_tx_retry = 0U;
    s_asm_len = 0U;
    s_cmd_handler = NULL;
    s_query_handler = NULL;
    s_last_ctrl_valid = 0U;
    s_last_ctrl_seq = 0U;
    s_last_ctrl_reason = LINK_OK;
    s_last_ctrl_ms = 0U;

    if ((s_transport != NULL) && (s_transport->start_rx != NULL))
    {
        s_transport->start_rx();
    }
}

/**
 * @brief  Serve the link once, call it from the main loop.
 */
void Link_Poll(uint32_t now_ms)
{
    uint8_t  chunk[64];
    uint16_t n;

    s_now_ms = now_ms;
    if ((s_transport == NULL) || (s_transport->rx_take == NULL) || (s_transport->idle_take == NULL))
    {
        return;
    }
    if (s_transport->idle_take() != 0U)
    {
        /* The line just went idle, so a whole frame is waiting in the ring. */
        while ((n = s_transport->rx_take(chunk, (uint16_t)sizeof(chunk))) > 0U)
        {
            Link_ParserFeed(chunk, n);
        }
    }
    Link_TxTick();
}

/**
 * @brief  Install the command handler.
 */
void Link_SetCmdHandler(Link_CmdHandler_t handler)
{
    s_cmd_handler = handler;
}

/**
 * @brief  Install the query handler.
 */
void Link_SetQueryHandler(Link_QueryHandler_t handler)
{
    s_query_handler = handler;
}


/**
 * @brief  Report a list of data items to the gateway.
 */
uint8_t Link_SendReport(const Link_Item_t *items, uint8_t count)
{
    /* A report is fire and forget: it takes the next free SEQ and waits for
       no answer. */
    return Link_SendItems(LINK_CMD_REPORT, s_tx_seq++, items, count);
}


/**
 * @brief  Send one heartbeat frame.
 */
uint8_t Link_SendHeartbeat(void)
{
    return Link_SendFrame(LINK_CMD_HEARTBEAT, s_tx_seq++, NULL, 0U);
}

/**
 * @brief  Send a frame that must be acknowledged (stop and wait).
 * @note   The deadline runs on the tick sampled by the last Link_Poll() call.
 */
uint8_t Link_SendReliable(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t ret;

    if (s_tx_pend != 0U)
    {
        return LINK_RET_BUSY;                     /* one transaction at a time */
    }
    if (len > LINK_MAX_PAYLOAD)
    {
        return LINK_RET_FAIL;
    }
    if ((len > 0U) && (payload == NULL))
    {
        return LINK_RET_FAIL;
    }
    if (len > 0U)
    {
        memcpy(s_tx_pend_payload, payload, len);
    }
    s_tx_pend_len = len;
    s_tx_pend_cmd = cmd;
    s_tx_pend_seq = s_tx_seq;
    s_tx_seq++;
    ret = Link_SendFrame(cmd, s_tx_pend_seq, s_tx_pend_payload, len);
    if (ret != LINK_RET_OK)
    {
        return ret;                               /* nothing left the port */
    }
    s_tx_retry = 0U;
    s_tx_deadline = s_now_ms + LINK_ACK_TIMEOUT_MS;
    s_tx_pend = 1U;
    return LINK_RET_OK;
}

/**
 * @brief  Tell whether an acknowledged transaction is still in flight.
 */
uint8_t Link_TxBusy(void)
{
    return (s_tx_pend != 0U) ? 1U : 0U;
}

/**
 * @brief  Result of the last acknowledged transaction.
 */
uint8_t Link_LastTxFailed(void)
{
    return s_tx_failed;
}
