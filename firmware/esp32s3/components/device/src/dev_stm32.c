/**
  ******************************************************************************
  * @file    dev_stm32.c
  * @brief   Implementation of the STM32F103 node device.
  * @note    Receiving is polled from here on top of the UART driver buffer,
  *          sending is a SEQ + ACK stop and wait machine, so exactly one
  *          acknowledged transaction is in flight at a time.
  ******************************************************************************
  */
#include "dev_stm32.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "app_debug.h"
#include "bsp_uart.h"
#include "link_protocol.h"

#define DEV_STM32_READ_CHUNK    128     /* bytes read in one poll slice       */
#define DEV_STM32_READ_TIMEOUT  50      /* ms, longest blocking slice of the loop */
#define DEV_STM32_PUMP_SLICE    10      /* ms, read slice while waiting       */

#ifdef APP_DEBUG
static const char *TAG = "DEV_STM32";   /* only used by the LOGx macros */
#endif

/** What the pending transaction waits for. */
typedef enum
{
    DEV_WAIT_NONE = 0,                  /* nothing in flight                  */
    DEV_WAIT_ACK,                       /* CONTROL: an ACK or NAK is expected */
    DEV_WAIT_REPORT                     /* QUERY: a REPORT is expected        */
} dev_wait_t;

static link_parser_t s_parser;
static uint32_t      s_bad_seen;

static uint8_t    s_tx_seq;             /* next sequence number to hand out   */
static dev_wait_t s_wait;               /* stop and wait, one transaction     */
static uint8_t    s_wait_seq;
static bool       s_wait_done;
static bool       s_wait_nak;
static uint8_t    s_wait_reason;
static uint8_t    s_wait_payload[LINK_MAX_PAYLOAD];  /* REPORT of a QUERY      */
static uint8_t    s_wait_len;

static dev_stm32_event_cb_t s_event_cb;     /* subscriber of REPORT / HEARTBEAT */
static void                *s_event_user;
static dev_stm32_stats_t    s_stats;

#ifdef APP_DEBUG
/**
 * @brief  Log a byte buffer as hex.
 * @param  prefix: text printed before the hex dump.
 * @param  data:   bytes to dump.
 * @param  len:    number of bytes.
 */
static void dev_log_hex(const char *prefix, const uint8_t *data, size_t len)
{
    char   line[(3 * LINK_MAX_FRAME) + 1] = { 0 };
    size_t pos = 0U;
    size_t i;

    for (i = 0U; (i < len) && ((pos + 4U) < sizeof(line)); i++)
    {
        pos += (size_t)snprintf(&line[pos], sizeof(line) - pos, "%02X ", data[i]);
    }
    if (pos > 0U)
    {
        pos--;                                  /* drop the last space */
        line[pos] = 0;
    }
    LOGI("%s %s", prefix, line);
}

/**
 * @brief  Log a value that is stored scaled by 100.
 * @param  name:  item name.
 * @param  value: stored value, value / 100 is the real number.
 * @param  unit:  unit string.
 */
static void dev_log_x100(const char *name, int32_t value, const char *unit)
{
    int32_t whole = value / 100;
    int32_t frac = value % 100;

    if (frac < 0)
    {
        frac = -frac;
    }
    if ((value < 0) && (whole == 0))
    {
        LOGI("  %s = -0.%02ld %s", name, (long)frac, unit);
    }
    else
    {
        LOGI("  %s = %ld.%02ld %s", name, (long)whole, (long)frac, unit);
    }
}

/**
 * @brief  Log one payload item.
 * @param  id:    item ID, see LINK_ID_xxx.
 * @param  value: item value.
 */
static void dev_log_item(uint8_t id, int32_t value)
{
    switch (id)
    {
    case LINK_ID_TEMP:
        dev_log_x100("air temperature", value, "C");
        break;
    case LINK_ID_HUMI:
        dev_log_x100("air humidity", value, "%RH");
        break;
    case LINK_ID_ERRCODE:
        LOGW("  sensor error code = %ld", (long)value);
        break;
    default:
        /* unknown IDs are ignored, so both sides can be upgraded separately */
        LOGW("  unknown item ID 0x%02X (value %ld), ignored", id, (long)value);
        break;
    }
}
#endif /* APP_DEBUG */

/**
 * @brief  Hand the items of a pushed frame to the Tasks layer subscriber.
 * @note   Frames that answer a pending transaction are consumed by
 *         dev_stm32_transact() and never reach the subscriber.
 * @param  cmd: command code of the frame.
 * @param  pay: payload bytes, valid only during this call.
 * @param  len: payload length in bytes.
 */
static void dev_dispatch_event(uint8_t cmd, const uint8_t *pay, uint8_t len)
{
    dev_item_t items[DEV_STM32_MAX_ITEMS];
    uint8_t    n = 0U;
    uint8_t    off;

    if (s_event_cb == NULL)
    {
        return;
    }
    for (off = 0U; ((size_t)off + LINK_ITEM_LEN) <= (size_t)len; off = (uint8_t)(off + LINK_ITEM_LEN))
    {
        if (n >= (uint8_t)DEV_STM32_MAX_ITEMS)
        {
            break;
        }
        items[n].id = pay[off];
        items[n].value = link_get_i32(&pay[off + 1U]);
        n++;
    }
    s_event_cb(cmd, items, n, s_event_user);
}

/**
 * @brief  Handle one complete and CRC-checked frame.
 * @param  frame: pointer to the whole frame, starting with 0xAA 0x55.
 * @param  user:  unused.
 */
static void dev_on_frame(const uint8_t *frame, void *user)
{
    const uint8_t *pay = &frame[LINK_OFF_PAYLOAD];
    uint8_t len = frame[LINK_OFF_LEN];
    uint8_t cmd = frame[LINK_OFF_CMD];
    uint8_t seq = frame[LINK_OFF_SEQ];
    (void)user;

#ifdef APP_DEBUG
    LOGI("frame #%lu ADDR=%u CMD=0x%02X SEQ=%u LEN=%u",
            (unsigned long)s_parser.frame_ok, (unsigned)frame[LINK_OFF_ADDR], cmd, seq, len);
    dev_log_hex("  RAW", frame, (size_t)LINK_HDR_LEN + (size_t)len + (size_t)LINK_CRC_LEN);
#endif
    switch (cmd)
    {
    case LINK_CMD_REPORT:
        /* Either the periodic push of the node or the answer to a QUERY. A
           pending query keeps a copy, because the parser buffer is reused as
           soon as this callback returns. */
        if ((s_wait == DEV_WAIT_REPORT) && (seq == s_wait_seq))
        {
            s_wait_len = (len <= sizeof(s_wait_payload)) ? len : (uint8_t)sizeof(s_wait_payload);
            memcpy(s_wait_payload, pay, s_wait_len);
            s_wait_done = true;
        }
        else
        {
            dev_dispatch_event(LINK_CMD_REPORT, pay, len);   /* unsolicited push */
        }
#ifdef APP_DEBUG
        for (uint8_t off = 0U; ((size_t)off + LINK_ITEM_LEN) <= (size_t)len; off = (uint8_t)(off + LINK_ITEM_LEN))
        {
            dev_log_item(pay[off], link_get_i32(&pay[off + 1U]));
        }
#endif
        break;
    case LINK_CMD_HEARTBEAT:
        LOGI("  heartbeat, seq %u", seq);
        dev_dispatch_event(LINK_CMD_HEARTBEAT, pay, len);
        break;
    case LINK_CMD_ACK:
        if ((s_wait == DEV_WAIT_ACK) && (seq == s_wait_seq))
        {
            s_wait_done = true;
            LOGI("  ACK for seq %u", seq);
        }
        else
        {
            LOGW("  unexpected ACK for seq %u, ignored", seq);
        }
        break;
    case LINK_CMD_NAK:
        if ((s_wait != DEV_WAIT_NONE) && (seq == s_wait_seq))
        {
            s_wait_reason = (len > 1U) ? pay[1] : 0U;
            s_wait_nak = true;
            s_wait_done = true;
            LOGW("  NAK for seq %u, reason 0x%02X", seq, s_wait_reason);
        }
        else
        {
            LOGW("  unexpected NAK for seq %u, ignored", seq);
        }
        break;
    case LINK_CMD_QUERY:
        LOGW("  query from the node, ignored: this side asks, it does not answer");
        break;
    default:
        LOGW("  unknown command 0x%02X, ignored", cmd);
        break;
    }
}

/**
 * @brief  Read from the node and parse whatever arrived within the timeout.
 * @param  timeout_ms: how long the read may block.
 */
static void dev_stm32_pump(uint32_t timeout_ms)
{
    uint8_t chunk[DEV_STM32_READ_CHUNK];
    int     n;

    n = bsp_uart_read(chunk, sizeof(chunk), timeout_ms);
    if (n <= 0)
    {
        return;
    }
    link_parser_feed(&s_parser, chunk, (size_t)n, dev_on_frame, NULL);
    if (s_parser.frame_bad != s_bad_seen)
    {
        LOGW("dropped %lu bad frame(s)", (unsigned long)(s_parser.frame_bad - s_bad_seen));
        s_bad_seen = s_parser.frame_bad;
    }
}

/**
 * @brief  Bring up the link to the node: UART plus frame parser.
 */
void dev_stm32_init(void)
{
    bsp_uart_init();
    link_parser_init(&s_parser);
    s_tx_seq = 0U;
    s_wait = DEV_WAIT_NONE;
    s_wait_done = false;
    s_wait_nak = false;
    s_wait_len = 0U;
    s_bad_seen = 0U;
    memset(&s_stats, 0, sizeof(s_stats));
    LOGI("waiting for node %u frames on UART%d ...",
            (unsigned)LINK_ADDR_STM32, (int)BSP_UART_PORT);
}

/**
 * @brief  Poll the node once, every decoded frame is logged.
 */
void dev_stm32_poll(void)
{
    dev_stm32_pump(DEV_STM32_READ_TIMEOUT);
}

/**
 * @brief  Send one frame and wait for the answer of the node (stop and wait).
 * @param  cmd:     command code to send.
 * @param  payload: payload bytes, may be NULL when len is 0.
 * @param  len:     payload length in bytes.
 * @param  expect:  DEV_WAIT_ACK for CONTROL, DEV_WAIT_REPORT for QUERY.
 * @retval ESP_OK or one of the error codes of the public send functions.
 */
static esp_err_t dev_stm32_transact(uint8_t cmd, const uint8_t *payload, uint8_t len,
                                    dev_wait_t expect)
{
    uint8_t frame[LINK_MAX_FRAME];
    size_t  total;
    uint8_t seq;
    uint8_t attempt;

    if (s_wait != DEV_WAIT_NONE)
    {
        return ESP_ERR_INVALID_STATE;           /* one transaction at a time */
    }
    seq = s_tx_seq;
    s_tx_seq++;                                 /* a new transaction takes a new SEQ */
    total = link_frame_build(frame, sizeof(frame), LINK_ADDR_STM32, cmd, seq, payload, len);
    if (total == 0U)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (cmd == LINK_CMD_QUERY)
    {
        s_stats.tx_query++;
    }
    else
    {
        s_stats.tx_ctrl++;
    }
    s_wait_seq = seq;
    s_wait_done = false;
    s_wait_nak = false;
    s_wait_reason = 0U;
    s_wait_len = 0U;
    s_wait = expect;

    for (attempt = 0U; attempt <= LINK_ACK_RETRY; attempt++)
    {
        TickType_t deadline;

        (void)bsp_uart_write(frame, total);
        LOGI("sent CMD=0x%02X SEQ=%u, %u bytes, try %u",
                cmd, seq, (unsigned)total, (unsigned)(attempt + 1U));
        deadline = xTaskGetTickCount() + pdMS_TO_TICKS(LINK_ACK_TIMEOUT_MS);
        while ((int32_t)(xTaskGetTickCount() - deadline) < 0)
        {
            dev_stm32_pump(DEV_STM32_PUMP_SLICE);  /* the answer sets s_wait_done */
            if (s_wait_done)
            {
                break;
            }
        }
        if (s_wait_done)
        {
            break;
        }
        s_stats.tx_resend++;
        LOGW("no answer for SEQ %u, resend %u/%u",
                seq, (unsigned)(attempt + 1U), (unsigned)LINK_ACK_RETRY);
    }
    s_wait = DEV_WAIT_NONE;
    if (!s_wait_done)
    {
        s_stats.tx_timeout++;
        LOGW("transaction SEQ %u gave up, the number is dropped", seq);
        return ESP_ERR_TIMEOUT;
    }
    if (s_wait_nak)
    {
        s_stats.tx_nak++;
        LOGW("node refused SEQ %u with reason 0x%02X", seq, s_wait_reason);
        return ESP_ERR_INVALID_RESPONSE;
    }
    s_stats.tx_ok++;
    return ESP_OK;
}

/**
 * @brief  Send CONTROL items and wait for the answer of the node.
 * @param  items: data items.
 * @param  count: number of items.
 * @retval see dev_stm32.h.
 */
esp_err_t dev_stm32_send_control_items(const dev_item_t *items, uint8_t count)
{
    uint8_t payload[LINK_MAX_PAYLOAD];
    uint8_t len = 0U;
    uint8_t i;

    if ((items == NULL) || (count == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }
    for (i = 0U; i < count; i++)
    {
        if (!link_item_put(payload, (uint8_t)sizeof(payload), &len, items[i].id, items[i].value))
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }
    return dev_stm32_transact(LINK_CMD_CONTROL, payload, len, DEV_WAIT_ACK);
}

/**
 * @brief  Send one CONTROL item and wait for the answer of the node.
 * @param  id:    item ID to change, see LINK_ID_xxx.
 * @param  value: new value, scaled like the reported items.
 * @retval see dev_stm32.h.
 */
esp_err_t dev_stm32_send_control(uint8_t id, int32_t value)
{
    const dev_item_t item = { .id = id, .value = value };

    return dev_stm32_send_control_items(&item, 1U);
}

/**
 * @brief  Ask the node for one item, or for every item it has.
 * @param  id:  item ID to read, 0 asks for everything.
 * @param  out: buffer that receives the items of the answer.
 * @param  max: capacity of that buffer in items.
 * @param  got: number of items written, may be NULL.
 * @retval see dev_stm32.h.
 */
esp_err_t dev_stm32_send_query(uint8_t id, dev_item_t *out, uint8_t max, uint8_t *got)
{
    uint8_t   payload[1];
    uint8_t   plen = 0U;
    uint8_t   n = 0U;
    uint8_t   off;
    esp_err_t err;

    if ((out == NULL) || (max == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (id != 0U)
    {
        payload[0] = id;
        plen = 1U;
    }
    err = dev_stm32_transact(LINK_CMD_QUERY, (plen > 0U) ? payload : NULL, plen, DEV_WAIT_REPORT);
    if (err != ESP_OK)
    {
        if (got != NULL)
        {
            *got = 0U;
        }
        return err;
    }
    for (off = 0U; ((size_t)off + LINK_ITEM_LEN) <= (size_t)s_wait_len; off = (uint8_t)(off + LINK_ITEM_LEN))
    {
        /* The node answers with the items that were asked for. Anything else in
           the frame would be a periodic report that happened to use this SEQ,
           so it is skipped here. */
        if ((id != 0U) && (s_wait_payload[off] != id))
        {
            continue;
        }
        if (n >= max)
        {
            break;
        }
        out[n].id = s_wait_payload[off];
        out[n].value = link_get_i32(&s_wait_payload[off + 1U]);
        n++;
    }
    if (got != NULL)
    {
        *got = n;
    }
    return (n > 0U) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
 * @brief  Subscribe to the frames the node pushes.
 * @param  cb:   handler, NULL unsubscribes.
 * @param  user: context forwarded to the handler.
 */
void dev_stm32_set_event_handler(dev_stm32_event_cb_t cb, void *user)
{
    s_event_cb = cb;
    s_event_user = user;
}

/**
 * @brief  Copy the link counters.
 * @param  out: destination, NULL is ignored.
 */
void dev_stm32_get_stats(dev_stm32_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }
    *out = s_stats;
    out->frame_ok = s_parser.frame_ok;
    out->frame_bad = s_parser.frame_bad;
}