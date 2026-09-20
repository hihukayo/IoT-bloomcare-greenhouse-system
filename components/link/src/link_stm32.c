#include "link_stm32.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "app_debug.h"
#include "bsp_uart.h"
#include "link_protocol.h"

#define LINK_READ_CHUNK     128
#define LINK_READ_TIMEOUT   200     /* ms, one poll slice */
#define LINK_PUMP_SLICE     10      /* ms, read slice while waiting for an answer */

#ifdef APP_DEBUG
static const char   *TAG = "LINK";       /* only used by the LOGx macros */
#endif
static link_parser_t s_parser;
static uint32_t      s_bad_seen;

/* One acknowledged transaction at a time (stop and wait). The sequence number
   is released again when the answer arrives or when all resends failed. */
static uint8_t  s_tx_seq;
static uint8_t  s_tx_pend_seq;
static bool     s_tx_pend;
static bool     s_tx_pend_nak;
static uint8_t  s_tx_pend_reason;

#ifdef APP_DEBUG
/**
 * @brief  Log a byte buffer as hex.
 * @param  prefix: text printed before the hex dump.
 * @param  data:   bytes to dump.
 * @param  len:    number of bytes.
 */
static void link_log_hex(const char *prefix, const uint8_t *data, size_t len)
{
    char line[(3 * LINK_MAX_FRAME) + 1];
    size_t pos = 0;
    for (size_t i = 0; i < len && pos + 4 < sizeof(line); i++)
    {
        pos += (size_t)snprintf(&line[pos], sizeof(line) - pos, "%02X ", data[i]);
    }
    line[pos > 0 ? pos - 1 : 0] = '\0';
    LOGI("%s %s", prefix, line);
}

/**
 * @brief  Log a value that is stored scaled by 100.
 * @param  name:  item name.
 * @param  value: raw value, value / 100 is the real number.
 * @param  unit:  unit string.
 */
static void link_log_x100(const char *name, int32_t value, const char *unit)
{
    int32_t whole = value / 100;
    int32_t frac  = value % 100;
    if (frac < 0)
    {
        frac = -frac;
    }
    if (value < 0 && whole == 0)
    {
        LOGI("  %s = -0.%02ld %s", name, (long)frac, unit);
    }
    else
    {
        LOGI("  %s = %ld.%02ld %s", name, (long)whole, (long)frac, unit);
    }
}

#endif

/**
 * @brief  Decode one payload item.
 * @param  id:    item ID, see LINK_ID_xxx.
 * @param  value: item value.
 */
#ifdef APP_DEBUG
static void link_handle_item(uint8_t id, int32_t value)
{
    switch (id)
    {
    case LINK_ID_TEMP:
        link_log_x100("air temperature", value, "C");
        break;
    case LINK_ID_HUMI:
        link_log_x100("air humidity", value, "%RH");
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
#endif

/**
 * @brief  Handle one complete and CRC-checked frame.
 * @param  frame: pointer to the whole frame, starting with 0xAA 0x55.
 * @param  user:  unused.
 */
static void link_on_frame(const uint8_t *frame, void *user)
{
    const uint8_t *pay = &frame[LINK_OFF_PAYLOAD];
    uint8_t len = frame[LINK_OFF_LEN];
    uint8_t cmd = frame[LINK_OFF_CMD];
    uint8_t seq = frame[LINK_OFF_SEQ];
    (void)user;

#ifdef APP_DEBUG
    LOGI("frame #%lu ADDR=%u CMD=0x%02X SEQ=%u LEN=%u",
            (unsigned long)s_parser.frame_ok, (unsigned)frame[LINK_OFF_ADDR], cmd, seq, len);
    link_log_hex("  RAW", frame, (size_t)LINK_HDR_LEN + len + LINK_CRC_LEN);
#endif

    switch (cmd)
    {
    case LINK_CMD_REPORT:
#ifdef APP_DEBUG
        for (uint8_t off = 0; (size_t)off + LINK_ITEM_LEN <= len; off = (uint8_t)(off + LINK_ITEM_LEN))
        {
            link_handle_item(pay[off], link_get_i32(&pay[off + 1]));
        }
#endif
        break;
    case LINK_CMD_HEARTBEAT:
        LOGI("  heartbeat, seq %u", seq);
        break;
    case LINK_CMD_ACK:
        if (s_tx_pend && (seq == s_tx_pend_seq))
        {
            s_tx_pend = false;
            LOGI("  ACK for seq %u", seq);
        }
        else
        {
            LOGW("  unexpected ACK for seq %u, ignored", seq);
        }
        break;
    case LINK_CMD_NAK:
        if (s_tx_pend && (seq == s_tx_pend_seq))
        {
            s_tx_pend = false;
            s_tx_pend_nak = true;
            s_tx_pend_reason = (len > 1U) ? pay[1] : 0U;
            LOGW("  NAK for seq %u, reason 0x%02X", seq, s_tx_pend_reason);
        }
        else
        {
            LOGW("  unexpected NAK for seq %u, ignored", seq);
        }
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
static void link_stm32_pump(uint32_t timeout_ms)
{
    uint8_t chunk[LINK_READ_CHUNK];
    int n = bsp_uart_read(chunk, sizeof(chunk), timeout_ms);
    if (n <= 0)
    {
        return;
    }
    link_parser_feed(&s_parser, chunk, (size_t)n, link_on_frame, NULL);
    if (s_parser.frame_bad != s_bad_seen)
    {
        LOGW("dropped %lu bad frame(s)",
                (unsigned long)(s_parser.frame_bad - s_bad_seen));
        s_bad_seen = s_parser.frame_bad;
    }
}

/**
 * @brief  Initialise the link to the STM32F103: UART plus frame parser.
 */
void link_stm32_init(void)
{
    bsp_uart_init();
    link_parser_init(&s_parser);
    s_tx_seq = 0U;
    s_tx_pend = false;
    s_bad_seen = 0U;
    LOGI("waiting for node %u frames on UART%d ...",
            (unsigned)LINK_ADDR_STM32, (int)BSP_UART_PORT);
}

/**
 * @brief  Poll the STM32 link once, decoded frames are logged.
 */
void link_stm32_poll(void)
{
    link_stm32_pump(LINK_READ_TIMEOUT);
}

/**
 * @brief  Send one CONTROL command and wait for the answer of the node.
 * @param  id:    item ID to change, see LINK_ID_xxx.
 * @param  value: new value, scaled the same way as the reported items.
 * @retval ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_RESPONSE or ESP_ERR_TIMEOUT.
 */
esp_err_t link_stm32_send_control(uint8_t id, int32_t value)
{
    uint8_t  frame[LINK_HDR_LEN + LINK_ITEM_LEN + LINK_CRC_LEN];
    uint16_t crc;

    if (s_tx_pend)
    {
        return ESP_ERR_INVALID_STATE;          /* one transaction at a time */
    }
    frame[0] = LINK_SOF0;
    frame[1] = LINK_SOF1;
    frame[LINK_OFF_VER] = LINK_VER;
    frame[LINK_OFF_ADDR] = LINK_ADDR_STM32;
    frame[LINK_OFF_LEN] = LINK_ITEM_LEN;
    frame[LINK_OFF_CMD] = LINK_CMD_CONTROL;
    frame[LINK_OFF_SEQ] = s_tx_seq;
    frame[LINK_OFF_PAYLOAD] = id;
    frame[LINK_OFF_PAYLOAD + 1U] = (uint8_t)((uint32_t)value & 0xFFU);
    frame[LINK_OFF_PAYLOAD + 2U] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
    frame[LINK_OFF_PAYLOAD + 3U] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
    frame[LINK_OFF_PAYLOAD + 4U] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
    crc = link_crc16(&frame[LINK_OFF_VER],
            (size_t)((LINK_HDR_LEN - LINK_OFF_VER) + LINK_ITEM_LEN));
    frame[sizeof(frame) - 2U] = (uint8_t)(crc & 0xFFU);
    frame[sizeof(frame) - 1U] = (uint8_t)((crc >> 8) & 0xFFU);

    s_tx_pend_seq = s_tx_seq;
    s_tx_pend_nak = false;
    s_tx_pend = true;

    for (uint8_t attempt = 0U; attempt <= LINK_ACK_RETRY; attempt++)
    {
        TickType_t deadline;
        (void)bsp_uart_write(frame, sizeof(frame));
        deadline = xTaskGetTickCount() + pdMS_TO_TICKS(LINK_ACK_TIMEOUT_MS);
        while ((int32_t)(xTaskGetTickCount() - deadline) < 0)
        {
            link_stm32_pump(LINK_PUMP_SLICE);  /* an ACK clears s_tx_pend */
            if (!s_tx_pend)
            {
                break;
            }
        }
        if (!s_tx_pend)
        {
            break;
        }
        LOGW("no answer for seq %u, resend %u/%u",
                (unsigned)s_tx_pend_seq, (unsigned)(attempt + 1U), (unsigned)LINK_ACK_RETRY);
    }

    if (s_tx_pend)
    {
        s_tx_pend = false;                     /* give up, the SEQ is released */
        LOGW("transaction %u failed, SEQ released", (unsigned)s_tx_pend_seq);
        return ESP_ERR_TIMEOUT;
    }
    s_tx_seq++;
    if (s_tx_pend_nak)
    {
        LOGW("node refused seq %u with reason 0x%02X",
                (unsigned)s_tx_pend_seq, s_tx_pend_reason);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}
