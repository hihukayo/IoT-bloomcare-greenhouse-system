/**
  ******************************************************************************
  * @file    link_protocol.c
  * @brief   Building and parsing of the node <-> gateway frames.
  * @note    Only bytes are handled here. The caller moves them through the
  *          board layer (bsp_uart), so nothing depends on a register or a pin.
  ******************************************************************************
  */
#include "link_protocol.h"
#include <string.h>

/**
 * @brief  CRC16/MODBUS checksum, polynomial 0xA001, initial value 0xFFFF.
 * @param  data: input buffer.
 * @param  len:  input length in bytes.
 * @retval 16-bit checksum.
 */
uint16_t link_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    size_t   i;
    int      bit;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            crc = ((crc & 0x0001) != 0) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/**
 * @brief  Decode a little-endian int32 from an item value field.
 * @param  p: pointer to the 4 value bytes.
 * @retval decoded value.
 */
int32_t link_get_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/**
 * @brief  Build one frame (header, payload, CRC) into a caller buffer.
 * @param  out:     buffer that receives the frame.
 * @param  cap:     capacity of that buffer.
 * @param  addr:    address field, see LINK_ADDR_xxx.
 * @param  cmd:     command code, see LINK_CMD_xxx.
 * @param  seq:     sequence number to put into the frame.
 * @param  payload: payload bytes, may be NULL when len is 0.
 * @param  len:     payload length in bytes.
 * @retval total frame length in bytes, 0 when the arguments do not fit.
 */
size_t link_frame_build(uint8_t *out, size_t cap, uint8_t addr, uint8_t cmd,
                        uint8_t seq, const uint8_t *payload, uint8_t len)
{
    size_t   total = (size_t)LINK_HDR_LEN + (size_t)len + (size_t)LINK_CRC_LEN;
    uint16_t crc;

    if ((out == NULL) || (len > LINK_MAX_PAYLOAD) || (total > cap))
    {
        return 0U;
    }
    if ((len > 0U) && (payload == NULL))
    {
        return 0U;
    }
    out[0] = LINK_SOF0;
    out[1] = LINK_SOF1;
    out[LINK_OFF_VER] = LINK_VER;
    out[LINK_OFF_ADDR] = addr;
    out[LINK_OFF_LEN] = len;
    out[LINK_OFF_CMD] = cmd;
    out[LINK_OFF_SEQ] = seq;
    if (len > 0U)
    {
        memcpy(&out[LINK_OFF_PAYLOAD], payload, len);
    }
    crc = link_crc16(&out[LINK_OFF_VER], (size_t)(LINK_OFF_PAYLOAD - LINK_OFF_VER) + (size_t)len);
    out[total - 2U] = (uint8_t)(crc & 0xFFU);
    out[total - 1U] = (uint8_t)((crc >> 8) & 0xFFU);
    return total;
}

/**
 * @brief  Append one data item (ID plus int32 value) to a payload buffer.
 * @param  payload: payload buffer, at least cap bytes.
 * @param  cap:     capacity of that buffer.
 * @param  len:     current payload length, updated when the item was added.
 * @param  id:      item ID, see LINK_ID_xxx.
 * @param  value:   item value, scaled by 100 like the reported values.
 * @retval true when the item was appended, false when the buffer is too small.
 */
bool link_item_put(uint8_t *payload, uint8_t cap, uint8_t *len, uint8_t id, int32_t value)
{
    uint8_t n;

    if ((payload == NULL) || (len == NULL))
    {
        return false;
    }
    n = *len;
    if (((uint16_t)n + LINK_ITEM_LEN) > (uint16_t)cap)
    {
        return false;
    }
    payload[n] = id;
    n++;
    payload[n] = (uint8_t)((uint32_t)value & 0xFFU);
    n++;
    payload[n] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
    n++;
    payload[n] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
    n++;
    payload[n] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
    n++;
    *len = n;
    return true;
}

/**
 * @brief  Number of whole items inside a payload.
 * @param  len: payload length in bytes.
 * @retval number of items.
 */
uint8_t link_item_count(uint8_t len)
{
    return (uint8_t)(len / LINK_ITEM_LEN);
}

/**
 * @brief  Reset a parser context.
 * @param  p: parser context.
 */
void link_parser_init(link_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

/**
 * @brief  Cut every complete frame out of the buffer, verify it and report it.
 * @param  p:    parser context.
 * @param  cb:   frame callback, may be NULL.
 * @param  user: user context forwarded to the callback.
 */
static void link_parser_run(link_parser_t *p, link_frame_cb_t cb, void *user)
{
    while (p->len > 0U)
    {
        size_t start = (size_t)-1;
        size_t i;
        uint8_t payload_len;
        size_t total;
        uint16_t crc_calc;
        uint16_t crc_recv;

        for (i = 0U; (i + 1U) < p->len; i++)
        {
            if ((p->buf[i] == LINK_SOF0) && (p->buf[i + 1U] == LINK_SOF1))
            {
                start = i;
                break;
            }
        }
        if (start == (size_t)-1)
        {
            p->buf[0] = p->buf[p->len - 1U];    /* keep the possible half header */
            p->len = 1U;
            return;
        }
        if (start > 0U)
        {
            memmove(p->buf, &p->buf[start], p->len - start);
            p->len -= start;
        }
        if (p->len < LINK_HDR_LEN)
        {
            return;                             /* header not complete yet */
        }
        payload_len = p->buf[LINK_OFF_LEN];
        if (payload_len > LINK_MAX_PAYLOAD)
        {
            p->frame_bad++;                     /* bad length field, skip this 0xAA */
            memmove(p->buf, &p->buf[1], p->len - 1U);
            p->len--;
            continue;
        }
        total = (size_t)LINK_HDR_LEN + (size_t)payload_len + (size_t)LINK_CRC_LEN;
        if (p->len < total)
        {
            return;                             /* frame not complete yet */
        }
        crc_calc = link_crc16(&p->buf[LINK_OFF_VER],
                              (size_t)(LINK_OFF_PAYLOAD - LINK_OFF_VER) + (size_t)payload_len);
        crc_recv = (uint16_t)(p->buf[total - 2U] | ((uint16_t)p->buf[total - 1U] << 8));
        if ((p->buf[LINK_OFF_VER] != LINK_VER) || (crc_calc != crc_recv))
        {
            p->frame_bad++;
        }
        else
        {
            p->frame_ok++;
            if (cb != NULL)
            {
                cb(p->buf, user);
            }
        }
        memmove(p->buf, &p->buf[total], p->len - total);
        p->len -= total;
    }
}

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
                      link_frame_cb_t cb, void *user)
{
    if ((data == NULL) || (n == 0U))
    {
        return;
    }
    if (n > (sizeof(p->buf) - p->len))
    {
        p->len = 0U;                            /* should not happen, start over */
    }
    memcpy(&p->buf[p->len], data, n);
    p->len += n;
    link_parser_run(p, cb, user);
}
