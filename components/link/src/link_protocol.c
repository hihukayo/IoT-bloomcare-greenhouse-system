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
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x0001) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
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
    while (p->len > 0)
    {
        size_t start = (size_t)-1;
        for (size_t i = 0; i + 1 < p->len; i++)
        {
            if (p->buf[i] == LINK_SOF0 && p->buf[i + 1] == LINK_SOF1)
            {
                start = i;                       
                break;
            }
        }

        if (start == (size_t)-1)
        {
            p->buf[0] = p->buf[p->len - 1];     /* keep the possible half header */
            p->len = 1;
            return;
        }

        if (start > 0)
        {
            memmove(p->buf, &p->buf[start], p->len - start);
            p->len -= start;
        }

        if (p->len < LINK_HDR_LEN)
        {
            return;                             /* header not complete yet */
        }

        uint8_t payload_len = p->buf[LINK_OFF_LEN];
        if (payload_len > LINK_MAX_PAYLOAD)
        {
            p->frame_bad++;                     /* bad length field, skip this 0xAA */
            memmove(p->buf, &p->buf[1], p->len - 1);
            p->len--;
            continue;
        }

        size_t total = (size_t)LINK_HDR_LEN + payload_len + LINK_CRC_LEN;
        if (p->len < total)
        {
            return;                             /* frame not complete yet */
        }

        uint16_t crc_calc = link_crc16(&p->buf[LINK_OFF_VER],
                                      (size_t)(LINK_OFF_PAYLOAD - LINK_OFF_VER) + payload_len);
        uint16_t crc_recv = (uint16_t)(p->buf[total - 2] | ((uint16_t)p->buf[total - 1] << 8));
        if (p->buf[LINK_OFF_VER] != LINK_VER || crc_calc != crc_recv)
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
    if (n > sizeof(p->buf) - p->len)
    {
        p->len = 0;                             /* should not happen, start over */
    }
    memcpy(&p->buf[p->len], data, n);
    p->len += n;
    link_parser_run(p, cb, user);
}
