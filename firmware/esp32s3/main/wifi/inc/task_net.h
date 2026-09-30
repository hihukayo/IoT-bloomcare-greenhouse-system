/**
  ******************************************************************************
  * @file    task_net.h
  * @brief   Task layer: network task.
  * @note    Puts the gateway on the WiFi, which is how it gets a clock: the chip
  *          has no battery backed RTC, so the time the interface shows is only
  *          as real as the last NTP answer this task received.
  ******************************************************************************/
#ifndef __TASK_NET_H
#define __TASK_NET_H

#include <stdint.h>

/* ------------------------------------------------------------------
 * Task layer: network.
 *
 *   Task_Net_Init  joins the access point and starts the NTP client
 *   Task_Net_Poll  keeps the link alive and serves the query below
 *
 * The credentials are looked for in NVS first, which is where the setup of the
 * interface will write them, and fall back to the values compiled in through
 * wifi_secrets.h. An empty name means the gateway has nothing to join and
 * stays off the air.
 * ------------------------------------------------------------------ */

/** Room for the credentials and the address, terminators included. */
#define TASK_NET_SSID_LEN       33U
#define TASK_NET_PASS_LEN       65U
#define TASK_NET_IP_LEN         16U

/** What the link is doing, the four words a status bar can show. */
typedef enum
{
    TASK_NET_OFF = 0,     /**< nothing to join, the gateway stays off the air */
    TASK_NET_CONNECTING,  /**< looking for the access point */
    TASK_NET_OK,          /**< associated and addressed, the NTP client runs */
    TASK_NET_ERR          /**< the last round of attempts ran out */
} task_net_state_t;

/** Everything the interface knows about the link, in one copy. */
typedef struct
{
    task_net_state_t state;     /**< see task_net_state_t */
    uint8_t configured;         /**< 1 when there is a network to join */
    uint8_t clock_set;          /**< 1 when the time came from an NTP server */
    int8_t  rssi;               /**< dBm of the access point, 0 while down */
    char    ssid[TASK_NET_SSID_LEN];  /**< network the gateway joins */
    char    ip[TASK_NET_IP_LEN];      /**< address it was given, empty while down */
} task_net_info_t;

/**
 * @brief  Bring the gateway onto the WiFi and start the NTP client.
 * @note   Every step may fail on its own: a gateway without a radio, without
 *         stored credentials or without an access point keeps running, it only
 *         keeps its clock at the epoch.
 */
void Task_Net_Init(void);

/**
 * @brief  Serve the network task once, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Net_Poll(uint32_t now_ms);

/**
 * @brief  Copy what the interface knows about the link.
 * @param  out: destination of the copy, untouched when it is NULL.
 */
void Task_Net_Get(task_net_info_t *out);

#endif /* __TASK_NET_H */