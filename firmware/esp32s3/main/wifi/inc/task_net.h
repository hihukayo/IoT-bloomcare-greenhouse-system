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

/* ------------------------------------------------------------------
 * Looking for an access point.
 *
 * The WiFi screen of the panel asks for one sweep and shows what came
 * back. The radio hands the list over sorted by signal, strongest
 * first, so what does not fit in the buffer below is the weakest of
 * what is around and not an arbitrary part of it.
 * ------------------------------------------------------------------ */

/** Room for the answer of one sweep. */
#define TASK_NET_SCAN_MAX       10U

/** One access point the radio found. */
typedef struct
{
    char    ssid[TASK_NET_SSID_LEN];    /**< name of the access point           */
    int8_t  rssi;                       /**< dBm, -40 is close, -90 is far away */
    uint8_t locked;                     /**< 1 when it asks for a key           */
} task_net_ap_t;

/** State of the sweep the interface asked for. */
#define TASK_NET_SCAN_IDLE      0U      /**< nothing asked for yet       */
#define TASK_NET_SCAN_RUNNING   1U      /**< the radio is sweeping       */
#define TASK_NET_SCAN_DONE      2U      /**< the results below are ready */
#define TASK_NET_SCAN_FAILED    3U      /**< the radio refused to sweep  */

/**
 * @brief  Ask for one sweep of the surrounding access points.
 * @note   Returns at once: the radio sweeps in the background and raises
 *         WIFI_EVENT_SCAN_DONE, which Task_Net_Poll() turns into results.
 * @retval 1 when the sweep was started, 0 when the radio is not up, is still
 *         joining an access point, or a sweep is already running.
 */
uint8_t Task_Net_ScanStart(void);

/**
 * @brief  State of the last sweep, TASK_NET_SCAN_xxx.
 * @retval the state, TASK_NET_SCAN_IDLE before the first call to the starter.
 */
uint8_t Task_Net_ScanState(void);

/**
 * @brief  Copy what the last sweep found, strongest first.
 * @param  out: destination buffer.
 * @param  max: capacity of that buffer in entries.
 * @retval number of entries copied, 0 when there is nothing to copy.
 */
uint8_t Task_Net_ScanResults(task_net_ap_t *out, uint8_t max);

/**
 * @brief  Join an access point with the given passphrase.
 * @note   Both are written into the settings area first, so a gateway that
 *         reboots comes back on the network that was picked instead of on the
 *         one compiled in. Call it from the main loop: it writes flash.
 * @param  ssid: network to join, 1 .. 32 characters.
 * @param  pass: passphrase, empty for an open network, at most 64 characters.
 * @retval 1 when the join was started, 0 when the arguments do not fit or the
 *         radio refused them.
 */
uint8_t Task_Net_Join(const char *ssid, const char *pass);

#endif /* __TASK_NET_H */