/**
  ******************************************************************************
  * @file    task_net.c
  * @brief   Implementation of the network task.
  * @note    The gateway is on the air for one reason: without a network there is
  *          no NTP answer, and without an NTP answer the chip has no clock at
  *          all. Everything else this task owns - the access point, the address,
  *          the signal - is told to the interface so that whoever stands in front
  *          of the panel can see why the clock is still missing.
  ******************************************************************************/
#include "task_net.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "app_debug.h"
#include "task_display.h"

/* The credentials of the bench, when that file is there. A clone that never made
   one builds all the same and simply stays off the air, see the example. */
#if defined(__has_include)
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
#endif
#ifndef WIFI_SSID
#define WIFI_SSID       ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS       ""
#endif

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#ifdef APP_DEBUG
static const char *TAG = "TASK_NET";    /* only used by the LOGx macros */
#endif

/* Where the setup of the interface will keep the credentials. */
#define TASK_NET_NVS_NS         "bloomcare"
#define TASK_NET_NVS_SSID       "ssid"
#define TASK_NET_NVS_PASS       "pass"

/* The NTP server, and the zone the panel shows. CST-8 is UTC+8 with no daylight
   saving rule, which is what the bench is in. */
#define TASK_NET_NTP_SERVER     "pool.ntp.org"
#define TASK_NET_TZ             "CST-8"

/* 2025-01-01 as a Unix stamp: a time below it belongs to a chip that has never
   heard from an NTP server, it is not a date. */
#define TASK_NET_CLOCK_FLOOR    ((time_t)1735689600)

/* Five attempts three seconds apart is a network that is not there. From then on
   the gateway tries once every half minute, so it comes back on its own the
   moment the access point returns. */
#define TASK_NET_FAST_MAX       5U
#define TASK_NET_FAST_MS        3000U
#define TASK_NET_SLOW_MS        30000U

/* The signal is read now and then and not on every pass: the query goes down to
   the driver. */
#define TASK_NET_RSSI_MS        2000U

/* Written by the event task and read by the main loop and the LVGL task, one
   word or one byte at a time. */
static volatile uint8_t s_state;
static volatile uint8_t s_link_up;
static volatile uint8_t s_want_connect;
static volatile int8_t  s_rssi;

/* Written and read by the main loop only. */
static uint8_t  s_configured;
static uint8_t  s_attempts;
static uint8_t  s_sntp_started;
static uint32_t s_try_at;
static uint32_t s_rssi_at;

/* Read through Task_Net_Get(). The name only moves while the gateway is being
   set up, the address only while the link comes and goes. */
static char s_ssid[TASK_NET_SSID_LEN];
static char s_pass[TASK_NET_PASS_LEN];
static char s_ip[TASK_NET_IP_LEN];

/* The sweep the panel asks for. The state is written by the event task and by
   the main loop and read by the LVGL task; the list itself is written by the
   main loop and read by the LVGL task, and the count is raised last, so a
   reader either sees the previous list or the whole new one. */
static volatile uint8_t s_scan_state;
static volatile uint8_t s_scan_ready;               /* set by SCAN_DONE      */
static volatile uint8_t s_scan_count;
static task_net_ap_t    s_scan_aps[TASK_NET_SCAN_MAX];
static wifi_ap_record_t s_scan_raw[TASK_NET_SCAN_MAX];

/**
 * @brief  Name of one state, for the log only.
 * @param  state: TASK_NET_xxx.
 * @retval a word that fits a log line.
 */
static const char *Task_Net_StateName(uint8_t state)
{
    switch (state)
    {
        case TASK_NET_CONNECTING:
            return "connecting";
        case TASK_NET_OK:
            return "ok";
        case TASK_NET_ERR:
            return "error";
        default:
            return "offline";
    }
}

/**
 * @brief  Tell the status bar what the link is doing.
 * @note   Called from the event task and from the main loop, the handover is one
 *         byte and the bars refresh on their own, so nothing has to be locked.
 */
static void Task_Net_Publish(void)
{
    switch (s_state)
    {
        case TASK_NET_CONNECTING:
            Task_Display_SetNet(TASK_DISPLAY_NET_CONNECTING);
            break;
        case TASK_NET_OK:
            Task_Display_SetNet(TASK_DISPLAY_NET_OK);
            break;
        case TASK_NET_ERR:
            Task_Display_SetNet(TASK_DISPLAY_NET_ERR);
            break;
        default:
            Task_Display_SetNet(TASK_DISPLAY_NET_UNKNOWN);
            break;
    }
}

/**
 * @brief  Move to one state and pass it on to the interface.
 * @param  state: TASK_NET_xxx.
 */
static void Task_Net_SetState(task_net_state_t state)
{
    if (s_state != (uint8_t)state)
    {
        s_state = (uint8_t)state;
        Task_Net_Publish();
        LOGI("link %s", Task_Net_StateName(s_state));
    }
}

/**
 * @brief  Copy a string into a field, cut to fit and always terminated.
 * @note   The credentials come from a header or from the settings area, so their
 *         length is not known here; this keeps every copy inside its field.
 * @param  dst: destination field.
 * @param  dst_len: size of that field, terminator included.
 * @param  src: string to copy, must be terminated.
 */
static void Task_Net_Copy(char *dst, size_t dst_len, const char *src)
{
    size_t i = 0U;

    while (((i + 1U) < dst_len) && (src[i] != (char)0))
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = (char)0;
}

/**
 * @brief  Find the credentials to use.
 * @note   What the setup wrote into NVS wins over what was compiled in, so a
 *         gateway in the field can be moved to another access point without
 *         being flashed again.
 * @retval 1 when there is a network to join, 0 when there is none.
 */
static uint8_t Task_Net_LoadCredentials(void)
{
    nvs_handle_t handle;
    size_t len;

    s_ssid[0] = '\0';
    s_pass[0] = '\0';

    if (nvs_open(TASK_NET_NVS_NS, NVS_READONLY, &handle) == ESP_OK)
    {
        len = sizeof(s_ssid);
        if (nvs_get_str(handle, TASK_NET_NVS_SSID, s_ssid, &len) != ESP_OK)
        {
            s_ssid[0] = '\0';
        }
        len = sizeof(s_pass);
        if (nvs_get_str(handle, TASK_NET_NVS_PASS, s_pass, &len) != ESP_OK)
        {
            s_pass[0] = '\0';
        }
        nvs_close(handle);
    }

    if (s_ssid[0] == '\0')
    {
        Task_Net_Copy(s_ssid, sizeof(s_ssid), WIFI_SSID);
        Task_Net_Copy(s_pass, sizeof(s_pass), WIFI_PASS);
    }

    return (s_ssid[0] != '\0') ? 1U : 0U;
}

/**
 * @brief  React to the events of the WiFi driver.
 * @note   Nothing here blocks: a retry is only asked for and the main loop runs
 *         it, which is what keeps the task of the driver short.
 * @param  arg:  unused.
 * @param  base: event base, unused.
 * @param  id:   WIFI_EVENT_xxx.
 * @param  data: payload of that event.
 */
static void Task_Net_OnWifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;

    if (id == WIFI_EVENT_SCAN_DONE)
    {
        /* The list the driver allocated is collected by the main loop, so this
           task stays short and never blocks on the driver. */
        s_scan_ready = 1U;
        return;
    }
    if (id == WIFI_EVENT_STA_START)
    {
        s_want_connect = 1U;
        return;
    }
    if (id == WIFI_EVENT_STA_DISCONNECTED)
    {
        const wifi_event_sta_disconnected_t *lost = (const wifi_event_sta_disconnected_t *)data;

        s_link_up = 0U;
        s_want_connect = 1U;
        s_rssi = 0;
        s_ip[0] = '\0';
        if (s_attempts < 255U)
        {
            s_attempts++;
        }

        /* The reason is the whole diagnosis of a link that will not come up:
           201 is a name that was not found, 15 a passphrase that was refused. */
        LOGW("access point lost after %u attempt(s), reason %u",
             (unsigned)s_attempts, (unsigned)lost->reason);

        if (s_attempts > TASK_NET_FAST_MAX)
        {
            Task_Net_SetState(TASK_NET_ERR);
        }
        else
        {
            Task_Net_SetState(TASK_NET_CONNECTING);
        }
        return;
    }
}

/**
 * @brief  React to the events of the network stack.
 * @param  arg:  unused.
 * @param  base: event base, unused.
 * @param  id:   IP_EVENT_xxx.
 * @param  data: payload of that event.
 */
static void Task_Net_OnIp(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *got = (const ip_event_got_ip_t *)data;

    (void)arg;
    (void)base;

    if (id != IP_EVENT_STA_GOT_IP)
    {
        return;
    }
    if (esp_ip4addr_ntoa(&got->ip_info.ip, s_ip, (int)sizeof(s_ip)) == NULL)
    {
        s_ip[0] = '\0';
    }
    s_attempts = 0U;
    s_link_up = 1U;
    s_want_connect = 0U;
    Task_Net_SetState(TASK_NET_OK);
    LOGI("address %s", s_ip);
}

/**
 * @brief  Bring the gateway onto the WiFi and start the NTP client.
 */
void Task_Net_Init(void)
{
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t sta_cfg;
    esp_err_t err;

    s_state = TASK_NET_OFF;
    s_link_up = 0U;
    s_want_connect = 0U;
    s_rssi = 0;
    s_configured = 0U;
    s_attempts = 0U;
    s_sntp_started = 0U;
    s_try_at = 0U;
    s_rssi_at = 0U;
    s_scan_state = TASK_NET_SCAN_IDLE;
    s_scan_ready = 0U;
    s_scan_count = 0U;
    s_ssid[0] = '\0';
    s_pass[0] = '\0';
    s_ip[0] = '\0';
    Task_Net_Publish();

    /* The panel shows a wall clock, so the zone has to be in place before the
       first NTP answer lands and not after it. */
    (void)setenv("TZ", TASK_NET_TZ, 1);
    tzset();

    err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        LOGW("the settings area has to be rebuilt: %s", esp_err_to_name(err));
        (void)nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
    {
        LOGE("no settings area, the credentials cannot be kept: %s", esp_err_to_name(err));
        return;
    }

    s_configured = Task_Net_LoadCredentials();
    if (s_configured == 0U)
    {
        /* No network to join, but the radio still comes up: the panel has to be
           able to look for one, which is exactly what its WiFi screen does.
           Nothing connects until something writes credentials into the
           settings area, which is the setup step still to come. */
        LOGW("no network to join, the panel can look for one");
    }

    err = esp_netif_init();
    if (err != ESP_OK)
    {
        LOGE("network stack: %s", esp_err_to_name(err));
        return;
    }
    err = esp_event_loop_create_default();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE))
    {
        LOGE("event loop: %s", esp_err_to_name(err));
        return;
    }
    if (esp_netif_create_default_wifi_sta() == NULL)
    {
        LOGE("no station interface");
        return;
    }
    err = esp_wifi_init(&wifi_cfg);
    if (err != ESP_OK)
    {
        LOGE("radio: %s", esp_err_to_name(err));
        return;
    }
    (void)esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &Task_Net_OnWifi, NULL, NULL);
    (void)esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &Task_Net_OnIp, NULL, NULL);

    /* The credentials are kept in NVS by this task and not by the driver, so a
       gateway that moves to another access point changes one small value and not
       a blob of driver state. */
    (void)esp_wifi_set_storage(WIFI_STORAGE_RAM);

    memset(&sta_cfg, 0, sizeof(sta_cfg));
    /* The driver wants both credentials terminated inside their own field, and
       those fields are one byte longer than the longest value accepted. A
       gateway without credentials keeps both fields empty and never joins. */
    if (s_configured != 0U)
    {
        memcpy(sta_cfg.sta.ssid, s_ssid, sizeof(sta_cfg.sta.ssid) - 1U);
        memcpy(sta_cfg.sta.password, s_pass, sizeof(sta_cfg.sta.password) - 1U);
    }
    /* WPA2 is the floor: it admits WPA2, the mixed WPA2/WPA3 and WPA3, and keeps
       out the open and the WEP networks, which a gateway should not join. */
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK)
    {
        err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    }
    if (err == ESP_OK)
    {
        err = esp_wifi_start();
    }
    if (err != ESP_OK)
    {
        LOGE("radio start: %s", esp_err_to_name(err));
        return;
    }

    /* A gateway hangs on a wire: there is nothing to save by letting the radio
       sleep between two beacons, and every answer would come late. */
    (void)esp_wifi_set_ps(WIFI_PS_NONE);

    if (s_configured != 0U)
    {
        Task_Net_SetState(TASK_NET_CONNECTING);
        LOGI("joining %s", s_ssid);
    }
}

/**
 * @brief  Collect what the radio found and hand the list to the interface.
 * @note   Main loop only. The driver hands the list over sorted by signal, so
 *         what does not fit in the buffer is the weakest around; the call frees
 *         the whole list the driver allocated either way, so asking for less
 *         than was found leaks nothing.
 */
static void Task_Net_ScanCollect(void)
{
    uint16_t num = (uint16_t)TASK_NET_SCAN_MAX;
    uint8_t  n = 0U;
    uint16_t i;

    s_scan_count = 0U;
    if (esp_wifi_scan_get_ap_records(&num, s_scan_raw) != ESP_OK)
    {
        s_scan_state = TASK_NET_SCAN_FAILED;
        LOGW("the sweep results could not be read");
        return;
    }
    for (i = 0U; (i < num) && (n < (uint8_t)TASK_NET_SCAN_MAX); i++)
    {
        Task_Net_Copy(s_scan_aps[n].ssid, sizeof(s_scan_aps[n].ssid),
                      (const char *)s_scan_raw[i].ssid);
        s_scan_aps[n].rssi = s_scan_raw[i].rssi;
        s_scan_aps[n].locked = (s_scan_raw[i].authmode != WIFI_AUTH_OPEN) ? 1U : 0U;
        n++;
    }
    s_scan_count = n;                 /* raised last, see the note on the list */
    s_scan_state = TASK_NET_SCAN_DONE;
    LOGI("sweep found %u network(s)", (unsigned)n);
}

/**
 * @brief  Serve the network task once, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Net_Poll(uint32_t now_ms)
{
    if (s_scan_ready != 0U)
    {
        s_scan_ready = 0U;
        Task_Net_ScanCollect();
    }

    if (s_configured == 0U)
    {
        return;
    }

    if (s_want_connect != 0U)
    {
        uint32_t wait = (s_attempts <= TASK_NET_FAST_MAX) ? TASK_NET_FAST_MS
                                                          : TASK_NET_SLOW_MS;

        if ((s_attempts == 0U) || ((uint32_t)(now_ms - s_try_at) >= wait))
        {
            s_want_connect = 0U;
            s_try_at = now_ms;
            if (esp_wifi_connect() != ESP_OK)
            {
                s_want_connect = 1U;
            }
            else if (s_state != (uint8_t)TASK_NET_OK)
            {
                Task_Net_SetState(TASK_NET_CONNECTING);
            }
        }
    }

    /* The clock: started once there is an address to send from and not before,
       because the name of the server has to be resolved first. */
    if ((s_link_up != 0U) && (s_sntp_started == 0U))
    {
        esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(TASK_NET_NTP_SERVER);

        sntp_cfg.wait_for_sync = false;     /* the time is polled, not waited for */
        if (esp_netif_sntp_init(&sntp_cfg) == ESP_OK)
        {
            s_sntp_started = 1U;
            LOGI("ntp client started, server %s", TASK_NET_NTP_SERVER);
        }
        else
        {
            LOGW("the ntp client refused to start");
        }
    }

    if ((s_link_up != 0U) && ((uint32_t)(now_ms - s_rssi_at) >= TASK_NET_RSSI_MS))
    {
        wifi_ap_record_t ap;

        s_rssi_at = now_ms;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        {
            s_rssi = ap.rssi;
        }
        else
        {
            s_rssi = 0;
        }
    }
}

/**
 * @brief  Ask for one sweep of the surrounding access points.
 * @retval 1 when the sweep was started, 0 otherwise.
 */
uint8_t Task_Net_ScanStart(void)
{
    wifi_scan_config_t cfg;
    esp_err_t err;

    if (s_scan_state == TASK_NET_SCAN_RUNNING)
    {
        return 0U;
    }
    memset(&cfg, 0, sizeof(cfg));
    cfg.show_hidden = false;

    err = esp_wifi_scan_start(&cfg, false);        /* never blocks */
    if (err != ESP_OK)
    {
        /* ESP_ERR_WIFI_STATE is the usual one here: the radio is busy joining
           an access point and cannot sweep at the same time. */
        s_scan_state = TASK_NET_SCAN_FAILED;
        LOGW("the radio refused to sweep: %s", esp_err_to_name(err));
        return 0U;
    }
    s_scan_state = TASK_NET_SCAN_RUNNING;
    LOGI("sweep started");
    return 1U;
}

/**
 * @brief  State of the last sweep, TASK_NET_SCAN_xxx.
 */
uint8_t Task_Net_ScanState(void)
{
    return s_scan_state;
}

/**
 * @brief  Copy what the last sweep found, strongest first.
 * @param  out: destination buffer.
 * @param  max: capacity of that buffer in entries.
 * @retval number of entries copied, 0 when there is nothing to copy.
 */
uint8_t Task_Net_ScanResults(task_net_ap_t *out, uint8_t max)
{
    uint8_t n = s_scan_count;
    uint8_t i;

    if ((out == NULL) || (max == 0U))
    {
        return 0U;
    }
    if (n > max)
    {
        n = max;
    }
    for (i = 0U; i < n; i++)
    {
        out[i] = s_scan_aps[i];
    }
    return n;
}

/**
 * @brief  Says whether the clock of the gateway is real.
 * @retval 1 when the time came from an NTP server, 0 while it is the epoch.
 */
static uint8_t Task_Net_ClockSet(void)
{
    return (time(NULL) >= TASK_NET_CLOCK_FLOOR) ? 1U : 0U;
}

/**
 * @brief  Copy what the interface knows about the link.
 * @param  out: destination of the copy, untouched when it is NULL.
 */
void Task_Net_Get(task_net_info_t *out)
{
    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->state = (task_net_state_t)s_state;
    out->configured = s_configured;
    out->clock_set = Task_Net_ClockSet();
    out->rssi = s_rssi;
    memcpy(out->ssid, s_ssid, sizeof(out->ssid));
    memcpy(out->ip, s_ip, sizeof(out->ip));
}
