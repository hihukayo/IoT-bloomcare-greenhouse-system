/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Implementation of the sensor values task.
  * @note    The node pushes its values, so the cache is filled from the event of the
  *          device layer and the poll only watches the online state.
  ******************************************************************************
  */
#include "task_sensor.h"
#include <stddef.h>
#include "app_debug.h"
#include "link_protocol.h"

#define TASK_SENSOR_TIMEOUT_MS  10000U   /* 5 x the 2 s default report period */
#define TASK_SENSOR_MIN_TIMEOUT  1000U   /* floor accepted by the setter      */

#ifdef APP_DEBUG
static const char *TAG = "TASK_SENSOR";   /* only used by the LOGx macros */
#endif

/* Values of the last report, already in wire format: item ID plus value x100.
   The device layer logs every single one of them, so this task stays quiet
   about the numbers and only reports a change of the online state. */
static dev_item_t s_cache[TASK_SENSOR_MAX_VALUES];
static uint8_t    s_cache_count;
static uint32_t   s_timeout_ms;
static uint32_t   s_last_rx_ms;
static uint32_t   s_frames;
static uint8_t    s_got_frame;      /* set by the event, consumed by the poll */
static uint8_t    s_online;

/**
 * @brief  Handle a frame the node pushed on its own.
 * @param  cmd:   command code, LINK_CMD_REPORT or LINK_CMD_HEARTBEAT.
 * @param  items: decoded data items.
 * @param  count: number of items, 0 for a heartbeat.
 * @param  user:  unused.
 */
static void Task_Sensor_OnEvent(uint8_t cmd, const dev_item_t *items, uint8_t count, void *user)
{
    uint8_t i;

    (void)user;
    s_got_frame = 1U;                /* a heartbeat counts as liveness as well */
    if (cmd != LINK_CMD_REPORT)
    {
        return;
    }
    s_cache_count = 0U;
    for (i = 0U; (i < count) && (i < (uint8_t)TASK_SENSOR_MAX_VALUES); i++)
    {
        if (items[i].id == LINK_ID_ERRCODE)
        {
            /* The node failed to read its own sensor, so there is no value to
               cache and the old ones keep their timestamps. */
            LOGW("node reports a sensor error, code %ld", (long)items[i].value);
            continue;
        }
        s_cache[s_cache_count] = items[i];
        s_cache_count++;
    }
}

/**
 * @brief  Subscribe to the node and arm the state machine.
 */
void Task_Sensor_Init(void)
{
    s_cache_count = 0U;
    s_timeout_ms = TASK_SENSOR_TIMEOUT_MS;
    s_last_rx_ms = 0U;
    s_frames = 0U;
    s_got_frame = 0U;
    s_online = TASK_SENSOR_OFFLINE;
    dev_stm32_set_event_handler(Task_Sensor_OnEvent, NULL);
}

/**
 * @brief  Refresh the online state, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Sensor_Poll(uint32_t now_ms)
{
    if (s_got_frame != 0U)
    {
        s_got_frame = 0U;
        s_last_rx_ms = now_ms;
        s_frames++;
        if (s_online == TASK_SENSOR_OFFLINE)
        {
            s_online = TASK_SENSOR_ONLINE;
            LOGI("node %u is online", (unsigned)LINK_ADDR_NODE1);
        }
    }
    if ((s_online == TASK_SENSOR_ONLINE) && ((uint32_t)(now_ms - s_last_rx_ms) >= s_timeout_ms))
    {
        s_online = TASK_SENSOR_OFFLINE;
        LOGW("node %u went offline, silent for %lu ms",
             (unsigned)LINK_ADDR_NODE1, (unsigned long)(uint32_t)(now_ms - s_last_rx_ms));
    }
}

/**
 * @brief  Look one cached value up.
 * @param  id:    item ID, see LINK_ID_xxx.
 * @param  value: destination of the value, only written when it is known.
 * @retval 1 when the item is known, 0 otherwise.
 */
uint8_t Task_Sensor_Get(uint8_t id, int32_t *value)
{
    uint8_t i;

    if (value == NULL)
    {
        return 0U;
    }
    for (i = 0U; i < s_cache_count; i++)
    {
        if (s_cache[i].id == id)
        {
            *value = s_cache[i].value;
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief  Copy every cached item.
 * @param  out: destination buffer.
 * @param  max: capacity of that buffer in items.
 * @retval number of items copied.
 */
uint8_t Task_Sensor_Snapshot(dev_item_t *out, uint8_t max)
{
    uint8_t i;
    uint8_t n = 0U;

    if ((out == NULL) || (max == 0U))
    {
        return 0U;
    }
    for (i = 0U; (i < s_cache_count) && (n < max); i++)
    {
        out[n] = s_cache[i];
        n++;
    }
    return n;
}

/**
 * @brief  State of the node, TASK_SENSOR_ONLINE or TASK_SENSOR_OFFLINE.
 */
uint8_t Task_Sensor_IsOnline(void)
{
    return s_online;
}

/**
 * @brief  Set how long the node may stay silent before it counts as offline.
 * @param  ms: new timeout, values below 1000 are raised to 1000.
 */
void Task_Sensor_SetTimeout(uint32_t ms)
{
    s_timeout_ms = (ms < TASK_SENSOR_MIN_TIMEOUT) ? TASK_SENSOR_MIN_TIMEOUT : ms;
}

/**
 * @brief  Current offline timeout in milliseconds.
 */
uint32_t Task_Sensor_GetTimeout(void)
{
    return s_timeout_ms;
}

/**
 * @brief  Number of frames received from the node since boot.
 */
uint32_t Task_Sensor_FrameCount(void)
{
    return s_frames;
}
