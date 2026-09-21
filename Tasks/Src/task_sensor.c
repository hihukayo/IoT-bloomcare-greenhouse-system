/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Implementation of the acquisition task.
  * @note    The rate is kept by comparing HAL ticks, so nothing here blocks the main
  *          loop and one dead sensor never stops the others.
  ******************************************************************************
  */
#include "task_sensor.h"
#include "dev_manager.h"
#include "comp_link.h"
#include <stdio.h>

#define TASK_SENSOR_DEFAULT_MS   2000U   /* DHT11 wants >= 2s between reads */
#define TASK_SENSOR_STARTUP_MS   1000U   /* DHT11 needs ~1s to settle       */
#define TASK_SENSOR_MAX_VALUES   8U      /* room for a few more sensors     */

static uint32_t s_period_ms = TASK_SENSOR_DEFAULT_MS;
static uint32_t s_start_tick;
static uint32_t s_report_tick;
static uint32_t s_ok_cnt;
static uint32_t s_err_cnt;
static uint8_t  s_started;

/* Last good sample, already scaled and mapped onto protocol item IDs. The QUERY
   answer is built from here, see Task_Sensor_OnQuery(). */
static Link_Item_t s_cache[TASK_SENSOR_MAX_VALUES];
static uint8_t     s_cache_count;


/**
 * @brief  Map a device channel onto the item id used on the wire.
 * @note   This is the only place where the two worlds meet, so the
 *         Device layer stays independent of the protocol.
 */
static uint8_t ChannelToItem(uint8_t ch)
{
    switch (ch)
    {
    case DEV_CH_TEMP:  return LINK_ID_TEMP;
    case DEV_CH_HUMI:  return LINK_ID_HUMI;
    default:           return 0U;
    }
}

/**
 * @brief  Human readable name of a channel, for the debug log only.
 */
static const char *ChannelName(uint8_t ch)
{
    switch (ch)
    {
    case DEV_CH_TEMP:  return "Temp";
    case DEV_CH_HUMI:  return "Humi";
    case DEV_CH_SOIL:  return "Soil";
    case DEV_CH_LIGHT: return "Light";
    default:           return "?";
    }
}

/**
 * @brief  Unit of a channel, for the debug log only.
 */
static const char *ChannelUnit(uint8_t ch)
{
    switch (ch)
    {
    case DEV_CH_TEMP:  return "C";
    case DEV_CH_HUMI:  return "%RH";
    default:           return "";
    }
}

/**
 * @brief  Read every sensor once and store the result in the cache.
 * @retval number of items in the cache, 0 when every sensor failed.
 */
static uint8_t Task_Sensor_Sample(void)
{
    Dev_Value_t values[TASK_SENSOR_MAX_VALUES];
    uint8_t count;
    uint8_t i;
    uint8_t id;

    s_cache_count = 0U;
    count = Dev_Manager_Collect(values, (uint8_t)TASK_SENSOR_MAX_VALUES);
    if (count == 0U)
    {
        return 0U;
    }
    s_ok_cnt++;
    for (i = 0U; i < count; i++)
    {
        id = ChannelToItem(values[i].ch);
        if (id == 0U)
        {
            continue;                            /* channel nobody subscribed to */
        }
        printf("[OK  #%lu] %s = %ld %s\r\n",
               (unsigned long)s_ok_cnt, ChannelName(values[i].ch),
               (long)values[i].value, ChannelUnit(values[i].ch));
        s_cache[s_cache_count].id    = id;
        s_cache[s_cache_count].value = values[i].value * 100;  /* wire format is x100 */
        s_cache_count++;
    }
    return s_cache_count;
}

/**
 * @brief  Read every sensor once and report the values to the gateway.
 */
static void Task_Sensor_Report(void)
{
    Link_Item_t fault;
    uint8_t n;
    uint8_t ret;

    n = Task_Sensor_Sample();
    if (n == 0U)
    {
        s_err_cnt++;
        printf("[ERR #%lu] sensor read failed\r\n", (unsigned long)s_err_cnt);
        fault.id    = LINK_ID_ERRCODE;           /* the fault goes on the same link */
        fault.value = LINK_ERR_DHT11;
        (void)Link_SendReport(&fault, 1U);
        return;
    }
    ret = Link_SendReport(s_cache, n);
    printf("[LINK] report %s\r\n", (ret == LINK_RET_OK) ? "sent" : "FAILED");
}

/**
 * @brief  Arm the sampling timers, call it once after the devices are up.
 */
void Task_Sensor_Init(void)
{
    s_period_ms = TASK_SENSOR_DEFAULT_MS;
    s_start_tick = 0U;
    s_report_tick = 0U;
    s_ok_cnt = 0U;
    s_err_cnt = 0U;
    s_started = 0U;
    s_cache_count = 0U;
}

/**
 * @brief  Set the reporting period, used by the CONTROL command handler.
 */
void Task_Sensor_SetPeriod(uint32_t ms)
{
    s_period_ms = ms;
}

/**
 * @brief  Current reporting period in milliseconds.
 */
uint32_t Task_Sensor_GetPeriod(void)
{
    return s_period_ms;
}

/**
 * @brief  Sample and report when the period has elapsed.
 */
void Task_Sensor_Poll(uint32_t now_ms)
{
    if (s_started == 0U)
    {
        s_started = 1U;
        s_start_tick = now_ms;
        s_report_tick = now_ms;
    }
    if ((uint32_t)(now_ms - s_start_tick) < TASK_SENSOR_STARTUP_MS)
    {
        return;                                  /* let the sensors settle first */
    }
    if ((uint32_t)(now_ms - s_report_tick) >= s_period_ms)
    {
        s_report_tick = now_ms;
        Task_Sensor_Report();
    }
}
/**
 * @brief  Answer a QUERY of the gateway from the cache.
 * @param  id:  item id asked for, 0 means every item the node has.
 * @param  out: buffer that receives the answer items.
 * @param  max: capacity of that buffer in items.
 * @retval number of items written, 0 when nothing can be answered.
 */
uint8_t Task_Sensor_OnQuery(uint8_t id, Link_Item_t *out, uint8_t max)
{
    uint8_t i;
    uint8_t n = 0U;

    if ((out == NULL) || (max == 0U))
    {
        return 0U;
    }
    if (s_cache_count == 0U)
    {
        (void)Task_Sensor_Sample();              /* nothing cached yet, read now */
    }
    for (i = 0U; i < s_cache_count; i++)
    {
        if ((id != 0U) && (s_cache[i].id != id))
        {
            continue;                            /* only the item that was asked for */
        }
        if (n >= max)
        {
            break;
        }
        out[n] = s_cache[i];
        n++;
    }
    return n;
}
