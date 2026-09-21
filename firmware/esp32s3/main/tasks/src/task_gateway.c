/**
  ******************************************************************************
  * @file    task_gateway.c
  * @brief   Implementation of the gateway task.
  * @note    Everything that leaves the gateway towards the node goes through here,
  *          so the SEQ + ACK stop and wait stays in one place.
  ******************************************************************************
  */
#include "task_gateway.h"
#include <stddef.h>
#include "app_debug.h"
#include "link_protocol.h"
#include "task_sensor.h"

#define TASK_GATEWAY_MIN_MS     1000U    /* accepted range of the report period */
#define TASK_GATEWAY_MAX_MS     60000U
#define TASK_GATEWAY_TIMEOUT_K  5U       /* offline timeout = K x report period */

/* Set to 1 to send one QUERY every 30 s. The node has to answer with a REPORT,
   which is how the sending side is checked by hand without a phone app. */
#define TASK_GATEWAY_SELFTEST   0

#if TASK_GATEWAY_SELFTEST
#ifdef APP_DEBUG
static const char *TAG = "TASK_GATEWAY";   /* only used by the LOGx macros */
#endif
static uint32_t s_test_tick;
#endif

/**
 * @brief  Prepare the gateway task.
 */
void Task_Gateway_Init(void)
{
#if TASK_GATEWAY_SELFTEST
    s_test_tick = 0U;
#endif
}

/**
 * @brief  Serve the gateway task once, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Gateway_Poll(uint32_t now_ms)
{
    (void)now_ms;
#if TASK_GATEWAY_SELFTEST
    if ((uint32_t)(now_ms - s_test_tick) >= 30000U)
    {
        dev_item_t items[LINK_QUERY_MAX_ITEMS];
        uint8_t    got = 0U;
        esp_err_t  err;

        s_test_tick = now_ms;
        err = Task_Gateway_RequestValues(0U, items, (uint8_t)LINK_QUERY_MAX_ITEMS, &got);
        LOGI("selftest: query -> %s, %u item(s)", esp_err_to_name(err), (unsigned)got);
    }
#endif
}

/**
 * @brief  Tell the node how often it has to report its values.
 * @param  ms: period in milliseconds, accepted range 1000 .. 60000.
 * @retval see task_gateway.h.
 */
esp_err_t Task_Gateway_SetReportPeriod(uint32_t ms)
{
    esp_err_t err;

    if ((ms < TASK_GATEWAY_MIN_MS) || (ms > TASK_GATEWAY_MAX_MS))
    {
        return ESP_ERR_INVALID_ARG;
    }
    err = dev_stm32_send_control(LINK_ID_REPORT_MS, (int32_t)ms);
    if (err == ESP_OK)
    {
        /* From now on the node reports that rarely, so a silent node has to be
           judged against the period that was just agreed on. */
        Task_Sensor_SetTimeout(ms * TASK_GATEWAY_TIMEOUT_K);
    }
    return err;
}

/**
 * @brief  Ask the node for its values without waiting for the next report.
 * @param  id:  item ID to read, 0 asks for every item the node has.
 * @param  out: buffer that receives the items of the answer.
 * @param  max: capacity of that buffer in items.
 * @param  got: number of items received, may be NULL.
 * @retval see dev_stm32_send_query().
 */
esp_err_t Task_Gateway_RequestValues(uint8_t id, dev_item_t *out, uint8_t max, uint8_t *got)
{
    return dev_stm32_send_query(id, out, max, got);
}
