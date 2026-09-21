/**
  ******************************************************************************
  * @file    task_comm.c
  * @brief   Implementation of the communication task.
  * @note    The device layer moves the bytes, this task only decides when the link
  *          is served and how its counters are reported.
  ******************************************************************************
  */
#include "task_comm.h"
#include "app_debug.h"
#include "dev_stm32.h"

#define TASK_COMM_DUMP_MS   10000U   /* period of the counter dump */

#ifdef APP_DEBUG
static const char *TAG = "TASK_COMM";   /* only used by the LOGx macros */
#endif

static uint32_t s_dump_tick;

/**
 * @brief  Bring up the link to the node behind the gateway.
 */
void Task_Comm_Init(void)
{
    dev_stm32_init();
    s_dump_tick = 0U;
}

/**
 * @brief  Serve the link once, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Comm_Poll(uint32_t now_ms)
{
    dev_stm32_poll();
    if ((uint32_t)(now_ms - s_dump_tick) >= TASK_COMM_DUMP_MS)
    {
        s_dump_tick = now_ms;
        Task_Comm_DumpStats();
    }
}

/**
 * @brief  Print the counters of the link.
 */
void Task_Comm_DumpStats(void)
{
#ifdef APP_DEBUG
    dev_stm32_stats_t st;

    dev_stm32_get_stats(&st);
    LOGI("link rx: ok=%lu bad=%lu",
         (unsigned long)st.frame_ok, (unsigned long)st.frame_bad);
    LOGI("link tx: ctrl=%lu query=%lu ack=%lu nak=%lu resend=%lu timeout=%lu",
         (unsigned long)st.tx_ctrl, (unsigned long)st.tx_query, (unsigned long)st.tx_ok,
         (unsigned long)st.tx_nak, (unsigned long)st.tx_resend, (unsigned long)st.tx_timeout);
#endif
}
