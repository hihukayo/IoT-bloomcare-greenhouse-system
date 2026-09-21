/**
  ******************************************************************************
  * @file    task_comm.c
  * @brief   Implementation of the communication task.
  * @note    This is the only place that knows which board functions feed the protocol,
  *          which is what keeps Components/ portable.
  ******************************************************************************
  */
#include "task_comm.h"
#include "comp_link.h"
#include "bsp_uart.h"
#include "task_control.h"
#include "task_sensor.h"

#include <stdio.h>

#define TASK_COMM_DUMP_MS   10000U   /* period of the debug counter dump */

/* ------------------------------------------------------------------
 * Glue between the protocol layer and the board: comp_link only knows
 * these four function pointers, so it never sees a HAL handle.
 * ------------------------------------------------------------------ */
static void     LinkPort_RxStart(void)
{
    BSP_Uart2_Init();                          /* arm the DMA ring + IDLE */
}

static uint16_t LinkPort_RxTake(uint8_t *dst, uint16_t max)
{
    return BSP_Uart2_RxTake(dst, max);
}

static uint8_t  LinkPort_IdleTake(void)
{
    return BSP_Uart2_IdleTake();
}

static uint8_t  LinkPort_Tx(const uint8_t *data, uint16_t len)
{
    return BSP_Uart2_Tx(data, len);
}

static const Link_Transport_t s_transport =
{
    LinkPort_RxStart,
    LinkPort_RxTake,
    LinkPort_IdleTake,
    LinkPort_Tx
};

static uint32_t s_dump_tick;

/**
 * @brief  Bind the board transport to the protocol and start reception.
 */
void Task_Comm_Init(void)
{
    Link_Init(&s_transport);                   /* resets state and arms USART2 */
    Link_SetCmdHandler(Task_Control_OnCommand);
    Link_SetQueryHandler(Task_Sensor_OnQuery); /* QUERY is answered from the cache */

    s_dump_tick = 0U;
}

/**
 * @brief  Serve the link once, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Comm_Poll(uint32_t now_ms)
{
    Link_Poll(now_ms);

#if defined(LINK_DEBUG) || defined(BSP_UART_DEBUG)
    if ((uint32_t)(now_ms - s_dump_tick) >= TASK_COMM_DUMP_MS)
    {
        s_dump_tick = now_ms;
        Task_Comm_DumpStats();
    }
#endif
}

/**
 * @brief  Print the counters of the board and the protocol layer.
 */
void Task_Comm_DumpStats(void)
{
#ifdef BSP_UART_DEBUG
    printf("[STAT] rx=%lu drop=%lu idle=%lu uerr=%lu\r\n",
           (unsigned long)g_bsp_rx_bytes, (unsigned long)g_bsp_rx_drop,
           (unsigned long)g_bsp_idle_evt, (unsigned long)g_bsp_uart_err);
#endif
#ifdef LINK_DEBUG
    printf("[STAT] frm ok=%lu bad=%lu other=%lu cmd=%lu dup=%lu\r\n",
           (unsigned long)g_link_frame_ok, (unsigned long)g_link_frame_bad,
           (unsigned long)g_link_frame_other, (unsigned long)g_link_cmd_rx,
           (unsigned long)g_link_dup_rx);

    printf("[STAT] ack_tx=%lu nak_tx=%lu ack_rx=%lu nak_rx=%lu retry=%lu timeout=%lu\r\n",
           (unsigned long)g_link_ack_tx, (unsigned long)g_link_nak_tx,
           (unsigned long)g_link_ack_rx, (unsigned long)g_link_nak_rx,
           (unsigned long)g_link_retry_tx, (unsigned long)g_link_timeout_tx);
#endif
}
