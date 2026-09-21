/**
  ******************************************************************************
  * @file    task_comm.h
  * @brief   Task layer: communication task.
  * @note    Owns the link to the STM32 node and reports its counters.
  ******************************************************************************
  */
#ifndef __TASK_COMM_H
#define __TASK_COMM_H

#include <stdint.h>

/* ------------------------------------------------------------------
 * Task layer: communication.
 *
 * This is the only place of the Tasks layer that knows which device
 * carries the link, so the other tasks keep working when the link
 * moves to another UART or to another node.
 * ------------------------------------------------------------------ */

/**
 * @brief  Bring up the link to the node behind the gateway.
 */
void Task_Comm_Init(void);

/**
 * @brief  Serve the link once, call it every pass of the main loop.
 * @note   Blocks for one short slice, see DEV_STM32_READ_TIMEOUT.
 * @param  now_ms: current millisecond tick.
 */
void Task_Comm_Poll(uint32_t now_ms);

/**
 * @brief  Print the counters of the link.
 */
void Task_Comm_DumpStats(void);

#endif /* __TASK_COMM_H */
