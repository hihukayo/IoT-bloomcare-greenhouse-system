/**
  ******************************************************************************
  * @file    task_comm.h
  * @brief   Task layer: communication task.
  * @note    Owns the link protocol instance and the board transport that feeds it, and
  *          reports the debug counters.
  ******************************************************************************
  */
#ifndef __TASK_COMM_H
#define __TASK_COMM_H

#include <stdint.h>

/* ------------------------------------------------------------------
 * Task layer: communication. It owns the link protocol instance and the
 * board transport that feeds it, and it reports the debug counters.
 * ------------------------------------------------------------------ */

/**
 * @brief  Bind the board transport to the protocol and start reception.
 */
void Task_Comm_Init(void);

/**
 * @brief  Serve the link once, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Comm_Poll(uint32_t now_ms);

/**
 * @brief  Print the counters of the board and the protocol layer.
 */
void Task_Comm_DumpStats(void);

#endif /* __TASK_COMM_H */
