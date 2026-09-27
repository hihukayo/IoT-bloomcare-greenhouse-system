/**
  ******************************************************************************
  * @file    task_display.h
  * @brief   Task layer: the local display task.
  * @note    Owns the panel, the LVGL port and the knob, which together are the
  *          whole local user interface of the gateway.
  * @note    It reads the cache of the sensor values task instead of touching the
  *          node itself, and it never runs a transaction on its own: the ones it
  *          wants are left for Task_Display_Poll(), because a CONTROL or QUERY
  *          blocks for up to a second and that is far too long to keep the LVGL
  *          task waiting.
  ******************************************************************************
  */
#ifndef __TASK_DISPLAY_H
#define __TASK_DISPLAY_H

#include <stdint.h>

/* ------------------------------------------------------------------
 * Task layer: local display.
 *
 *   Task_Display_Init  brings up the panel, LVGL and the knob
 *   Task_Display_Poll  serves the transactions the interface asked for
 *
 * The interface itself runs in the LVGL task, this task only owns the
 * handover between the two.
 * ------------------------------------------------------------------ */

/**
 * @brief  Bring up the panel, the graphics library and the knob.
 * @note   Without a panel there is nothing to serve, so a failure of any step
 *         leaves this task idle instead of blocking the rest of the boot.
 */
void Task_Display_Init(void);

/**
 * @brief  Serve the display task once, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Display_Poll(uint32_t now_ms);

#endif /* __TASK_DISPLAY_H */
