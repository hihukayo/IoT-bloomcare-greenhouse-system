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
 *   Task_Display_Init      brings up the panel, LVGL and the knob
 *   Task_Display_Poll      serves the transactions the interface asked for
 *   Task_Display_SetNet    feeds in what the network link is doing
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

/**
 * @brief  State of the network link.
 * @note   The link of the gateway is the one thing of the status bar that no
 *         sensor of the node can answer, so it is handed in from outside.
 *         Until something calls Task_Display_SetNet() the bars keep reporting
 *         the unknown state.
 */
typedef enum
{
    TASK_DISPLAY_NET_UNKNOWN = 0U,  /**< nothing has reported yet */
    TASK_DISPLAY_NET_CONNECTING,    /**< looking for the access point */
    TASK_DISPLAY_NET_OK,            /**< associated, the broker answers */
    TASK_DISPLAY_NET_ERR            /**< gave up, or the link dropped */
} task_display_net_t;

/**
 * @brief  Tell the interface what the network link is doing.
 * @param  state: one of TASK_DISPLAY_NET_xxx.
 * @note   Safe to call from any task: it writes one byte, and the status bars
 *         pick it up on their own next refresh.
 */
void Task_Display_SetNet(task_display_net_t state);

#endif /* __TASK_DISPLAY_H */
