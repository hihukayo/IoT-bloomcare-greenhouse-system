/**
  ******************************************************************************
  * @file    task_control.h
  * @brief   Task layer: control task.
  * @note    Executes the commands that arrive from the gateway and drives the
  *          actuators through the device table.
  ******************************************************************************
  */
#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdint.h>

/* ------------------------------------------------------------------
 * Task layer: control. It executes the commands that arrive from the
 * gateway and drives the actuators through the device manager.
 * ------------------------------------------------------------------ */

/**
 * @brief  Prepare the control task, call it once during start-up.
 */
void Task_Control_Init(void);

/**
 * @brief  Execute one command that arrived from the gateway.
 * @param  cmd:     command code of the frame.
 * @param  payload: data items, ID(1B) + int32(4B, little endian) each.
 * @param  len:     payload length in bytes.
 * @retval LINK_OK when the command was executed, otherwise a LINK_NAK_xxx reason.
 */
uint8_t Task_Control_OnCommand(uint8_t cmd, const uint8_t *payload, uint8_t len);

#endif /* __TASK_CONTROL_H */
