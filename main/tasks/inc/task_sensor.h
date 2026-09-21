/**
  ******************************************************************************
  * @file    task_sensor.h
  * @brief   Task layer: sensor values task.
  * @note    Keeps the last values the node reported, so the OLED display and the
  *          phone application can read them at any time instead of forcing a
  *          read of the sensor itself.
  ******************************************************************************
  */
#ifndef __TASK_SENSOR_H
#define __TASK_SENSOR_H

#include <stdint.h>
#include "dev_stm32.h"

/** Room for the values of the node plus a few more sensors later on. */
#define TASK_SENSOR_MAX_VALUES  8U

/** State of the node as seen from the gateway. */
#define TASK_SENSOR_OFFLINE     0U
#define TASK_SENSOR_ONLINE      1U

/* ------------------------------------------------------------------
 * Task layer: sensor values.
 *
 * The node pushes its values on its own, so this task subscribes to
 * the device layer and keeps a small cache: whoever wants a value
 * reads the cache, it never has to wait for the next report.
 * ------------------------------------------------------------------ */

/**
 * @brief  Subscribe to the node and arm the state machine.
 */
void Task_Sensor_Init(void);

/**
 * @brief  Refresh the online state, call it every pass of the main loop.
 * @param  now_ms: current millisecond tick.
 */
void Task_Sensor_Poll(uint32_t now_ms);

/**
 * @brief  Look one cached value up.
 * @param  id:    item ID, see LINK_ID_xxx.
 * @param  value: destination of the value, only written when it is known.
 * @retval 1 when the item is known, 0 otherwise.
 */
uint8_t Task_Sensor_Get(uint8_t id, int32_t *value);

/**
 * @brief  Copy every cached item.
 * @param  out: destination buffer.
 * @param  max: capacity of that buffer in items.
 * @retval number of items copied.
 */
uint8_t Task_Sensor_Snapshot(dev_item_t *out, uint8_t max);

/**
 * @brief  State of the node, TASK_SENSOR_ONLINE or TASK_SENSOR_OFFLINE.
 */
uint8_t Task_Sensor_IsOnline(void);

/**
 * @brief  Set how long the node may stay silent before it counts as offline.
 * @param  ms: new timeout, values below 1000 are raised to 1000.
 */
void Task_Sensor_SetTimeout(uint32_t ms);

/**
 * @brief  Current offline timeout in milliseconds.
 */
uint32_t Task_Sensor_GetTimeout(void);

/**
 * @brief  Number of frames received from the node since boot.
 */
uint32_t Task_Sensor_FrameCount(void);

#endif /* __TASK_SENSOR_H */
