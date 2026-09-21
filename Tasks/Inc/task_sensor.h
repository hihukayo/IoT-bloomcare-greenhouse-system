/**
  ******************************************************************************
  * @file    task_sensor.h
  * @brief   Task layer: sensor acquisition task.
  * @note    Walks the device table and turns whatever comes back into the data items
  *          of the link protocol.
  ******************************************************************************
  */
#ifndef __TASK_SENSOR_H
#define __TASK_SENSOR_H

#include <stdint.h>
#include "comp_link.h"


/* ------------------------------------------------------------------
 * Task layer: sensor acquisition. It walks the device manager and turns
 * whatever comes back into the data items of the link protocol.
 * ------------------------------------------------------------------ */

/**
 * @brief  Arm the sampling timers, call it once after the devices are up.
 */
void Task_Sensor_Init(void);

/**
 * @brief  Set the reporting period, used by the CONTROL command handler.
 * @param  ms: new period in milliseconds.
 */
void Task_Sensor_SetPeriod(uint32_t ms);

/**
 * @brief  Current reporting period in milliseconds.
 */
uint32_t Task_Sensor_GetPeriod(void);

/**
 * @brief  Sample and report when the period has elapsed.
 * @param  now_ms: current millisecond tick.
 */
void Task_Sensor_Poll(uint32_t now_ms);

/**
 * @brief  Answer a QUERY of the gateway.
 * @note   The values come from the cache the periodic sampling fills, so a query
 *         never forces a second read inside the interval the sensor needs between
 *         two shots. Only when nothing is cached yet, one sample is taken.
 * @param  id:  item id asked for, 0 means every item the node has.
 * @param  out: buffer that receives the answer items.
 * @param  max: capacity of that buffer in items.
 * @retval number of items written, 0 when nothing can be answered.
 */
uint8_t Task_Sensor_OnQuery(uint8_t id, Link_Item_t *out, uint8_t max);


#endif /* __TASK_SENSOR_H */
