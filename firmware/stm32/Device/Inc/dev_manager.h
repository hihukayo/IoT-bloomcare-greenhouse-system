/**
  ******************************************************************************
  * @file    dev_manager.h
  * @brief   Device layer: what one sensor and one actuator look like, plus the table.
  * @note    The Tasks layer walks this table instead of calling a chip driver by name,
  *          so a new chip is added without touching any task.
  ******************************************************************************
  */
#ifndef __DEV_MANAGER_H
#define __DEV_MANAGER_H

#include <stdint.h>

/* ------------------------------------------------------------------
 * Device layer: one driver per piece of hardware, plus this manager.
 *
 * A driver only knows how to talk to its own chip: it does not format
 * frames, does not print logs and does not know the link protocol.
 * The manager keeps one array of sensors and one array of actuators,
 * so the Tasks layer can walk them without knowing which chip is which.
 *
 * Channel ids are device level names. The Tasks layer maps them onto
 * the item ids of the protocol (Components/comp_link.h), which keeps
 * this layer independent of the wire format.
 *
 * A driver scales its own value to hundredths of its unit, because the
 * resolution differs per chip: a DHT22 resolves 0.1 C and 0.1 %RH while
 * a soil probe may only resolve 1 %. Doing it here keeps the Tasks
 * layer free of a per chip factor.
 * ------------------------------------------------------------------ */

#define DEV_CH_TEMP     0x01U
#define DEV_CH_HUMI     0x02U
#define DEV_CH_SOIL     0x03U
#define DEV_CH_LIGHT    0x04U

/** One value produced by a sensor, in the unit of that sensor. */
typedef struct
{
    uint8_t ch;      /* DEV_CH_xxx                                */
    int32_t value;   /* unit x 100, so 2340 is 23.40 of that unit */
} Dev_Value_t;

/** A sensor: one device, possibly several values per shot (DHT22 = T + RH). */
typedef struct
{
    const char *name;
    uint8_t (*init)(void);
    uint8_t (*sample)(Dev_Value_t *out, uint8_t max_values);   /* 0 = failed */
} Dev_Sensor_t;

/** An actuator: one device, driven by one channel id. */
typedef struct
{
    const char *name;
    uint8_t ch;
    uint8_t (*init)(void);
    uint8_t (*set)(int32_t value);
} Dev_Actuator_t;

/**
 * @brief  Init every registered sensor and actuator.
 * @retval number of devices that failed to init.
 */
uint8_t Dev_Manager_Init(void);

/**
 * @brief  Read every registered sensor once.
 * @param  out:        buffer that receives the values.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when every sensor failed.
 */
uint8_t Dev_Manager_Collect(Dev_Value_t *out, uint8_t max_values);

/**
 * @brief  Number of registered sensors.
 */
uint8_t Dev_Manager_SensorCount(void);

/**
 * @brief  Sensor descriptor by index, NULL when the index is out of range.
 */
const Dev_Sensor_t *Dev_Manager_SensorAt(uint8_t index);

/**
 * @brief  Number of registered actuators.
 */
uint8_t Dev_Manager_ActuatorCount(void);

/**
 * @brief  Actuator descriptor by index, NULL when the index is out of range.
 */
const Dev_Actuator_t *Dev_Manager_ActuatorAt(uint8_t index);

/**
 * @brief  Look one actuator up by its channel id.
 * @retval the descriptor or NULL when no actuator uses that channel.
 */
const Dev_Actuator_t *Dev_Manager_FindActuator(uint8_t ch);

/**
 * @brief  Print the device table on the debug console.
 */
void Dev_Manager_List(void);

#endif /* __DEV_MANAGER_H */
