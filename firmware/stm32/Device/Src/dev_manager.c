/**
  ******************************************************************************
  * @file    dev_manager.c
  * @brief   Implementation of the device table and the walks over it.
  * @note    Registering a driver is one line in s_sensors[] or s_actuators[] below;
  *          nothing else in the project has to be kept in step by hand.
  ******************************************************************************
  */
#include "dev_manager.h"
#include "sen_dht22.h"
#include "act_beep.h"
#include <stdio.h>

/* ------------------------------------------------------------------
 * Device table: add one pointer here as soon as a new driver exists,
 * nothing else in the project has to change.
 *
 * The slot count is computed from the table itself, and a slot may stay
 * NULL: every walk below skips it. That is what lets a place holder sit
 * here while a driver is still missing, so adding a driver really is a
 * one line edit and no counter has to be kept in step by hand.
 * ------------------------------------------------------------------ */
static const Dev_Sensor_t *const s_sensors[] =
{
    &g_sen_dht22,
};

static const Dev_Actuator_t *const s_actuators[] =
{
    &g_act_beep,
};

#define SENSOR_SLOTS      ((uint8_t)(sizeof(s_sensors) / sizeof(s_sensors[0])))
#define ACTUATOR_SLOTS    ((uint8_t)(sizeof(s_actuators) / sizeof(s_actuators[0])))

/**
 * @brief  nth sensor that is really registered.
 * @param  nth: 0 based index, counted over the slots that are not NULL.
 * @retval the descriptor, or NULL when fewer sensors are registered.
 */
static const Dev_Sensor_t *SensorNth(uint8_t nth)
{
    uint8_t i;
    uint8_t seen = 0U;

    for (i = 0U; i < SENSOR_SLOTS; i++)
    {
        if (s_sensors[i] == NULL)
        {
            continue;
        }
        if (seen == nth)
        {
            return s_sensors[i];
        }
        seen++;
    }
    return NULL;
}

/**
 * @brief  nth actuator that is really registered, same rules as SensorNth().
 */
static const Dev_Actuator_t *ActuatorNth(uint8_t nth)
{
    uint8_t i;
    uint8_t seen = 0U;

    for (i = 0U; i < ACTUATOR_SLOTS; i++)
    {
        if (s_actuators[i] == NULL)
        {
            continue;
        }
        if (seen == nth)
        {
            return s_actuators[i];
        }
        seen++;
    }
    return NULL;
}

/**
 * @brief  Init every registered sensor and actuator.
 * @retval number of devices that failed to init.
 */
uint8_t Dev_Manager_Init(void)
{
    uint8_t i;
    uint8_t failed = 0U;

    for (i = 0U; i < SENSOR_SLOTS; i++)
    {
        if ((s_sensors[i] != NULL) && (s_sensors[i]->init != NULL))
        {
            if (s_sensors[i]->init() != 0U)
            {
                failed++;
                printf("[DEV ] %s init FAILED\r\n", s_sensors[i]->name);
            }
        }
    }
    for (i = 0U; i < ACTUATOR_SLOTS; i++)
    {
        if ((s_actuators[i] != NULL) && (s_actuators[i]->init != NULL))
        {
            if (s_actuators[i]->init() != 0U)
            {
                failed++;
                printf("[DEV ] %s init FAILED\r\n", s_actuators[i]->name);
            }
        }
    }
    return failed;
}

/**
 * @brief  Read every registered sensor once.
 * @param  out:        buffer that receives the values.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when every sensor failed.
 */
uint8_t Dev_Manager_Collect(Dev_Value_t *out, uint8_t max_values)
{
    uint8_t i;
    uint8_t used = 0U;
    uint8_t got;

    if ((out == NULL) || (max_values == 0U))
    {
        return 0U;
    }
    for (i = 0U; (i < SENSOR_SLOTS) && (used < max_values); i++)
    {
        if ((s_sensors[i] == NULL) || (s_sensors[i]->sample == NULL))
        {
            continue;
        }
        got = s_sensors[i]->sample(&out[used], (uint8_t)(max_values - used));
        if (got == 0U)
        {
            printf("[DEV ] %s sample failed\r\n", s_sensors[i]->name);
            continue;                 /* one dead sensor must not stop the rest */
        }
        used = (uint8_t)(used + got);
    }
    return used;
}

/**
 * @brief  Number of registered sensors.
 */
uint8_t Dev_Manager_SensorCount(void)
{
    uint8_t i;
    uint8_t n = 0U;

    for (i = 0U; i < SENSOR_SLOTS; i++)
    {
        if (s_sensors[i] != NULL)
        {
            n++;
        }
    }
    return n;
}

/**
 * @brief  Sensor descriptor by index, NULL when the index is out of range.
 */
const Dev_Sensor_t *Dev_Manager_SensorAt(uint8_t index)
{
    return SensorNth(index);
}

/**
 * @brief  Number of registered actuators.
 */
uint8_t Dev_Manager_ActuatorCount(void)
{
    uint8_t i;
    uint8_t n = 0U;

    for (i = 0U; i < ACTUATOR_SLOTS; i++)
    {
        if (s_actuators[i] != NULL)
        {
            n++;
        }
    }
    return n;
}

/**
 * @brief  Actuator descriptor by index, NULL when the index is out of range.
 */
const Dev_Actuator_t *Dev_Manager_ActuatorAt(uint8_t index)
{
    return ActuatorNth(index);
}

/**
 * @brief  Look one actuator up by its channel id.
 * @retval the descriptor or NULL when no actuator uses that channel.
 */
const Dev_Actuator_t *Dev_Manager_FindActuator(uint8_t ch)
{
    uint8_t i;

    for (i = 0U; i < ACTUATOR_SLOTS; i++)
    {
        if ((s_actuators[i] != NULL) && (s_actuators[i]->ch == ch))
        {
            return s_actuators[i];
        }
    }
    return NULL;
}

/**
 * @brief  Print the device table on the debug console.
 */
void Dev_Manager_List(void)
{
    uint8_t i;

    printf("[DEV ] %u sensor(s):\r\n", (unsigned)Dev_Manager_SensorCount());
    for (i = 0U; i < SENSOR_SLOTS; i++)
    {
        if (s_sensors[i] != NULL)
        {
            printf("[DEV ]   [%u] %s\r\n", (unsigned)i, s_sensors[i]->name);
        }
    }
    printf("[DEV ] %u actuator(s):\r\n", (unsigned)Dev_Manager_ActuatorCount());
    for (i = 0U; i < ACTUATOR_SLOTS; i++)
    {
        if (s_actuators[i] != NULL)
        {
            printf("[DEV ]   [%u] %s (ch 0x%02X)\r\n",
                   (unsigned)i, s_actuators[i]->name, (unsigned)s_actuators[i]->ch);
        }
    }
}
