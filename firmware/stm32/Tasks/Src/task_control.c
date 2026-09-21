/**
  ******************************************************************************
  * @file    task_control.c
  * @brief   Implementation of the control task.
  * @note    System items are handled here; everything else is routed to an actuator
  *          through Dev_Manager_FindActuator().
  ******************************************************************************
  */
#include "task_control.h"
#include "comp_link.h"
#include "dev_manager.h"
#include "task_sensor.h"
#include <stdio.h>

#define TASK_CONTROL_MIN_MS   1000U    /* accepted range of the CONTROL command */
#define TASK_CONTROL_MAX_MS   60000U

/**
 * @brief  Map a protocol item id onto a device channel.
 * @note   Actuator ids live in 0x80~0xEF, add one case per actuator as
 *         soon as the driver exists in Device/Actuators/.
 */
static uint8_t ItemToChannel(uint8_t id)
{
    (void)id;
    return 0U;                        /* no actuator wired yet */
}

/**
 * @brief  Prepare the control task, call it once during start-up.
 */
void Task_Control_Init(void)
{
    /* Nothing to prepare yet, the command handler is registered by task_comm. */
}

/**
 * @brief  Run one data item of a CONTROL frame.
 * @param  id:    item id, see LINK_ID_xxx.
 * @param  value: decoded value of that item.
 * @retval LINK_OK when it was applied, otherwise a LINK_NAK_xxx reason.
 */
static uint8_t ControlExecItem(uint8_t id, int32_t value)
{
    uint8_t ch;
    const Dev_Actuator_t *act;

    if (id == LINK_ID_REPORT_MS)          /* system item: reporting period */
    {
        if ((value < (int32_t)TASK_CONTROL_MIN_MS) || (value > (int32_t)TASK_CONTROL_MAX_MS))
        {
            return LINK_NAK_BAD_PARAM;
        }
        Task_Sensor_SetPeriod((uint32_t)value);
        printf("[CMD ] report period = %lu ms\r\n", (unsigned long)value);
        return LINK_OK;
    }
    /* Everything else has to be an actuator. */
    ch = ItemToChannel(id);
    act = Dev_Manager_FindActuator(ch);
    if ((act == NULL) || (act->set == NULL))
    {
        return LINK_NAK_UNSUPPORTED;
    }
    if (act->set(value) != 0U)
    {
        return LINK_NAK_EXEC_FAIL;
    }
    printf("[CMD ] %s = %ld\r\n", act->name, (long)value);
    return LINK_OK;
}

/**
 * @brief  Execute one command that arrived from the gateway.
 * @note   Every item of the frame is applied, one bad item does not hold back the
 *         others. The frame is refused with the first reason that failed, because
 *         a NAK carries a reason and not the id that caused it.
 */
uint8_t Task_Control_OnCommand(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t off = 0U;
    uint8_t ret = LINK_OK;
    uint8_t item_ret;
    int32_t value;

    if (cmd != LINK_CMD_CONTROL)
    {
        return LINK_NAK_UNSUPPORTED;
    }
    if ((payload == NULL) || (len < LINK_ITEM_LEN))
    {
        return LINK_NAK_BAD_PARAM;
    }
    while ((uint16_t)(off + LINK_ITEM_LEN) <= (uint16_t)len)
    {
        value = (int32_t)((uint32_t)payload[off + 1U] |
                          ((uint32_t)payload[off + 2U] << 8) |
                          ((uint32_t)payload[off + 3U] << 16) |
                          ((uint32_t)payload[off + 4U] << 24));
        item_ret = ControlExecItem(payload[off], value);
        if ((item_ret != LINK_OK) && (ret == LINK_OK))
        {
            ret = item_ret;                       /* keep the first failure */
        }
        off = (uint8_t)(off + LINK_ITEM_LEN);
    }
    return ret;
}
