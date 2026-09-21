/**
  ******************************************************************************
  * @file    main.c
  * @brief   Application entry point of the Bloomcare gateway.
  * @note    The application logic lives in the Tasks/ layer:
  *            tasks/task_comm.c    - link to the STM32 node and its counters
  *            tasks/task_sensor.c  - the values the node reports plus its state
  *            tasks/task_gateway.c - CONTROL and QUERY, the outward face
  *          main.c stays the thin shell: start the layers, then poll them.
  ******************************************************************************
  */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_debug.h"
#include "task_comm.h"
#include "task_sensor.h"
#include "task_gateway.h"

#define APP_LOOP_DELAY_MS   10U

#ifdef APP_DEBUG
static const char *TAG = "MAIN";        /* only used by the LOGx macros */
#endif

/**
 * @brief  Millisecond time base of the main loop.
 * @note   Same clock the device layer uses for its timeouts, so one frame
 *         deadline is judged against one time base only.
 * @retval milliseconds since boot.
 */
static uint32_t App_Millis(void)
{
    return (uint32_t)xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS;
}

/**
 * @brief  Application entry point of the Bloomcare gateway.
 *
 *         For now the gateway relays the STM32F103 sensor node. The OLED
 *         display and the phone application link will be added as further
 *         tasks of the same loop.
 */
void app_main(void)
{
    uint32_t now;

    LOGI("Bloomcare gateway booting ...");
    Task_Comm_Init();                     /* UART plus frame parser          */
    Task_Sensor_Init();                   /* subscribe to the values         */
    Task_Gateway_Init();                  /* outward actions                 */
    while (1)
    {
        now = App_Millis();

        Task_Comm_Poll(now);              /* read the link, log the frames   */
        Task_Sensor_Poll(now);            /* online state of the node        */
        Task_Gateway_Poll(now);           /* pending CONTROL and QUERY work  */

        vTaskDelay(pdMS_TO_TICKS(APP_LOOP_DELAY_MS));
    }
}
