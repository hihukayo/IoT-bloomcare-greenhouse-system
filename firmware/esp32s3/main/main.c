/**
  ******************************************************************************
  * @file    main.c
  * @brief   Application entry point of the Bloomcare gateway.
  * @note    The application logic lives in the Tasks/ layer, next to the WiFi
  *          module it reports to:
  *            tasks/task_comm.c    - link to the STM32 node and its counters
  *            tasks/task_sensor.c  - the values the node reports plus its state
  *            tasks/task_gateway.c - CONTROL and QUERY, the outward face
  *            tasks/task_display.c - the panel and the knob, the local interface
  *            wifi/task_net.c      - the access point and the clock it brings
  *          main.c stays the thin shell: start the layers, then poll them.
  ******************************************************************************
  */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_debug.h"
#include "task_comm.h"
#include "task_sensor.h"
#include "task_gateway.h"
#include "task_display.h"
#include "task_net.h"

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
 *         The gateway relays the STM32F103 sensor node and runs the local
 *         interface on its own panel. The phone application link will be
 *         added as one more task of the same loop.
 */
void app_main(void)
{
    uint32_t now;

    LOGI("Bloomcare gateway booting ...");
    Task_Comm_Init();                     /* UART plus frame parser          */
    Task_Sensor_Init();                   /* subscribe to the values         */
    Task_Gateway_Init();                  /* outward actions                 */
    /* The radio comes up before the panel. Bringing it up costs a few hundred
       milliseconds of calibration during which the main loop cannot serve the
       interface, and that pause is invisible while the panel is still dark but
       very visible once it is not. Nothing here touches the interface: the one
       thing the network task tells it is a state byte it only stores. */
    Task_Net_Init();                      /* the radio, and the clock        */
    Task_Display_Init();                  /* panel, LVGL and the knob        */
    while (1)
    {
        now = App_Millis();

        Task_Comm_Poll(now);              /* read the link, log the frames   */
        Task_Sensor_Poll(now);            /* online state of the node        */
        Task_Gateway_Poll(now);           /* pending CONTROL and QUERY work  */
        Task_Display_Poll(now);           /* what the interface asked for    */
        Task_Net_Poll(now);               /* the link, and the clock         */

        vTaskDelay(pdMS_TO_TICKS(APP_LOOP_DELAY_MS));
    }
}
