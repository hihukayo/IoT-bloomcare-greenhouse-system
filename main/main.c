#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_debug.h"
#include "link_protocol.h"
#include "link_stm32.h"

/* Set to 1 to send one CONTROL command every 30 s. The node has to answer with
   an ACK, so this is how the SEQ + ACK handshake is checked by hand. */
#define LINK_TEST_CONTROL   0

#ifdef APP_DEBUG
static const char *TAG = "MAIN";        /* only used by the LOGx macros */
#endif

/**
 * @brief  Application entry point of the Bloomcare gateway.
 *
 *         For now the gateway only relays the STM32F103 sensor frames.
 *         OLED, WiFi and the phone application link will be added later.
 */
void app_main(void)
{
#if LINK_TEST_CONTROL
    TickType_t last_test = xTaskGetTickCount();
#endif
    LOGI("Bloomcare gateway booting ...");
    link_stm32_init();
    while (1)
    {
        link_stm32_poll();
#if LINK_TEST_CONTROL
        if ((xTaskGetTickCount() - last_test) >= pdMS_TO_TICKS(30000))
        {
            esp_err_t err;
            last_test = xTaskGetTickCount();
            err = link_stm32_send_control(LINK_ID_REPORT_MS, 5000);   /* real work */
            LOGI("control -> %s", esp_err_to_name(err));             /* debug only */
        }
#endif
    }
}
