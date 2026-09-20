#include "bsp_uart.h"
#include "freertos/FreeRTOS.h"
#include "app_debug.h"

#ifdef APP_DEBUG
static const char *TAG = "BSP_UART";    /* only used by the LOGx macros */
#endif

/**
 * @brief  Install the UART driver, configure pins and 115200 8N1 format.
 */
void bsp_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = BSP_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BSP_UART_PORT, BSP_UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BSP_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(BSP_UART_PORT, BSP_UART_TX_GPIO, BSP_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    LOGI("UART%d ready: TX=GPIO%d RX=GPIO%d %d 8N1",
            (int)BSP_UART_PORT, BSP_UART_TX_GPIO, BSP_UART_RX_GPIO, BSP_UART_BAUD);
}

/**
 * @brief  Read bytes from the link UART.
 * @param  buf:        destination buffer.
 * @param  len:        maximum number of bytes to read.
 * @param  timeout_ms: timeout in milliseconds.
 * @retval number of bytes read, or -1 on error.
 */
int bsp_uart_read(uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
    return uart_read_bytes(BSP_UART_PORT, buf, len, pdMS_TO_TICKS(timeout_ms));
}

/**
 * @brief  Write bytes to the link UART.
 * @param  buf: source buffer.
 * @param  len: number of bytes to write.
 * @retval number of bytes written, or -1 on error.
 */
int bsp_uart_write(const uint8_t *buf, uint32_t len)
{
    return uart_write_bytes(BSP_UART_PORT, buf, len);
}
