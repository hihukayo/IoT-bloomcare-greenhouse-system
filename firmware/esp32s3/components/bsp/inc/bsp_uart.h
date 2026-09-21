#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include "driver/uart.h"
 
/** UART used to talk to the STM32F103 (USART2). */
#define BSP_UART_PORT       UART_NUM_1
#define BSP_UART_TX_GPIO    17      /* U1TXD -> STM32 PA3 (USART2_RX) */
#define BSP_UART_RX_GPIO    18      /* U1RXD <- STM32 PA2 (USART2_TX) */
#define BSP_UART_BAUD       115200
#define BSP_UART_RX_BUF     1024

/**
 * @brief  Install the UART driver, configure pins and 115200 8N1 format.
 */
void bsp_uart_init(void);

/**
 * @brief  Read bytes from the link UART.
 * @param  buf:        destination buffer.
 * @param  len:        maximum number of bytes to read.
 * @param  timeout_ms: timeout in milliseconds.
 * @retval number of bytes read, or -1 on error.
 */
int bsp_uart_read(uint8_t *buf, uint32_t len, uint32_t timeout_ms);

/**
 * @brief  Write bytes to the link UART.
 * @param  buf: source buffer.
 * @param  len: number of bytes to write.
 * @retval number of bytes written, or -1 on error.
 */
int bsp_uart_write(const uint8_t *buf, uint32_t len);

#endif /* __BSP_UART_H */
