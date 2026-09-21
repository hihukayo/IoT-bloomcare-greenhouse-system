/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   Board layer: USART1 debug console and USART2 gateway link.
  * @note    Only moves bytes in and out of the hardware. What those bytes mean is
  *          decided one layer up, in Components/comp_link.h.
  ******************************************************************************
  */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include <stdio.h>

/* ------------------------------------------------------------------
 * Board support: the two UARTs wired on this board.
 *
 *   USART1 : debug console, printf is redirected here (115200 8N1)
 *   USART2 : link to the ESP32-S3 gateway, 115200 8N1, RX on DMA
 *
 * This file only moves bytes in and out of the hardware. What the bytes
 * mean is decided one layer up, in Components/ (comp_link).
 * ------------------------------------------------------------------ */

/* Comment out the next line to drop every debug counter of this layer. */
#define BSP_UART_DEBUG

#define BSP_UART2_RX_BUF_SIZE    256U   /* DMA circular ring size in bytes  */
#define BSP_UART2_TX_TIMEOUT_MS  100U   /* timeout of one blocking frame TX */

/**
 * @brief  Bring up the debug console.
 * @note   The console itself is configured by CubeMX in MX_USART1_UART_Init(),
 *         this hook only exists so the board layer has a single entry point.
 */
void BSP_Uart_Init(void);

/**
 * @brief  Arm the USART2 RX DMA ring and its IDLE interrupt.
 * @note   CubeMX already configured DMA1_Channel6 as byte/circular/incrementing,
 *         so only the pointers and the enable bits are programmed here.
 *         HAL is deliberately kept out of the receive path: HAL_UART_Receive_DMA()
 *         aborts reception on the first overrun or framing error, which would
 *         silently kill the link until the next reset.
 */
void BSP_Uart2_Init(void);

/**
 * @brief  Copy the bytes that arrived since the last call out of the ring.
 * @param  dst: destination buffer.
 * @param  max: capacity of the destination buffer.
 * @retval number of bytes copied, 0 when nothing is pending.
 */
uint16_t BSP_Uart2_RxTake(uint8_t *dst, uint16_t max);

/**
 * @brief  Tell whether the line went idle since the last call.
 * @retval 1 when a frame ending idle event is pending, 0 otherwise.
 */
uint8_t BSP_Uart2_IdleTake(void);

/**
 * @brief  Blocking transmit on USART2.
 * @param  data: bytes to send.
 * @param  len:  number of bytes to send.
 * @retval 0 on success, 1 on failure.
 */
uint8_t BSP_Uart2_Tx(const uint8_t *data, uint16_t len);

/**
 * @brief  USART2 interrupt hook, called first inside USART2_IRQHandler().
 * @note   It only records that a frame ended on the wire. The frame itself is
 *         parsed and answered later in the task layer, so the ISR stays short
 *         and never performs a blocking transmit.
 */
void BSP_Uart2_IrqHandler(void);

#ifdef BSP_UART_DEBUG
/* Plain globals instead of a struct: they only serve debugging on USART1. */
extern volatile uint32_t g_bsp_rx_bytes;    /* bytes taken from the DMA ring */
extern volatile uint32_t g_bsp_rx_drop;     /* bytes lost, poll came too late */
extern volatile uint32_t g_bsp_idle_evt;    /* IDLE interrupts served        */
extern volatile uint32_t g_bsp_uart_err;    /* ORE / NE / FE / PE on USART2  */
#endif /* BSP_UART_DEBUG */

#endif /* __BSP_UART_H */
