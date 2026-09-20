#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H

#include "main.h"

/**
 * @brief  Initialize the microsecond delay based on TIM2.
 * @note   TIM2 must be configured in CubeMX as:
 *         Prescaler = 72 - 1  (72MHz / 72 = 1MHz, 1 tick = 1us)
 *         Counter Period (ARR) = 0xFFFF
 */
void bsp_delay_init(void);

/**
 * @brief  Blocking delay in microseconds, based on TIM2 counter.
 * @param  us: number of microseconds to wait (recommend < 60000).
 */
void bsp_delay_us(uint32_t us);

/**
 * @brief  Blocking delay in milliseconds, implemented by looping bsp_delay_us.
 * @param  ms: number of milliseconds to wait.
 */
void bsp_delay_ms(uint32_t ms);

#endif /* __BSP_DELAY_H */
