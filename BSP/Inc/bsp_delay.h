/**
  ******************************************************************************
  * @file    bsp_delay.h
  * @brief   Board layer: microsecond and millisecond blocking delays.
  * @note    Both rely on TIM2 running at 1MHz, so one counter tick is exactly 1us.
  *          See BSP_Delay_Init() for the timer settings CubeMX has to provide.
  ******************************************************************************
  */
#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H

#include "main.h"

/**
 * @brief  Initialize the microsecond delay based on TIM2.
 * @note   TIM2 must be configured in CubeMX as:
 *         Prescaler = 72 - 1  (72MHz / 72 = 1MHz, 1 tick = 1us)
 *         Counter Period (ARR) = 0xFFFF
 */
void BSP_Delay_Init(void);

/**
 * @brief  Blocking delay in microseconds, based on TIM2 counter.
 * @param  us: number of microseconds to wait (recommend < 60000).
 */
void BSP_Delay_Us(uint32_t us);

/**
 * @brief  Blocking delay in milliseconds, implemented by looping BSP_Delay_Us.
 * @param  ms: number of milliseconds to wait.
 */
void BSP_Delay_Ms(uint32_t ms);

#endif /* __BSP_DELAY_H */
