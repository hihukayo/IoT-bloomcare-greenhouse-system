/**
  ******************************************************************************
  * @file    bsp_gpio.h
  * @brief   Board layer: helpers around the GPIO pins of this board.
  * @note    A device driver hands in the CubeMX label of its pin, so it never
  *          has to know or repeat which port that pin belongs to.
  ******************************************************************************
  */
#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "main.h"

/**
 * @brief  Switch on the clock of one GPIO port.
 * @note   Called with the CubeMX label of the pin, for example:
 *             BSP_GPIO_ClkEnable(SEN_DHT11_GPIO_PORT);
 *         Building the HAL macro by token pasting is deliberately not used:
 *         GPIOG and friends are macros themselves (a pointer constant), so
 *         they cannot be pasted into __HAL_RCC_GPIOx_CLK_ENABLE().
 * @param  port: GPIOA .. GPIOG, taken from the label of the pin.
 */
void BSP_GPIO_ClkEnable(GPIO_TypeDef *port);

#endif /* __BSP_GPIO_H */
