/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @brief   Implementation of the board GPIO helpers.
  ******************************************************************************
  */
#include "bsp_gpio.h"

/**
 * @brief  Switch on the clock of one GPIO port.
 * @param  port: GPIOA .. GPIOG, taken from the label of the pin.
 */
void BSP_GPIO_ClkEnable(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    else
    {
        /* unknown port, nothing to switch on */
    }
}
