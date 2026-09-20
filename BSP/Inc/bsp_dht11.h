#ifndef __BSP_DHT11_H
#define __BSP_DHT11_H

#include "main.h"

/* DHT11 data pin definition - change according to your wiring */
#define DHT11_GPIO_PORT     DHT11_GPIO_Port
#define DHT11_GPIO_PIN      DHT11_Pin

/* GPIO clock of the port above (change it together with the pin) */
#define DHT11_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()

/**
 * @brief  Configure the DHT11 data pin: open-drain output + pull-up.
 * @note   Called from main() USER CODE section. It is applied at runtime
 *         on purpose, so it survives a CubeMX code regeneration (CubeMX
 *         generates the pin as push-pull, which DHT11 cannot drive).
 */
void DHT11_Init(void);

/**
 * @brief  Read temperature and humidity from DHT11.
 * @param  temp: pointer to store temperature (integer, Celsius).
 * @param  humi: pointer to store humidity (integer, %RH).
 * @retval 0 on success, 1 on failure (timeout or checksum error).
 */
uint8_t DHT11_Read(uint8_t *temp, uint8_t *humi);

#endif /* __BSP_DHT11_H */
