/**
  ******************************************************************************
  * @file    sen_dht11.h
  * @brief   Device layer: DHT11 temperature and humidity sensor driver.
  * @note    Knows only its own chip: it does not build frames, does not print logs and
  *          does not know the link protocol.
  ******************************************************************************
  */
#ifndef __SEN_DHT11_H
#define __SEN_DHT11_H

#include "main.h"
#include "bsp_gpio.h"
#include "dev_manager.h"

/* DHT11 data pin definition - change according to your wiring.
   DHT11_GPIO_Port / DHT11_Pin are the CubeMX labels of that pin. */
#define SEN_DHT11_GPIO_PORT     DHT11_GPIO_Port
#define SEN_DHT11_GPIO_PIN      DHT11_Pin

/* Clock of the port above, taken from the same label, so moving the sensor to
   another port means editing the pin in CubeMX only. */
#define SEN_DHT11_GPIO_CLK_ENABLE()     BSP_GPIO_ClkEnable(SEN_DHT11_GPIO_PORT)


/**
 * @brief  Configure the DHT11 data pin: open-drain output + pull-up.
 * @note   Called from Dev_Manager_Init(). It is applied at runtime on purpose,
 *         so it survives a CubeMX code regeneration (CubeMX generates the pin
 *         as push-pull, which DHT11 cannot drive).
 * @retval 0 on success.
 */
uint8_t Sen_Dht11_Init(void);

/**
 * @brief  Read temperature and humidity from DHT11.
 * @param  temp: pointer to store temperature (integer, Celsius).
 * @param  humi: pointer to store humidity (integer, %RH).
 * @retval 0 on success, 1 on failure (timeout or checksum error).
 */
uint8_t Sen_Dht11_Read(uint8_t *temp, uint8_t *humi);

/**
 * @brief  Device layer entry: read the chip and hand out DEV_CH_TEMP / DEV_CH_HUMI.
 * @param  out:        buffer that receives the values.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when the read failed.
 */
uint8_t Sen_Dht11_Sample(Dev_Value_t *out, uint8_t max_values);

/* The instance that Dev_Manager registers in its sensor array. */
extern const Dev_Sensor_t g_sen_dht11;

#endif /* __SEN_DHT11_H */
