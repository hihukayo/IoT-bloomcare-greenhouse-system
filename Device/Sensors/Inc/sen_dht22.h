/**
  ******************************************************************************
  * @file    sen_dht22.h
  * @brief   Device layer: DHT22 (AM2302) temperature and humidity sensor driver.
  * @note    Knows only its own chip: it does not build frames, does not print logs and
  *          does not know the link protocol.
  ******************************************************************************
  */
#ifndef __SEN_DHT22_H
#define __SEN_DHT22_H

#include "main.h"
#include "bsp_gpio.h"
#include "dev_manager.h"

/* DHT22 data pin definition - change according to your wiring.
   DHT22_GPIO_Port / DHT22_Pin are the CubeMX labels of that pin. */
#define SEN_DHT22_GPIO_PORT     DHT22_GPIO_Port
#define SEN_DHT22_GPIO_PIN      DHT22_Pin

/* Clock of the port above, taken from the same label, so moving the sensor to
   another port means editing the pin in CubeMX only. */
#define SEN_DHT22_GPIO_CLK_ENABLE()     BSP_GPIO_ClkEnable(SEN_DHT22_GPIO_PORT)

/* The one wire bus needs a pull-up. Most DHT22 breakout boards carry their own
   resistor, a bare sensor wants 4.7k .. 10k between DATA and VCC. */

/**
 * @brief  Configure the DHT22 data pin: open-drain output + pull-up.
 * @note   Called from Dev_Manager_Init(). It is applied at runtime on purpose,
 *         so it survives a CubeMX code regeneration (CubeMX generates the pin
 *          as push-pull, which a one wire bus cannot drive).
 * @retval 0 on success.
 */
uint8_t Sen_Dht22_Init(void);

/**
 * @brief  Read temperature and humidity from the DHT22.
 * @note   The chip resolves 0.1, so the values are handed out scaled by 100 the
 *         same way the link protocol carries them: 2340 means 23.40 C and a
 *         negative temperature keeps its sign. Scaling in the driver keeps the
 *         Tasks layer free of a per chip factor.
 * @param  temp_x100: destination of the temperature, unit C x 100.
 * @param  humi_x100: destination of the humidity, unit %RH x 100.
 * @retval 0 on success, 1 on failure (timeout or checksum error).
 */
uint8_t Sen_Dht22_Read(int32_t *temp_x100, int32_t *humi_x100);

/**
 * @brief  Device layer entry: read the chip and hand out DEV_CH_TEMP / DEV_CH_HUMI.
 * @param  out:        buffer that receives the values.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when the read failed.
 */
uint8_t Sen_Dht22_Sample(Dev_Value_t *out, uint8_t max_values);

/* The instance that Dev_Manager registers in its sensor array. */
extern const Dev_Sensor_t g_sen_dht22;

#endif /* __SEN_DHT22_H */
