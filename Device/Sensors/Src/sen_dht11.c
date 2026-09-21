/**
  ******************************************************************************
  * @file    sen_dht11.c
  * @brief   Implementation of the DHT11 one wire driver.
  * @note    The protocol is bit banged: the pin is switched open drain and the pulse
  *          widths are measured with BSP_Delay_Us().
  ******************************************************************************
  */
#include "sen_dht11.h"
#include "bsp_delay.h"

/* Pin operation macros */
#define SEN_DHT11_OUT_H()   HAL_GPIO_WritePin(SEN_DHT11_GPIO_PORT, SEN_DHT11_GPIO_PIN, GPIO_PIN_SET)
#define SEN_DHT11_OUT_L()   HAL_GPIO_WritePin(SEN_DHT11_GPIO_PORT, SEN_DHT11_GPIO_PIN, GPIO_PIN_RESET)
#define SEN_DHT11_IN()      HAL_GPIO_ReadPin(SEN_DHT11_GPIO_PORT, SEN_DHT11_GPIO_PIN)

/**
 * @brief  Wait until the data pin reaches the expected level.
 * @param  level: expected level (0 or 1).
 * @param  timeout_us: timeout in microseconds.
 * @retval 1 on success, 0 on timeout.
 */
static uint8_t Sen_Dht11_WaitLevel(uint8_t level, uint16_t timeout_us)
{
    while (SEN_DHT11_IN() != level)
    {
        if (timeout_us-- == 0)
        {
            return 0;
        }
        BSP_Delay_Us(1);
    }
    return 1;
}

/**
 * @brief  Send the start signal and wait for DHT11 response.
 * @retval 0 on success, 1 on failure.
 */
static uint8_t Sen_Dht11_Start(void)
{
    /* Step 1: MCU pulls the bus low for at least 18ms */
    SEN_DHT11_OUT_L();
    HAL_Delay(20);          /* 20ms > 18ms, safe */

    /* Step 2: MCU releases the bus (pull high) for 20~40us */
    SEN_DHT11_OUT_H();
    BSP_Delay_Us(30);

    /* Step 3: Wait for DHT11 to pull the bus low (response signal, ~80us) */
    if (!Sen_Dht11_WaitLevel(0, 100)) return 1;

    /* Step 4: Wait for DHT11 to pull the bus high (~80us) */
    if (!Sen_Dht11_WaitLevel(1, 100)) return 1;

    /* Step 5: Wait for DHT11 to pull the bus low again (data starts) */
    if (!Sen_Dht11_WaitLevel(0, 100)) return 1;

    return 0;
}

/**
 * @brief  Configure the DHT11 data pin (open-drain + pull-up).
 * @retval 0 on success.
 */
uint8_t Sen_Dht11_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SEN_DHT11_GPIO_CLK_ENABLE();

    /* Idle level of the one-wire bus is high */
    SEN_DHT11_OUT_H();

    GPIO_InitStruct.Pin = SEN_DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SEN_DHT11_GPIO_PORT, &GPIO_InitStruct);

    return 0U;
}

/**
 * @brief  Read 40-bit data (5 bytes) from DHT11.
 * @param  temp: pointer to store temperature.
 * @param  humi: pointer to store humidity.
 * @retval 0 on success, 1 on failure.
 */
uint8_t Sen_Dht11_Read(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5] = {0};
    uint8_t i, j;

    if ((temp == NULL) || (humi == NULL))
    {
        return 1;
    }

    /* 1. Send start signal and wait for response */
    if (Sen_Dht11_Start() != 0)
    {
        return 1;
    }

    /* 2. Read 40 bits (5 bytes) */
    for (i = 0; i < 5; i++)
    {
        for (j = 0; j < 8; j++)
        {
            /* Each bit starts with a 50us low level.
             * Wait for the rising edge (start of high level). */
            if (!Sen_Dht11_WaitLevel(1, 100)) return 1;

            /* High level duration decides the bit value:
             *   ~26-28us -> bit 0
             *   ~70us    -> bit 1
             * Delay 40us then sample: if still high, it's bit 1. */
            BSP_Delay_Us(40);

            if (SEN_DHT11_IN() == 1)
            {
                buf[i] |= (1 << (7 - j));

                /* Wait until the high level ends */
                if (!Sen_Dht11_WaitLevel(0, 100)) return 1;
            }
        }
    }

    /* 3. Checksum: sum of first 4 bytes == 5th byte */
    if ((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]) != buf[4])
    {
        return 1;
    }

    /* 4. Extract integer parts */
    *humi = buf[0];   /* humidity integer */
    *temp = buf[2];   /* temperature integer */

    return 0;
}

/**
 * @brief  Device layer entry: read the chip and hand out DEV_CH_TEMP / DEV_CH_HUMI.
 * @param  out:        buffer that receives the values.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when the read failed.
 */
uint8_t Sen_Dht11_Sample(Dev_Value_t *out, uint8_t max_values)
{
    uint8_t temp = 0U;
    uint8_t humi = 0U;

    if ((out == NULL) || (max_values < 2U))
    {
        return 0U;
    }
    if (Sen_Dht11_Read(&temp, &humi) != 0U)
    {
        return 0U;
    }
    out[0].ch = DEV_CH_TEMP;
    out[0].value = (int32_t)temp;
    out[1].ch = DEV_CH_HUMI;
    out[1].value = (int32_t)humi;
    return 2U;
}

/* The instance that Dev_Manager registers in its sensor array. */
const Dev_Sensor_t g_sen_dht11 =
{
    "DHT11",
    Sen_Dht11_Init,
    Sen_Dht11_Sample
};
