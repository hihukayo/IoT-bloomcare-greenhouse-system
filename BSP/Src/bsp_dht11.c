#include "bsp_dht11.h"
#include "bsp_delay.h"

/* Pin operation macros */
#define DHT11_OUT_H()   HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET)
#define DHT11_OUT_L()   HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET)
#define DHT11_IN()      HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN)

/**
 * @brief  Configure the DHT11 data pin (open-drain + pull-up).
 */
void DHT11_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    DHT11_GPIO_CLK_ENABLE();

    /* Idle level of the one-wire bus is high */
    DHT11_OUT_H();

    GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  Wait until the data pin reaches the expected level.
 * @param  level: expected level (0 or 1).
 * @param  timeout_us: timeout in microseconds.
 * @retval 1 on success, 0 on timeout.
 */
static uint8_t DHT11_WaitLevel(uint8_t level, uint16_t timeout_us)
{
    while (DHT11_IN() != level)
    {
        if (timeout_us-- == 0)
        {
            return 0;
        }
        bsp_delay_us(1);
    }
    return 1;
}

/**
 * @brief  Send the start signal and wait for DHT11 response.
 * @retval 0 on success, 1 on failure.
 */
static uint8_t DHT11_Start(void)
{
    /* Step 1: MCU pulls the bus low for at least 18ms */
    DHT11_OUT_L();
    HAL_Delay(20);          /* 20ms > 18ms, safe */

    /* Step 2: MCU releases the bus (pull high) for 20~40us */
    DHT11_OUT_H();
    bsp_delay_us(30);

    /* Step 3: Wait for DHT11 to pull the bus low (response signal, ~80us) */
    if (!DHT11_WaitLevel(0, 100)) return 1;

    /* Step 4: Wait for DHT11 to pull the bus high (~80us) */
    if (!DHT11_WaitLevel(1, 100)) return 1;

    /* Step 5: Wait for DHT11 to pull the bus low again (data starts) */
    if (!DHT11_WaitLevel(0, 100)) return 1;

    return 0;
}

/**
 * @brief  Read 40-bit data (5 bytes) from DHT11.
 * @param  temp: pointer to store temperature.
 * @param  humi: pointer to store humidity.
 * @retval 0 on success, 1 on failure.
 */
uint8_t DHT11_Read(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5] = {0};
    uint8_t i, j;

    /* 1. Send start signal and wait for response */
    if (DHT11_Start() != 0)
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
            if (!DHT11_WaitLevel(1, 100)) return 1;

            /* High level duration decides the bit value:
             *   ~26-28us -> bit 0
             *   ~70us    -> bit 1
             * Delay 40us then sample: if still high, it's bit 1. */
            bsp_delay_us(40);

            if (DHT11_IN() == 1)
            {
                buf[i] |= (1 << (7 - j));

                /* Wait until the high level ends */
                if (!DHT11_WaitLevel(0, 100)) return 1;
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
