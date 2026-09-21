/**
  ******************************************************************************
  * @file    sen_dht22.c
  * @brief   Implementation of the DHT22 one wire driver.
  * @note    The protocol is bit banged: the pin is switched open drain and the pulse
  *          widths are measured with BSP_Delay_Us().
  ******************************************************************************
  */
#include "sen_dht22.h"
#include "bsp_delay.h"

/* Timing of the datasheet. The step constants are counted in BSP_Delay_Us(1)
   turns, so they are compared against each other and never against real time. */
#define SEN_DHT22_START_HOLD_MS     2U      /* host keeps the bus low, >= 1 ms  */
#define SEN_DHT22_START_RELEASE_US  30U     /* then releases it for 20 .. 40 us */
#define SEN_DHT22_WAIT_STEPS        200U    /* give up on one edge after that   */
#define SEN_DHT22_HIGH_MAX_STEPS    150U    /* bound of one measured high level */
#define SEN_DHT22_ONE_MIN_STEPS     45U     /* a 0 holds ~28 us, a 1 holds ~70  */

/* Pin operation macros */
#define SEN_DHT22_OUT_H()   HAL_GPIO_WritePin(SEN_DHT22_GPIO_PORT, SEN_DHT22_GPIO_PIN, GPIO_PIN_SET)
#define SEN_DHT22_OUT_L()   HAL_GPIO_WritePin(SEN_DHT22_GPIO_PORT, SEN_DHT22_GPIO_PIN, GPIO_PIN_RESET)
#define SEN_DHT22_IN()      HAL_GPIO_ReadPin(SEN_DHT22_GPIO_PORT, SEN_DHT22_GPIO_PIN)

/**
 * @brief  Wait until the data pin reaches the expected level.
 * @param  level: expected level (0 or 1).
 * @param  steps: give up after this many delay steps.
 * @retval 1 on success, 0 on timeout.
 */
static uint8_t Sen_Dht22_WaitLevel(uint8_t level, uint16_t steps)
{
    while (SEN_DHT22_IN() != level)
    {
        if (steps-- == 0U)
        {
            return 0U;
        }
        BSP_Delay_Us(1);
    }
    return 1U;
}

/**
 * @brief  Measure how long the data pin stays high.
 * @note   The bit value is the width of its high level, so measuring it is more
 *         robust than sampling after a fixed delay: the line is left low again,
 *         which is where the next bit starts from.
 * @param  steps: give up after this many delay steps.
 * @retval number of delay steps the pin stayed high.
 */
static uint16_t Sen_Dht22_MeasureHigh(uint16_t steps)
{
    uint16_t used = 0U;

    while (SEN_DHT22_IN() == 1)
    {
        if (used >= steps)
        {
            return steps;
        }
        BSP_Delay_Us(1);
        used++;
    }
    return used;
}

/**
 * @brief  Send the start signal and wait for the response of the chip.
 * @retval 0 on success, 1 on failure.
 */
static uint8_t Sen_Dht22_Start(void)
{
    /* Step 1: the host pulls the bus low, the datasheet asks for at least 1 ms */
    SEN_DHT22_OUT_L();
    HAL_Delay(SEN_DHT22_START_HOLD_MS);

    /* Step 2: the host releases the bus for 20 .. 40 us */
    SEN_DHT22_OUT_H();
    BSP_Delay_Us(SEN_DHT22_START_RELEASE_US);

    /* Step 3: the chip answers with 80 us low ... */
    if (!Sen_Dht22_WaitLevel(0, SEN_DHT22_WAIT_STEPS)) return 1U;

    /* Step 4: ... then 80 us high, after which the first bit starts */
    if (!Sen_Dht22_WaitLevel(1, SEN_DHT22_WAIT_STEPS)) return 1U;

    return 0U;
}

/**
 * @brief  Configure the DHT22 data pin (open-drain + pull-up).
 * @retval 0 on success.
 */
uint8_t Sen_Dht22_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SEN_DHT22_GPIO_CLK_ENABLE();

    /* Idle level of the one-wire bus is high */
    SEN_DHT22_OUT_H();

    GPIO_InitStruct.Pin = SEN_DHT22_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SEN_DHT22_GPIO_PORT, &GPIO_InitStruct);

    return 0U;
}

/**
 * @brief  Read the 40 bit answer of the chip.
 * @note   Bit layout, most significant bit first:
 *             buf[0] buf[1] : humidity, %RH x 10
 *             buf[2] buf[3] : temperature, C x 10, bit 15 of buf[2] is the sign
 *             buf[4]        : checksum, low byte of the sum of the four above
 * @param  temp_x100: destination of the temperature, unit C x 100.
 * @param  humi_x100: destination of the humidity, unit %RH x 100.
 * @retval 0 on success, 1 on failure.
 */
uint8_t Sen_Dht22_Read(int32_t *temp_x100, int32_t *humi_x100)
{
    uint8_t buf[5] = {0};
    uint8_t i;
    uint8_t j;
    int32_t raw;

    if ((temp_x100 == NULL) || (humi_x100 == NULL))
    {
        return 1U;
    }

    /* 1. Send the start signal and wait for the response */
    if (Sen_Dht22_Start() != 0U)
    {
        return 1U;
    }

    /* 2. Read 40 bits: every bit is a 50 us low level followed by a high level */
    for (i = 0U; i < 5U; i++)
    {
        for (j = 0U; j < 8U; j++)
        {
            if (!Sen_Dht22_WaitLevel(0, SEN_DHT22_WAIT_STEPS)) return 1U;
            if (!Sen_Dht22_WaitLevel(1, SEN_DHT22_WAIT_STEPS)) return 1U;
            if (Sen_Dht22_MeasureHigh(SEN_DHT22_HIGH_MAX_STEPS) >= SEN_DHT22_ONE_MIN_STEPS)
            {
                buf[i] |= (uint8_t)(1U << (7U - j));
            }
        }
    }

    /* 3. Checksum: the low byte of the sum of the first four bytes */
    if ((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]) != buf[4])
    {
        return 1U;
    }

    /* 4. Scale the tenths of the chip up to the hundredths of the wire format */
    raw = ((int32_t)buf[0] << 8) | (int32_t)buf[1];
    *humi_x100 = raw * 10;

    raw = ((int32_t)(buf[2] & 0x7FU) << 8) | (int32_t)buf[3];
    *temp_x100 = raw * 10;
    if ((buf[2] & 0x80U) != 0U)
    {
        *temp_x100 = -*temp_x100;
    }

    return 0U;
}

/**
 * @brief  Device layer entry: read the chip and hand out DEV_CH_TEMP / DEV_CH_HUMI.
 * @param  out:        buffer that receives the values.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when the read failed.
 */
uint8_t Sen_Dht22_Sample(Dev_Value_t *out, uint8_t max_values)
{
    int32_t temp_x100 = 0;
    int32_t humi_x100 = 0;

    if ((out == NULL) || (max_values < 2U))
    {
        return 0U;
    }
    if (Sen_Dht22_Read(&temp_x100, &humi_x100) != 0U)
    {
        return 0U;
    }
    out[0].ch = DEV_CH_TEMP;
    out[0].value = temp_x100;
    out[1].ch = DEV_CH_HUMI;
    out[1].value = humi_x100;
    return 2U;
}

/* The instance that Dev_Manager registers in its sensor array. */
const Dev_Sensor_t g_sen_dht22 =
{
    "DHT22",
    Sen_Dht22_Init,
    Sen_Dht22_Sample
};
