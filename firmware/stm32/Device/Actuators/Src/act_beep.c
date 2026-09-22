/**
  ******************************************************************************
  * @file    act_beep.c
  * @brief   Implementation of the buzzer driver.
  * @note    The module is active: it sounds while the pin is high, so no timer and
  *          no PWM is needed. A passive buzzer without its own oscillator would
  *          need a tone frequency instead, which this driver does not do.
  ******************************************************************************
  */
#include "act_beep.h"
#include "bsp_delay.h"

/**
 * @brief  Configure the buzzer pin as push-pull output, silent.
 */
static void Act_Beep_ConfigurePin(void)
{
    GPIO_InitTypeDef gpio = {0};

    ACT_BEEP_GPIO_CLK_ENABLE();
    HAL_GPIO_WritePin(ACT_BEEP_GPIO_PORT, ACT_BEEP_GPIO_PIN, ACT_BEEP_IDLE_LEVEL);

    gpio.Pin = ACT_BEEP_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ACT_BEEP_GPIO_PORT, &gpio);
}

/**
 * @brief  Configure the buzzer pin and leave it silent.
 * @retval 0 on success.
 */
uint8_t Act_Beep_Init(void)
{
    Act_Beep_ConfigurePin();
    return 0U;
}

/**
 * @brief  Sound the buzzer.
 */
void Act_Beep_On(void)
{
    HAL_GPIO_WritePin(ACT_BEEP_GPIO_PORT, ACT_BEEP_GPIO_PIN, ACT_BEEP_ACTIVE_LEVEL);
}

/**
 * @brief  Silence the buzzer.
 */
void Act_Beep_Off(void)
{
    HAL_GPIO_WritePin(ACT_BEEP_GPIO_PORT, ACT_BEEP_GPIO_PIN, ACT_BEEP_IDLE_LEVEL);
}

/**
 * @brief  Device layer entry: 0 silences the buzzer, anything else sounds it.
 * @param  value: 0 = silent, otherwise sounding.
 * @retval 0 on success.
 */
uint8_t Act_Beep_Set(int32_t value)
{
    if (value == 0)
    {
        Act_Beep_Off();
    }
    else
    {
        Act_Beep_On();
    }
    return 0U;
}

/**
 * @brief  Sound the buzzer several times in a row.
 * @param  times:  number of pulses. 0 keeps the buzzer silent.
 * @param  on_ms:  length of one tone.
 * @param  gap_ms: silence between two tones, and after the last one.
 */
void Act_Beep_Pattern(uint8_t times, uint16_t on_ms, uint16_t gap_ms)
{
    uint8_t i;

    for (i = 0U; i < times; i++)
    {
        Act_Beep_On();
        BSP_Delay_Ms((uint32_t)on_ms);
        Act_Beep_Off();
        BSP_Delay_Ms((uint32_t)gap_ms);
    }
}

/**
 * @brief  Sound the buzzer once and stay silent for a while afterwards.
 * @param  on_ms:  length of the tone.
 * @param  off_ms: silence after it.
 */
void Act_Beep_Beep(uint16_t on_ms, uint16_t off_ms)
{
    Act_Beep_Pattern(1U, on_ms, off_ms);
}

/* The instance behind the g_act_beep pointer of the device table. */
const Dev_Actuator_t g_act_beep =
{
    "buzzer",
    DEV_CH_BEEP,
    Act_Beep_Init,
    Act_Beep_Set
};
