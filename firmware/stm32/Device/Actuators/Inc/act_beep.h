/**
  ******************************************************************************
  * @file    act_beep.h
  * @brief   Device layer: buzzer on one GPIO pin, the local audible cue.
  * @note    Knows only its own pin: it does not build frames, does not print logs
  *          and does not know the link protocol.
  ******************************************************************************
  */
#ifndef __ACT_BEEP_H
#define __ACT_BEEP_H

#include "main.h"
#include "bsp_gpio.h"
#include "dev_manager.h"

/* Buzzer pin definition - change according to your wiring.
   BEEP_GPIO_Port / BEEP_Pin are the CubeMX labels of that pin. */
#define ACT_BEEP_GPIO_PORT      BEEP_GPIO_Port
#define ACT_BEEP_GPIO_PIN       BEEP_Pin

/* Clock of the port above, taken from the same label, so moving the buzzer to
   another port means editing the pin in CubeMX only. */
#define ACT_BEEP_GPIO_CLK_ENABLE()      BSP_GPIO_ClkEnable(ACT_BEEP_GPIO_PORT)

/* An active buzzer sounds while the pin is high, which is how CubeMX leaves it
   configured. Swap the two levels when the module on the board is wired the
   other way round. */
#define ACT_BEEP_ACTIVE_LEVEL   GPIO_PIN_SET
#define ACT_BEEP_IDLE_LEVEL     GPIO_PIN_RESET

/**
 * @brief  Configure the buzzer pin as push-pull output and leave it silent.
 * @note   Called from Dev_Manager_Init(). The pin is set up at runtime on purpose,
 *         so the driver also works when CubeMX is regenerated with different
 *         defaults for that pin.
 * @retval 0 on success.
 */
uint8_t Act_Beep_Init(void);

/**
 * @brief  Sound the buzzer.
 */
void Act_Beep_On(void);

/**
 * @brief  Silence the buzzer.
 */
void Act_Beep_Off(void);

/**
 * @brief  Device layer entry: 0 silences the buzzer, anything else sounds it.
 * @param  value: 0 = silent, otherwise sounding.
 * @retval 0 on success.
 */
uint8_t Act_Beep_Set(int32_t value);

/**
 * @brief  Sound the buzzer once and stay silent for a while afterwards.
 * @note   Blocking: the caller waits for the whole pulse. Keep the pulses short,
 *         the Tasks layer is polled and cannot run during that wait.
 * @param  on_ms:  length of the tone.
 * @param  off_ms: silence after it.
 */
void Act_Beep_Beep(uint16_t on_ms, uint16_t off_ms);

/**
 * @brief  Sound the buzzer several times in a row, e.g. two short beeps.
 * @param  times:  number of pulses. 0 keeps the buzzer silent.
 * @param  on_ms:  length of one tone.
 * @param  gap_ms: silence between two tones, and after the last one.
 */
void Act_Beep_Pattern(uint8_t times, uint16_t on_ms, uint16_t gap_ms);

/* The instance that Dev_Manager registers in its actuator array. */
extern const Dev_Actuator_t g_act_beep;

#endif /* __ACT_BEEP_H */
