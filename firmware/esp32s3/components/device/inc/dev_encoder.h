/**
  ******************************************************************************
  * @file    dev_encoder.h
  * @brief   Device layer: the rotary knob with its push switch.
  * @note    Pin map of the five pin module header, ESP32-S3 side:
  *
  *            +   3V3                        GND  GND
  *            CLK DEV_ENC_A_GPIO             SW   DEV_ENC_SW_GPIO
  *            DT  DEV_ENC_B_GPIO
  *
  *          The three signals sit on the right header of the board, so the knob
  *          is wired on the opposite side of the panel and of the link UART.
  *          They are the MTDO / MTDI / MTMS pins, which costs an external JTAG
  *          probe; debugging stays available over the USB serial / JTAG port of
  *          the module. The right header of this board carries no 3V3, so the +
  *          line is the single wire that crosses over to the left column.
  * @note    The quadrature pair is decoded in hardware by one PCNT unit, so the
  *          CPU never sees an edge and a fast turn cannot lose a step. The push
  *          switch is a plain GPIO, debounced here.
  ******************************************************************************
  */
#ifndef __DEV_ENCODER_H
#define __DEV_ENCODER_H

#include <stdint.h>
#include "esp_err.h"

/** Pins of the module header, see the note above. */
#define DEV_ENC_A_GPIO          42
#define DEV_ENC_B_GPIO          41
#define DEV_ENC_SW_GPIO         40

/** One mechanical detent of the knob is one full quadrature cycle, which the
    hardware turns into four counts. Lower this if a detent is ever missed,
    raise it if one detent ever steps twice. */
#define DEV_ENC_COUNTS_PER_STEP 4

/** Window of the hardware counter. A turn between two polls never comes close
    to it, the number only has to sit far above the detents of one second. */
#define DEV_ENC_PCNT_LIMIT      1000

/** A contact that stays at one level for this long counts as settled. */
#define DEV_ENC_DEBOUNCE_MS     20U

/** Pressed for longer than this is a hold, not a click. */
#define DEV_ENC_LONG_PRESS_MS   800U

/** Events latched by one call of dev_encoder_poll(). */
#define DEV_ENC_EV_NONE         0x00U
#define DEV_ENC_EV_SHORT        0x01U
#define DEV_ENC_EV_LONG         0x02U

/**
 * @brief  Bring up the quadrature decoder and the push switch.
 * @retval ESP_OK on success, the code of the failing step otherwise.
 */
esp_err_t dev_encoder_init(void);

/**
 * @brief  Read the push switch once and report what happened since the last
 *         call.
 * @note   Call it often enough that a press shorter than DEV_ENC_DEBOUNCE_MS
 *         cannot slip through, the LVGL input callback is the natural place.
 * @retval DEV_ENC_EV_NONE, or one of DEV_ENC_EV_SHORT / DEV_ENC_EV_LONG.
 */
uint8_t dev_encoder_poll(void);

/**
 * @brief  Level of the push switch, already debounced.
 * @retval 1 while the knob is held down, 0 otherwise.
 */
uint8_t dev_encoder_button_down(void);

/**
 * @brief  Detents the knob turned since the last call, positive is clockwise.
 * @note   The remainder of a part turned detent is kept, so a slow turn is
 *         never swallowed. The counter is cleared by this call.
 * @retval number of detents, negative when turned the other way.
 */
int dev_encoder_take_steps(void);

#endif /* __DEV_ENCODER_H */