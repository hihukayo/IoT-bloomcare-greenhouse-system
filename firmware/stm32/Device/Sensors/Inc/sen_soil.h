/**
  ******************************************************************************
  * @file    sen_soil.h
  * @brief   Device layer: the soil moisture probe on PA1.
  * @note    Knows only its own pin: it does not build frames and does not know the
  *          link protocol. It does print, but only behind SEN_SOIL_TRACE, which is
  *          a calibration aid and is meant to go back to 0.
  * @note    A probe of this kind is not an instrument. It measures how well the
  *          soil conducts, which the salts in it influence as much as the water,
  *          and its two ends sit nowhere near the ends of the converter's range.
  *          What comes out is a position between two measured points, so by
  *          construction it reads 0 % in the air and 100 % in water and means
  *          "wetter than the air was, drier than the water was" in between.
  ******************************************************************************
  */
#ifndef __SEN_SOIL_H
#define __SEN_SOIL_H

#include "main.h"
#include "dev_manager.h"

/* ------------------------------------------------------------------
 * Device layer: the soil probe on PA1, which is ADC1_IN1 on this board.
 *
 * This one hangs on ADC1 and the light sensor on ADC3, and that is on
 * purpose. An ADC holds a single regular sequence, so two drivers sharing
 * one peripheral would each overwrite the other's channel and a reading
 * would quietly come from the wrong pin. One peripheral each keeps them
 * apart and keeps every driver knowing only its own chip.
 * ------------------------------------------------------------------ */

#define SEN_SOIL_ADC            hadc1
#define SEN_SOIL_CHANNEL        ADC_CHANNEL_1       /* PA1 is ADC1_IN1 */

/* The two ends of the scale, as raw counts straight out of the probe.
 *
 * These are not engineering units and no two modules share them: a probe spans
 * only part of the converter's range, and where that part sits depends on the
 * board, on the soil and on how far the probe is pushed in. So the driver
 * interpolates between two points it was told, instead of scaling from zero.
 *
 * To find yours: leave SEN_SOIL_TRACE at 1, run the board, hold the probe in the
 * air and note the count the console prints, then stand it in a glass of water
 * and note that one. Put the two here and set SEN_SOIL_TRACE back to 0.
 *
 * Which of the two is the larger does not matter. A dry probe usually reads high
 * and a wet one low, but not on every board, and the arithmetic below takes the
 * pair as it finds them. */
#define SEN_SOIL_RAW_DRY        3700
#define SEN_SOIL_RAW_WET        1500

/* Set to 1 while measuring the two points above: every read then puts its raw
   count on the console. The device layer is supposed to keep quiet, so this is a
   calibration aid and nothing else. Turn it back to 0 once DRY and WET hold real
   numbers. */
#define SEN_SOIL_TRACE          1

/* Conversions averaged into one reading. A probe in soil is a noisy thing to
   measure, and eight of them cost about a fifth of a millisecond. */
#define SEN_SOIL_SAMPLES        8U

/* Longest sampling window of the ADC. The module drives the pin through its own
   output stage, but the long window costs nothing and removes the question. */
#define SEN_SOIL_SAMPLE_TIME    ADC_SAMPLETIME_239CYCLES_5

/* How long one conversion may take. At 12 MHz it needs about 21us, so this is
   only ever reached when the ADC stops answering altogether. */
#define SEN_SOIL_TIMEOUT_MS     2U

/**
 * @brief  Bring up the ADC the soil probe is read through.
 * @note   The sampling window and the calibration are applied here at runtime
 *         rather than left to the generated file, the same way the DHT22 driver
 *         fixes up its own pin: a regeneration that resets them then costs
 *         nothing but a rebuild.
 * @retval 0 on success, 1 on failure.
 */
uint8_t Sen_Soil_Init(void);

/**
 * @brief  Read the probe once.
 * @param  moisture_x100: destination of the moisture as hundredths of a percent,
 *                        0 .. 10000, that is 0.00 % .. 100.00 %.
 * @retval 0 on success, 1 on failure.
 */
uint8_t Sen_Soil_Read(int32_t *moisture_x100);

/**
 * @brief  Device layer entry: hand out DEV_CH_SOIL.
 * @param  out:        buffer that receives the value.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when the read failed.
 */
uint8_t Sen_Soil_Sample(Dev_Value_t *out, uint8_t max_values);

/* The instance that Dev_Manager registers in its sensor array. */
extern const Dev_Sensor_t g_sen_soil;

#endif /* __SEN_SOIL_H */
