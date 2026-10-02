/**
  ******************************************************************************
  * @file    sen_light.h
  * @brief   Device layer: the light sensor the board carries on PF8.
  * @note    Knows only its own pin: it does not build frames, does not print logs
  *          and does not know the link protocol.
  * @note    What it hands out is a share of the reference and not a unit of
  *          light. LS1 sits in a divider between 3V3 and ground with a 1k
  *          resistor in series to the pin, so the reading follows the voltage
  *          across the sensor. That is enough to tell dawn from noon, or to
  *          decide that a lamp is wanted; it is not a figure in lux. A
  *          photodiode is neither linear nor steady over temperature, and a real
  *          light meter wants a digital part such as a BH1750 behind it.
  ******************************************************************************
  */
#ifndef __SEN_LIGHT_H
#define __SEN_LIGHT_H

#include "main.h"
#include "dev_manager.h"

/* ------------------------------------------------------------------
 * Device layer: the light sensor on PF8, which is ADC3_IN6 on this board.
 *
 * The pin and the ADC were set up by CubeMX; the label of the pin there
 * is LIGHT_SENSOR and the handle it generated is hadc3. Nothing here is
 * wired by hand, so moving the sensor to another ADC channel means one
 * edit in CubeMX and one line below.
 * ------------------------------------------------------------------ */

#define SEN_LIGHT_ADC           hadc3
#define SEN_LIGHT_CHANNEL       ADC_CHANNEL_6       /* PF8 is ADC3_IN6 */

/* Which way the reading moves as the light grows.
   The divider pulls the pin down as the sensor conducts harder, so a brighter
   room reads as a smaller number and this stays at 1 to turn that back into a
   share that grows with the light. Cover the sensor with a hand and watch the
   log: if the number climbs instead of falling, the board is wired the other
   way round and this wants to be 0. */
#define SEN_LIGHT_INVERT        1

/* Conversions averaged into one reading, so the last digit stops jumping. Eight
   of them cost about a fifth of a millisecond. */
#define SEN_LIGHT_SAMPLES       8U

/* Longest sampling window of the ADC. The 1k in series and the 47k above it
   leave the source impedance near fifty kilo ohm, which a short window cannot
   charge to the right voltage. */
#define SEN_LIGHT_SAMPLE_TIME   ADC_SAMPLETIME_239CYCLES_5

/* How long one conversion may take. At 12 MHz it needs about 21us, so this is
   only ever reached when the ADC stops answering altogether. */
#define SEN_LIGHT_TIMEOUT_MS    2U

/* Full scale of the 12 bit conversion. */
#define SEN_LIGHT_FULL_SCALE    4095

/**
 * @brief  Bring up the ADC the light sensor is read through.
 * @note   The sampling window and the calibration are applied here at runtime
 *         rather than left to the generated file, the same way the DHT22 driver
 *         fixes up its own pin: a regeneration that resets them then costs
 *         nothing but a rebuild.
 * @retval 0 on success, 1 on failure.
 */
uint8_t Sen_Light_Init(void);

/**
 * @brief  Read the sensor once.
 * @param  level_x100: destination of the brightness as hundredths of a percent,
 *                     0 .. 10000, that is 0.00 % .. 100.00 % of the reference.
 * @retval 0 on success, 1 on failure.
 */
uint8_t Sen_Light_Read(int32_t *level_x100);

/**
 * @brief  Device layer entry: hand out DEV_CH_LIGHT.
 * @param  out:        buffer that receives the value.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when the read failed.
 */
uint8_t Sen_Light_Sample(Dev_Value_t *out, uint8_t max_values);

/* The instance that Dev_Manager registers in its sensor array. */
extern const Dev_Sensor_t g_sen_light;

#endif /* __SEN_LIGHT_H */
