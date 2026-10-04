/**
  ******************************************************************************
  * @file    sen_co2.h
  * @brief   Device layer: the JW01 CO2 module on USART3.
  * @note    Knows only its own sensor: it does not build frames and does not know
  *          the link protocol. It does print, but only behind SEN_CO2_TRACE, which
  *          is a bring up aid and is meant to go back to 0.
  * @note    This one is not like the others. The DHT22, the light sensor and the
  *          soil probe all answer when they are asked; this module talks on its
  *          own, six bytes every so often, and never needs a request. So the
  *          driver only ever listens.
  ******************************************************************************
  */
#ifndef __SEN_CO2_H
#define __SEN_CO2_H

#include "main.h"
#include "dev_manager.h"

/* ------------------------------------------------------------------
 * Device layer: the JW01-CO2-V2.2 module, on USART3 at 9600 8N1.
 *
 *   PB10  USART3_TX  ->  module A, which is the module's own RX
 *   PB11  USART3_RX  <-  module B, which is the module's own TX
 *
 * Those two pins read like an RS485 pair and are not one: the datasheet names
 * them A (RX) and B (TX) and the interface is plain UART, so no transceiver
 * belongs in between. It does ask for a divider on the B line when the host
 * runs below five volts, because the module drives B to five.
 *
 * The module is a tin oxide gas sensor, not an IR one, and the datasheet says
 * so itself: it answers to alcohol, smoke and cooking fumes as readily as to
 * CO2 and has no absolute calibration. What it is good for is telling stale air
 * from fresh, which is the question a greenhouse actually asks.
 * ------------------------------------------------------------------ */

#define SEN_CO2_UART            huart3

/* The frame is fixed at six bytes and the address in the first of them is
   always 0x2C:

       B1 address | B2 ppm high | B3 ppm low | B4 0x03 | B5 0xFF | B6 checksum

   The checksum is B1..B5 added up and taken modulo 256, and the reading is
   B2 * 256 + B3, already in ppm. B4 and B5 carry 0x03FF, the full scale of the
   converter behind the sensor. */
#define SEN_CO2_FRAME_LEN       6U
#define SEN_CO2_ADDR            0x2CU
#define SEN_CO2_FULL_SCALE_HI   0x03U
#define SEN_CO2_FULL_SCALE_LO   0xFFU

/* The ring the DMA fills. Only the newest reading is ever wanted, so it need
   only be long enough to hold a frame or two: anything older is dropped. */
#define SEN_CO2_RX_BUF_SIZE     256U

/* How long a reading stays good. The datasheet gives the response time as at
   most ten seconds, so a frame older than that is worth calling a fault rather
   than a value. */
#define SEN_CO2_STALE_MS        12000U

/* The datasheet asks for a warm up: sixty seconds before the readings mean
   anything, and five to ten minutes before they settle. Until that has passed
   the driver reports a fault, so a module that has only just been switched on
   cannot put a wild number on the screen and have it believed. */
#define SEN_CO2_WARMUP_MS       60000U

/* Set to 1 to put every frame that checks out on the console, with the ppm it
   carries. The device layer is supposed to keep quiet, so this is a bring up aid
   and nothing else. Turn it back to 0 once the wiring is known to be right. */
#define SEN_CO2_TRACE           1

/**
 * @brief  Arm the DMA ring the module is listened to through.
 * @note   The port itself is set up by CubeMX; what is left here is the ring,
 *         armed at runtime for the same reason the DHT22 driver fixes up its own
 *         pin: a regeneration that resets it then costs nothing but a rebuild.
 * @retval 0 on success, 1 on failure.
 */
uint8_t Sen_Co2_Init(void);

/**
 * @brief  Report the most recent reading.
 * @param  ppm: destination, parts per million.
 * @retval 0 on success, 1 when there is nothing current to report.
 */
uint8_t Sen_Co2_Read(int32_t *ppm);

/**
 * @brief  Device layer entry: hand out DEV_CH_CO2.
 * @param  out:        buffer that receives the value.
 * @param  max_values: capacity of that buffer.
 * @retval number of values written, 0 when the read failed.
 */
uint8_t Sen_Co2_Sample(Dev_Value_t *out, uint8_t max_values);

/* The instance that Dev_Manager registers in its sensor array. */
extern const Dev_Sensor_t g_sen_co2;

#endif /* __SEN_CO2_H */
