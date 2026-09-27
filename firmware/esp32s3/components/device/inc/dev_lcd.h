/**
  ******************************************************************************
  * @file    dev_lcd.h
  * @brief   Device layer: the 240x320 ST7789 panel on the four wire SPI bus.
  * @note    One module, one job: bring the panel up and keep it alive. What is
  *          drawn on it is decided one layer up, by the display task.
  * @note    Pin map of the module header, left to right on the module itself:
  *
  *            GND  GND
  *            3.3V 3V3
  *            SCL  DEV_LCD_SCLK_GPIO
  *            SDA  DEV_LCD_MOSI_GPIO
  *            RES  DEV_LCD_RST_GPIO
  *            DC   DEV_LCD_DC_GPIO
  *            CS   DEV_LCD_CS_GPIO
  *            BLK  DEV_LCD_BL_GPIO
  *
  *          All six signals sit on the left header of the board, inside the one
  *          run of free pins GPIO9 .. GPIO14, so the panel and the link UART
  *          above it share that column and the whole right column stays free
  *          for the knob. SCLK and MOSI are the IOMUX pins of FSPI, which is
  *          what allows the highest pixel clock on this chip. The other four
  *          are plain GPIO, any free pin would do, they only had to land in the
  *          same run to keep the module on one column.
  * @note    Depends on CONFIG_LV_* being set: see sdkconfig.defaults, the
  *          display is useless without the graphics library on top of it.
  ******************************************************************************
  */
#ifndef __DEV_LCD_H
#define __DEV_LCD_H

#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "hal/spi_types.h"

/** Panel geometry in pixels, portrait. */
#define DEV_LCD_H_RES           240U
#define DEV_LCD_V_RES           320U

/** Pins of the module header, see the note above. */
#define DEV_LCD_SCLK_GPIO       12
#define DEV_LCD_MOSI_GPIO       11
#define DEV_LCD_RST_GPIO        10
#define DEV_LCD_DC_GPIO         9
#define DEV_LCD_CS_GPIO         13
#define DEV_LCD_BL_GPIO         14

/** SPI bus of the panel, FSPI is the one whose IOMUX pins are used above. */
#define DEV_LCD_SPI_HOST        SPI2_HOST

/** Pixel clock. 40MHz moves one full 240x320 frame in about 31ms. Most of these
    modules accept 80MHz, raise it once the wiring is short and proven. */
#define DEV_LCD_PCLK_HZ         40000000

/** Lines of the panel one DMA transfer covers. */
#define DEV_LCD_BAND_LINES      32U

/** Backlight PWM resolution in bits, 10 gives 1024 steps. */
#define DEV_LCD_BL_DUTY_BITS    10U

/** Brightness dev_lcd_init() leaves behind, in percent. */
#define DEV_LCD_BL_START        80U


/**
 * @brief  Bring up the bus, the panel and the backlight.
 * @note   The backlight stays dark until the panel answered its own bring up,
 *         so the first frame is never shown half drawn.
 * @retval ESP_OK on success, the code of the failing step otherwise.
 */
esp_err_t dev_lcd_init(void);

/**
 * @brief  Handle of the panel IO, the LVGL display needs it.
 */
esp_lcd_panel_io_handle_t dev_lcd_io(void);

/**
 * @brief  Handle of the panel itself, the LVGL display needs it.
 */
esp_lcd_panel_handle_t dev_lcd_panel(void);

/**
 * @brief  Set the backlight brightness.
 * @param  percent: 0 is dark, 100 is full, more than 100 is clamped.
 * @retval ESP_OK on success.
 */
esp_err_t dev_lcd_backlight(uint8_t percent);

#endif /* __DEV_LCD_H */