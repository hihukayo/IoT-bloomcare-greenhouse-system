/**
  ******************************************************************************
  * @file    dev_lcd.c
  * @brief   Implementation of the ST7789 panel device.
  * @note    The bytes are pushed by the esp_lcd SPI panel IO driver, this file
  *          only decides the order the hardware is brought up in and owns the
  *          backlight PWM.
  ******************************************************************************
  */
#include "dev_lcd.h"
#include "app_debug.h"

#include <stddef.h>
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#ifdef APP_DEBUG
static const char *TAG = "DEV_LCD";     /* only used by the LOGx macros */
#endif

/* Backlight PWM. One timer and one channel of the LEDC are used, the numbers
   are arbitrary as long as the two of them match. */
#define DEV_LCD_BL_TIMER        LEDC_TIMER_0
#define DEV_LCD_BL_CHANNEL      LEDC_CHANNEL_0
#define DEV_LCD_BL_FREQ_HZ      5000U
#define DEV_LCD_BL_MAX_DUTY     ((1U << DEV_LCD_BL_DUTY_BITS) - 1U)

static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_panel_handle_t    s_panel;

/**
 * @brief  Prepare the PWM the backlight hangs on, the output stays dark.
 * @retval ESP_OK on success, the code of the failing step otherwise.
 */
static esp_err_t Dev_Lcd_BacklightInit(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)DEV_LCD_BL_DUTY_BITS,
        .timer_num       = DEV_LCD_BL_TIMER,
        .freq_hz         = DEV_LCD_BL_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel_cfg = {
        .gpio_num   = DEV_LCD_BL_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = DEV_LCD_BL_CHANNEL,
        .timer_sel  = DEV_LCD_BL_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    esp_err_t err;

    err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK)
    {
        LOGE("backlight timer: %s", esp_err_to_name(err));
        return err;
    }
    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK)
    {
        LOGE("backlight channel: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/**
 * @brief  Set the backlight brightness.
 * @note   The percentage is scaled onto the duty of the LEDC here, so the
 *         caller never sees a timer, a channel or a duty register.
 * @param  percent: 0 is dark, 100 is full, more than 100 is clamped.
 * @retval ESP_OK on success, the code of the failing LEDC call otherwise.
 */
esp_err_t dev_lcd_backlight(uint8_t percent)
{
    uint32_t duty;
    esp_err_t err;

    if (percent > 100U)
    {
        percent = 100U;
    }
    duty = ((uint32_t)DEV_LCD_BL_MAX_DUTY * (uint32_t)percent) / 100U;

    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, DEV_LCD_BL_CHANNEL, duty);
    if (err != ESP_OK)
    {
        return err;
    }
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, DEV_LCD_BL_CHANNEL);
}

/**
 * @brief  Handle of the panel IO.
 * @note   The LVGL display needs both handles, that is the only reason they
 *         are handed out here.
 * @retval the handle, NULL until dev_lcd_init() succeeded.
 */
esp_lcd_panel_io_handle_t dev_lcd_io(void)
{
    return s_io;
}

/**
 * @brief  Handle of the panel itself.
 * @retval the handle, NULL until dev_lcd_init() succeeded.
 */
esp_lcd_panel_handle_t dev_lcd_panel(void)
{
    return s_panel;
}

/**
 * @brief  Bring up the bus, the panel and the backlight.
 * @note   The steps run in the order the hardware needs them: the backlight
 *         first but dark, then the bus, the panel IO and the controller.
 * @retval ESP_OK on success, the code of the failing step otherwise.
 */
esp_err_t dev_lcd_init(void)
{
    esp_err_t err;

    s_io = NULL;
    s_panel = NULL;

    /* Step 1: the backlight keeps the panel dark while it is set up, so half
       a frame is never shown. */
    err = Dev_Lcd_BacklightInit();
    if (err != ESP_OK)
    {
        return err;
    }

    /* Step 2: the bus. A panel is write only, so there is no MISO. */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = DEV_LCD_MOSI_GPIO,
        .miso_io_num     = -1,
        .sclk_io_num     = DEV_LCD_SCLK_GPIO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = (int)(DEV_LCD_H_RES * DEV_LCD_BAND_LINES * sizeof(uint16_t)),
    };
    err = spi_bus_initialize(DEV_LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK)
    {
        LOGE("spi bus: %s", esp_err_to_name(err));
        return err;
    }

    /* Step 3: the panel IO, that is where CS, DC and the pixel clock live. */
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num       = DEV_LCD_CS_GPIO,
        .dc_gpio_num       = DEV_LCD_DC_GPIO,
        .spi_mode          = 0,
        .pclk_hz           = DEV_LCD_PCLK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)DEV_LCD_SPI_HOST, &io_cfg, &s_io);
    if (err != ESP_OK)
    {
        LOGE("panel io: %s", esp_err_to_name(err));
        return err;
    }

    /* Step 4: the controller itself. */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = DEV_LCD_RST_GPIO,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7789(s_io, &panel_cfg, &s_panel);
    if (err != ESP_OK)
    {
        LOGE("st7789: %s", esp_err_to_name(err));
        return err;
    }

    /* Step 5: reset, then the vendor table, then the picture. The colour
       polarity is DEV_LCD_INVERT_COLOR, see dev_lcd.h: a module whose picture
       comes out as a negative wants that value flipped. A module that is
       shifted by a few pixels wants set_gap() with the offsets its data sheet
       prints. */
    err = esp_lcd_panel_reset(s_panel);
    if (err != ESP_OK)
    {
        LOGE("reset: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_lcd_panel_init(s_panel);
    if (err != ESP_OK)
    {
        LOGE("init: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_lcd_panel_invert_color(s_panel, (DEV_LCD_INVERT_COLOR != 0));
    if (err != ESP_OK)
    {
        LOGE("invert: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_lcd_panel_set_gap(s_panel, 0, 0);
    if (err != ESP_OK)
    {
        LOGE("gap: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_lcd_panel_disp_on_off(s_panel, true);
    if (err != ESP_OK)
    {
        LOGE("display on: %s", esp_err_to_name(err));
        return err;
    }

    /* Step 6: only now is there something worth looking at. */
    err = dev_lcd_backlight(DEV_LCD_BL_START);
    if (err != ESP_OK)
    {
        return err;
    }

    LOGI("ST7789 %ux%u up, SPI %u MHz, backlight %u%%",
         (unsigned)DEV_LCD_H_RES, (unsigned)DEV_LCD_V_RES,
         (unsigned)(DEV_LCD_PCLK_HZ / 1000000U), (unsigned)DEV_LCD_BL_START);
    return ESP_OK;
}
