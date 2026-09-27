/**
  ******************************************************************************
  * @file    dev_encoder.c
  * @brief   Implementation of the rotary knob device.
  * @note    The turning is decoded by a PCNT unit and only read here, the push
  *          switch is sampled and debounced here. Nothing in this file knows
  *          what a turn or a click is used for.
  ******************************************************************************
  */
#include "dev_encoder.h"
#include "app_debug.h"

#include <stddef.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

#ifdef APP_DEBUG
static const char *TAG = "DEV_ENCODER";     /* only used by the LOGx macros */
#endif

#define DEV_ENC_US_PER_MS       1000

static pcnt_unit_handle_t s_unit;
static pcnt_channel_handle_t s_chan_a;
static pcnt_channel_handle_t s_chan_b;

/* Counts of a turn that did not reach a whole detent yet. */
static int s_residual;

/* Switch state machine. The raw level is the pin, the stable level is what the
   debounce agreed on, and the time stamp belongs to the last raw change. */
static uint8_t s_button_raw;
static uint8_t s_button_stable;
static int64_t s_raw_changed_us;
static int64_t s_pressed_us;

/**
 * @brief  Configure the two quadrature pins and the switch pin as inputs.
 * @retval ESP_OK on success.
 */
static esp_err_t Dev_Encoder_GpioInit(void)
{
    /* The module carries pull ups of its own, the internal ones are enabled as
       well so a bare encoder without that little board works just the same. */
    gpio_config_t quad_cfg = {
        .pin_bit_mask = (1ULL << DEV_ENC_A_GPIO) | (1ULL << DEV_ENC_B_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config_t sw_cfg = {
        .pin_bit_mask = (1ULL << DEV_ENC_SW_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err;

    err = gpio_config(&quad_cfg);
    if (err != ESP_OK)
    {
        LOGE("quadrature pins: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_config(&sw_cfg);
    if (err != ESP_OK)
    {
        LOGE("switch pin: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/**
 * @brief  Arm the PCNT unit as a four count per detent quadrature decoder.
 * @note   Channel A counts on the edges of A and reads the level of B to decide
 *         the direction, channel B does the exact mirror image. Together the two
 *         of them count every edge of the cycle, which is what makes four counts
 *         out of one detent.
 * @retval ESP_OK on success, the code of the failing step otherwise.
 */
static esp_err_t Dev_Encoder_PcntInit(void)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -DEV_ENC_PCNT_LIMIT,
        .high_limit = DEV_ENC_PCNT_LIMIT,
    };
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000,
    };
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = DEV_ENC_A_GPIO,
        .level_gpio_num = DEV_ENC_B_GPIO,
    };
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = DEV_ENC_B_GPIO,
        .level_gpio_num = DEV_ENC_A_GPIO,
    };
    esp_err_t err;

    err = pcnt_new_unit(&unit_cfg, &s_unit);
    if (err != ESP_OK)
    {
        LOGE("pcnt unit: %s", esp_err_to_name(err));
        return err;
    }

    /* A contact that bounces shorter than a microsecond is noise, not a turn. */
    err = pcnt_unit_set_glitch_filter(s_unit, &filter_cfg);
    if (err != ESP_OK)
    {
        LOGW("glitch filter: %s", esp_err_to_name(err));
    }

    err = pcnt_new_channel(s_unit, &chan_a_cfg, &s_chan_a);
    if (err != ESP_OK)
    {
        LOGE("pcnt channel a: %s", esp_err_to_name(err));
        return err;
    }
    err = pcnt_new_channel(s_unit, &chan_b_cfg, &s_chan_b);
    if (err != ESP_OK)
    {
        LOGE("pcnt channel b: %s", esp_err_to_name(err));
        return err;
    }

    err = pcnt_channel_set_edge_action(s_chan_a,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err != ESP_OK)
    {
        LOGE("channel a edges: %s", esp_err_to_name(err));
        return err;
    }
    err = pcnt_channel_set_level_action(s_chan_a,
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK)
    {
        LOGE("channel a levels: %s", esp_err_to_name(err));
        return err;
    }
    err = pcnt_channel_set_edge_action(s_chan_b,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (err != ESP_OK)
    {
        LOGE("channel b edges: %s", esp_err_to_name(err));
        return err;
    }
    err = pcnt_channel_set_level_action(s_chan_b,
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK)
    {
        LOGE("channel b levels: %s", esp_err_to_name(err));
        return err;
    }

    err = pcnt_unit_enable(s_unit);
    if (err != ESP_OK)
    {
        LOGE("pcnt enable: %s", esp_err_to_name(err));
        return err;
    }
    err = pcnt_unit_clear_count(s_unit);
    if (err != ESP_OK)
    {
        LOGE("pcnt clear: %s", esp_err_to_name(err));
        return err;
    }
    err = pcnt_unit_start(s_unit);
    if (err != ESP_OK)
    {
        LOGE("pcnt start: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/**
 * @brief  Bring up the quadrature decoder and the push switch.
 * @note   The two quadrature pins and the switch pin are claimed as plain
 *         inputs, the PCNT then takes the two of them over.
 * @retval ESP_OK on success, the code of the failing step otherwise.
 */
esp_err_t dev_encoder_init(void)
{
    esp_err_t err;
    int64_t now;

    err = Dev_Encoder_GpioInit();
    if (err != ESP_OK)
    {
        return err;
    }
    err = Dev_Encoder_PcntInit();
    if (err != ESP_OK)
    {
        return err;
    }

    now = esp_timer_get_time();
    s_residual = 0;
    s_button_raw = (gpio_get_level(DEV_ENC_SW_GPIO) == 0) ? 1U : 0U;
    s_button_stable = s_button_raw;
    s_raw_changed_us = now;
    s_pressed_us = now;

    LOGI("knob up on A=%d B=%d SW=%d, %d counts per detent",
         (int)DEV_ENC_A_GPIO, (int)DEV_ENC_B_GPIO, (int)DEV_ENC_SW_GPIO,
         (int)DEV_ENC_COUNTS_PER_STEP);
    return ESP_OK;
}

/**
 * @brief  Read the push switch once and report what happened since the last call.
 * @note   A press is only turned into an event on the release, because only
 *         then is it known whether it was a click or a hold.
 * @retval DEV_ENC_EV_NONE, or one of DEV_ENC_EV_SHORT / DEV_ENC_EV_LONG.
 */
uint8_t dev_encoder_poll(void)
{
    int64_t now = esp_timer_get_time();
    uint8_t raw;
    uint8_t events = DEV_ENC_EV_NONE;

    /* The switch of these modules pulls the line down, so low is pressed. */
    raw = (gpio_get_level(DEV_ENC_SW_GPIO) == 0) ? 1U : 0U;

    if (raw != s_button_raw)
    {
        s_button_raw = raw;
        s_raw_changed_us = now;
    }
    else if ((raw != s_button_stable) &&
             ((now - s_raw_changed_us) >= ((int64_t)DEV_ENC_DEBOUNCE_MS * DEV_ENC_US_PER_MS)))
    {
        s_button_stable = raw;
        if (raw != 0U)
        {
            s_pressed_us = now;
        }
        else if ((now - s_pressed_us) >= ((int64_t)DEV_ENC_LONG_PRESS_MS * DEV_ENC_US_PER_MS))
        {
            events |= DEV_ENC_EV_LONG;
        }
        else
        {
            events |= DEV_ENC_EV_SHORT;
        }
    }
    return events;
}

/**
 * @brief  Level of the push switch, already debounced.
 * @retval 1 while the knob is held down, 0 otherwise.
 */
uint8_t dev_encoder_button_down(void)
{
    return s_button_stable;
}

/**
 * @brief  Detents the knob turned since the last call, positive is clockwise.
 * @note   The hardware counter is cleared here. A part of a detent is carried
 *         over into the next call, so a slow turn is never swallowed.
 * @retval number of detents, negative when the knob went the other way.
 */
int dev_encoder_take_steps(void)
{
    int raw = 0;
    int steps;

    if (s_unit == NULL)
    {
        return 0;
    }
    if (pcnt_unit_get_count(s_unit, &raw) != ESP_OK)
    {
        return 0;
    }
    (void)pcnt_unit_clear_count(s_unit);

    s_residual += raw;
    steps = s_residual / DEV_ENC_COUNTS_PER_STEP;
    s_residual -= steps * DEV_ENC_COUNTS_PER_STEP;
    return steps;
}
