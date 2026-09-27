/**
  ******************************************************************************
  * @file    task_display.c
  * @brief   Implementation of the local display task.
  * @note    Two screens, one knob. The turning is handed to LVGL as it is and
  *          moves the focus inside a group. The switch is not: a hold has to be
  *          told apart from a click first, so the device layer latches which of
  *          the two it was and the tick of this file injects the matching
  *          event. That keeps the click and the hold from ever firing together.
  * @note    Nothing here touches the node directly. A transaction is asked for
  *          by setting a request flag, and is run by Task_Display_Poll().
  ******************************************************************************
  */
#include "task_display.h"

#include <stddef.h>
#include <string.h>

#include "app_debug.h"
#include "dev_encoder.h"
#include "dev_lcd.h"
#include "dev_stm32.h"
#include "link_protocol_defs.h"
#include "task_gateway.h"
#include "task_sensor.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#ifdef APP_DEBUG
static const char *TAG = "TASK_DISPLAY";    /* only used by the LOGx macros */
#endif

/* ------------------------------------------------------------------
 * Sizes and timings.
 * ------------------------------------------------------------------ */

/* The LVGL task sits below the link tasks on purpose: the interface may lag,
   the protocol may not. */
#define DISPLAY_TASK_PRIORITY       3
#define DISPLAY_TASK_STACK          6144
#define DISPLAY_TASK_SLEEP_MS       100
#define DISPLAY_TICK_MS             5

/* Two draw buffers of one band each, internal RAM and DMA capable. One tenth
   of the panel is the sweet spot on a SPI panel: low memory, few transactions. */
#define DISPLAY_BUF_LINES           32U

/* The values only move when the node reports, so a slow refresh costs nothing
   and keeps the panel bus quiet. */
#define DISPLAY_REFRESH_MS          200U

/* Housekeeping of the interface, fast enough for a click to feel immediate. */
#define DISPLAY_TICK_PERIOD_MS      25U

/* How long an answer of the node stays on the footer. */
#define DISPLAY_STATUS_MS           4000U

#define DISPLAY_ANIM_MS             200U
#define DISPLAY_ROW_HEIGHT          32
#define DISPLAY_TEXT_LEN            32U

/* ------------------------------------------------------------------
 * Colours of the interface, one place so the look stays consistent.
 * ------------------------------------------------------------------ */
#define DISPLAY_COL_BG              lv_color_hex(0x0F1720)
#define DISPLAY_COL_CARD            lv_color_hex(0x1B2733)
#define DISPLAY_COL_FOCUS           lv_color_hex(0x24405A)
#define DISPLAY_COL_TEXT            lv_color_hex(0xE6EDF3)
#define DISPLAY_COL_DIM             lv_color_hex(0x8296A8)
#define DISPLAY_COL_ACCENT          lv_color_hex(0x3DDC84)
#define DISPLAY_COL_WARN            lv_color_hex(0xFFB300)

#define DISPLAY_HINT_DASH           "turn: move  press: run  hold: stats"
#define DISPLAY_HINT_LINK           "turn: move  press: run  hold: back"

/* ------------------------------------------------------------------
 * Screens, requests and actions.
 * ------------------------------------------------------------------ */
#define DISPLAY_SCREEN_DASH         0U
#define DISPLAY_SCREEN_LINK         1U

/* Left behind by the interface, served by the main loop. */
#define DISPLAY_REQ_NONE            0U
#define DISPLAY_REQ_QUERY           1U
#define DISPLAY_REQ_PERIOD          2U
#define DISPLAY_REQ_BEEP            3U

/* Answers, rendered by the tick of the interface. */
#define DISPLAY_STATUS_NONE         0U
#define DISPLAY_STATUS_QUERY_OK     1U
#define DISPLAY_STATUS_QUERY_FAIL   2U
#define DISPLAY_STATUS_PERIOD_OK    3U
#define DISPLAY_STATUS_PERIOD_FAIL  4U
#define DISPLAY_STATUS_BEEP_OK      5U
#define DISPLAY_STATUS_BEEP_FAIL    6U

/* What a click on a row asks for, handed over as the context of the handler. */
#define DISPLAY_ACTION_QUERY        1U
#define DISPLAY_ACTION_BEEP         2U
#define DISPLAY_ACTION_PERIOD       3U
#define DISPLAY_ACTION_SCREEN       4U

/* Report periods the knob cycles through, the node takes 1000 .. 60000 ms.
   The first entry is what the node reports at until the first click, see the
   default report period of the node side. */
#define DISPLAY_PERIOD_COUNT        4U

static const uint32_t s_periods[DISPLAY_PERIOD_COUNT] = { 2000U, 5000U, 10000U, 30000U };

/* ------------------------------------------------------------------
 * State.
 * ------------------------------------------------------------------ */
static lv_display_t *s_disp;
static lv_indev_t   *s_indev;
static lv_obj_t     *s_scr_dash;
static lv_obj_t     *s_scr_link;
static lv_group_t   *s_group_dash;
static lv_group_t   *s_group_link;

/* Every screen owns its own widgets, the two headers and the two footers are
   therefore two separate pairs. */
static lv_obj_t *s_badge_dash;
static lv_obj_t *s_badge_link;
static lv_obj_t *s_state_dash;
static lv_obj_t *s_state_link;
static lv_obj_t *s_footer_dash;
static lv_obj_t *s_footer_link;

static lv_obj_t *s_lbl_temp;
static lv_obj_t *s_lbl_humi;
static lv_obj_t *s_lbl_buzzer;
static lv_obj_t *s_lbl_period;
static lv_obj_t *s_lbl_frames_ok;
static lv_obj_t *s_lbl_frames_bad;
static lv_obj_t *s_lbl_tx_ok;
static lv_obj_t *s_lbl_tx_timeout;
static lv_obj_t *s_lbl_tx_resend;

/* Written by the LVGL task and read by the main loop, one byte either way. */
static volatile uint8_t s_request;
static volatile uint8_t s_status;
static volatile uint8_t s_status_seq;

/* Written and read by the LVGL task only. */
static uint8_t  s_screen;
static uint8_t  s_click_pending;
static uint8_t  s_back_pending;
static uint8_t  s_shown_status;
static uint8_t  s_shown_seq;
static uint8_t  s_period_index;
static uint8_t  s_buzzer_on;
static uint32_t s_last_refresh;
static uint32_t s_status_until;

/* Handed to the click handler as its context, so no integer is ever cast back
   into a pointer. */
static const uint8_t s_action_query  = DISPLAY_ACTION_QUERY;
static const uint8_t s_action_beep   = DISPLAY_ACTION_BEEP;
static const uint8_t s_action_period = DISPLAY_ACTION_PERIOD;
static const uint8_t s_action_screen = DISPLAY_ACTION_SCREEN;

/* ------------------------------------------------------------------
 * Forward declarations, the builders below use the handler and the helper
 * uses the builders.
 * ------------------------------------------------------------------ */
static void Display_OnAction(lv_event_t *event);
static void Display_BuildDashboard(void);
static void Display_BuildLink(void);
static void Display_ShowScreen(uint8_t screen);
static void Display_Refresh(void);
static void Display_UpdateSettings(void);
static void Display_UpdateFooter(void);

/* ------------------------------------------------------------------
 * Small helpers.
 * ------------------------------------------------------------------ */

/**
 * @brief  Write a label only when its text really changed.
 * @note   Every change turns into traffic on the panel bus, so a comparison
 *         here is worth more than it looks.
 * @param  label: label to write.
 * @param  text:  new text, a NULL is ignored.
 */
static void Display_SetText(lv_obj_t *label, const char *text)
{
    const char *old;

    if ((label == NULL) || (text == NULL))
    {
        return;
    }
    old = lv_label_get_text(label);
    if ((old != NULL) && (strcmp(old, text) == 0))
    {
        return;
    }
    lv_label_set_text(label, text);
}

/**
 * @brief  Turn a value of the link into text with two decimals.
 * @note   The wire carries values scaled by 100, so 2340 means 23.40.
 * @note   Only %d is used: the built in formatter of LVGL does not have to
 *         understand the length modifiers of the C library for this.
 * @param  out:    destination buffer.
 * @param  len:    size of that buffer.
 * @param  scaled: value as it came off the link.
 * @param  unit:   unit appended behind a space.
 */
static void Display_FormatScaled(char *out, size_t len, int32_t scaled, const char *unit)
{
    int32_t whole = scaled / 100;
    int32_t frac = scaled % 100;

    if (frac < 0)
    {
        frac = -frac;
    }
    if ((scaled < 0) && (whole == 0))
    {
        lv_snprintf(out, len, "-%d.%02d %s", (int)whole, (int)frac, unit);
    }
    else
    {
        lv_snprintf(out, len, "%d.%02d %s", (int)whole, (int)frac, unit);
    }
}

/**
 * @brief  Text of one status code.
 * @param  code: DISPLAY_STATUS_xxx.
 * @retval the text, empty for DISPLAY_STATUS_NONE.
 */
static const char *Display_StatusText(uint8_t code)
{
    switch (code)
    {
        case DISPLAY_STATUS_QUERY_OK:
            return "query ok";
        case DISPLAY_STATUS_QUERY_FAIL:
            return "query failed";
        case DISPLAY_STATUS_PERIOD_OK:
            return "period set";
        case DISPLAY_STATUS_PERIOD_FAIL:
            return "period rejected";
        case DISPLAY_STATUS_BEEP_OK:
            return "buzzer set";
        case DISPLAY_STATUS_BEEP_FAIL:
            return "buzzer rejected";
        default:
            return "";
    }
}

/* ------------------------------------------------------------------
 * Building blocks of the two screens.
 * ------------------------------------------------------------------ */

/**
 * @brief  Create the root of one screen with the shared background.
 * @retval the screen object.
 */
static lv_obj_t *Display_CreateScreen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(scr, DISPLAY_COL_BG, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_set_style_pad_all(scr, 8, 0);
    lv_obj_set_style_pad_row(scr, 8, 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_scrollable(scr, false);
    return scr;
}

/**
 * @brief  Create the title bar with the link state badge on its right.
 * @param  parent:    screen the bar belongs to.
 * @param  title:     text on the left.
 * @param  badge_out: receives the badge, its colour is the link state.
 * @param  state_out: receives the label inside the badge.
 */
static void Display_CreateHeader(lv_obj_t *parent, const char *title,
                                 lv_obj_t **badge_out, lv_obj_t **state_out)
{
    lv_obj_t *header = lv_obj_create(parent);
    lv_obj_t *label;
    lv_obj_t *badge;
    lv_obj_t *state;

    lv_obj_set_size(header, LV_PCT(100), 30);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(header, 0, 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_scrollable(header, false);

    label = lv_label_create(header);
    lv_label_set_text(label, title);
    lv_obj_set_style_text_color(label, DISPLAY_COL_TEXT, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);

    badge = lv_obj_create(header);
    lv_obj_set_size(badge, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(badge, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(badge, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_hor(badge, 8, 0);
    lv_obj_set_style_pad_ver(badge, 2, 0);
    lv_obj_set_style_radius(badge, 10, 0);
    lv_obj_set_style_bg_color(badge, DISPLAY_COL_WARN, 0);
    lv_obj_set_style_border_width(badge, 0, 0);
    lv_obj_set_scrollable(badge, false);

    state = lv_label_create(badge);
    lv_label_set_text(state, "OFFLINE");
    lv_obj_set_style_text_color(state, DISPLAY_COL_BG, 0);
    lv_obj_set_style_text_font(state, &lv_font_montserrat_12, 0);

    *badge_out = badge;
    *state_out = state;
}

/**
 * @brief  Create one of the two value tiles of the dashboard.
 * @param  parent:    row the tile belongs to.
 * @param  caption:   name of the value.
 * @param  value_out: receives the label that carries the number.
 */
static void Display_CreateTile(lv_obj_t *parent, const char *caption, lv_obj_t **value_out)
{
    lv_obj_t *tile = lv_obj_create(parent);
    lv_obj_t *cap;

    lv_obj_set_flex_grow(tile, 1);
    lv_obj_set_height(tile, LV_PCT(100));
    lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(tile, 4, 0);
    lv_obj_set_style_radius(tile, 8, 0);
    lv_obj_set_style_bg_color(tile, DISPLAY_COL_CARD, 0);
    lv_obj_set_style_border_width(tile, 0, 0);
    lv_obj_set_scrollable(tile, false);

    cap = lv_label_create(tile);
    lv_label_set_text(cap, caption);
    lv_obj_set_style_text_color(cap, DISPLAY_COL_DIM, 0);
    lv_obj_set_style_text_font(cap, &lv_font_montserrat_12, 0);

    *value_out = lv_label_create(tile);
    lv_label_set_text(*value_out, "--");
    lv_obj_set_style_text_color(*value_out, DISPLAY_COL_ACCENT, 0);
    lv_obj_set_style_text_font(*value_out, &lv_font_montserrat_20, 0);
}

/**
 * @brief  Create one row: caption on the left, value or hint on the right.
 * @param  parent:    column the row belongs to.
 * @param  caption:   text on the left.
 * @param  value_out: receives the right hand label, may be NULL.
 * @param  group:     focus group that should hold the row, NULL keeps it out.
 * @param  action:    context handed to the click handler, NULL for a row that
 *                    is only there to be read.
 */
static void Display_CreateRow(lv_obj_t *parent, const char *caption, lv_obj_t **value_out,
                              lv_group_t *group, const uint8_t *action)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_t *cap;

    lv_obj_set_size(row, LV_PCT(100), DISPLAY_ROW_HEIGHT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_hor(row, 8, 0);
    lv_obj_set_style_pad_ver(row, 0, 0);
    lv_obj_set_style_radius(row, 6, 0);
    lv_obj_set_style_bg_color(row, DISPLAY_COL_CARD, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_scrollable(row, false);

    cap = lv_label_create(row);
    lv_label_set_text(cap, caption);
    lv_obj_set_style_text_color(cap, DISPLAY_COL_TEXT, 0);
    lv_obj_set_style_text_font(cap, &lv_font_montserrat_14, 0);

    if (value_out != NULL)
    {
        *value_out = lv_label_create(row);
        lv_label_set_text(*value_out, "");
        lv_obj_set_style_text_color(*value_out, DISPLAY_COL_DIM, 0);
        lv_obj_set_style_text_font(*value_out, &lv_font_montserrat_14, 0);
    }

    if ((group != NULL) && (action != NULL))
    {
        lv_obj_set_clickable(row, true);
        (void)lv_obj_add_event_cb(row, Display_OnAction, LV_EVENT_CLICKED, (void *)action);
        lv_obj_set_style_bg_color(row, DISPLAY_COL_FOCUS, LV_STATE_FOCUSED);
        lv_obj_set_style_outline_color(row, DISPLAY_COL_ACCENT, LV_STATE_FOCUSED);
        lv_obj_set_style_outline_width(row, 2, LV_STATE_FOCUSED);
        lv_obj_set_style_outline_pad(row, 0, LV_STATE_FOCUSED);
        lv_group_add_obj(group, row);
    }
}

/**
 * @brief  Create the hint line at the bottom of a screen.
 * @param  parent: screen the line belongs to.
 * @retval the label, each screen needs its own.
 */
static lv_obj_t *Display_CreateFooter(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);

    lv_label_set_text(label, "");
    lv_obj_set_style_text_color(label, DISPLAY_COL_DIM, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
    return label;
}

/**
 * @brief  Build the column that holds the rows of one screen.
 * @param  parent: screen the column belongs to.
 * @retval the column.
 */
static lv_obj_t *Display_CreateRowList(lv_obj_t *parent)
{
    lv_obj_t *list = lv_obj_create(parent);

    lv_obj_set_width(list, LV_PCT(100));
    lv_obj_set_flex_grow(list, 1);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_set_style_pad_row(list, 4, 0);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scrollable(list, false);
    return list;
}

/* ------------------------------------------------------------------
 * The two screens.
 * ------------------------------------------------------------------ */

/**
 * @brief  Dashboard: the live values plus the four rows the knob acts on.
 */
static void Display_BuildDashboard(void)
{
    lv_obj_t *cards;
    lv_obj_t *list;

    s_scr_dash = Display_CreateScreen();
    s_group_dash = lv_group_create();
    lv_group_set_wrap(s_group_dash, true);

    Display_CreateHeader(s_scr_dash, "Bloomcare", &s_badge_dash, &s_state_dash);

    cards = lv_obj_create(s_scr_dash);
    lv_obj_set_size(cards, LV_PCT(100), 78);
    lv_obj_set_flex_flow(cards, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cards, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(cards, 0, 0);
    lv_obj_set_style_pad_column(cards, 6, 0);
    lv_obj_set_style_bg_opa(cards, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cards, 0, 0);
    lv_obj_set_scrollable(cards, false);
    Display_CreateTile(cards, "TEMP", &s_lbl_temp);
    Display_CreateTile(cards, "HUMI", &s_lbl_humi);

    list = Display_CreateRowList(s_scr_dash);
    Display_CreateRow(list, "Query now", NULL, s_group_dash, &s_action_query);
    Display_CreateRow(list, "Buzzer", &s_lbl_buzzer, s_group_dash, &s_action_beep);
    Display_CreateRow(list, "Period", &s_lbl_period, s_group_dash, &s_action_period);
    Display_CreateRow(list, "Link stats", NULL, s_group_dash, &s_action_screen);

    s_footer_dash = Display_CreateFooter(s_scr_dash);
}

/**
 * @brief  Link screen: the counters of the device layer, read only.
 */
static void Display_BuildLink(void)
{
    lv_obj_t *list;

    s_scr_link = Display_CreateScreen();
    s_group_link = lv_group_create();
    lv_group_set_wrap(s_group_link, true);

    Display_CreateHeader(s_scr_link, "Link", &s_badge_link, &s_state_link);

    list = Display_CreateRowList(s_scr_link);
    Display_CreateRow(list, "Frames ok", &s_lbl_frames_ok, NULL, NULL);
    Display_CreateRow(list, "Frames bad", &s_lbl_frames_bad, NULL, NULL);
    Display_CreateRow(list, "TX ok", &s_lbl_tx_ok, NULL, NULL);
    Display_CreateRow(list, "TX timeout", &s_lbl_tx_timeout, NULL, NULL);
    Display_CreateRow(list, "Resends", &s_lbl_tx_resend, NULL, NULL);
    Display_CreateRow(list, "Back", NULL, s_group_link, &s_action_screen);

    s_footer_link = Display_CreateFooter(s_scr_link);
}

/* ------------------------------------------------------------------
 * Behaviour of the interface.
 * ------------------------------------------------------------------ */

/**
 * @brief  Note what a click on one row asks for.
 * @note   Nothing is run here. The interface only sets the request, the main
 *         loop serves it, so a transaction can never stall the LVGL task.
 * @param  event: the click event of the row.
 */
static void Display_OnAction(lv_event_t *event)
{
    const uint8_t *action = (const uint8_t *)lv_event_get_user_data(event);

    if (action == NULL)
    {
        return;
    }
    switch (*action)
    {
        case DISPLAY_ACTION_QUERY:
            s_request = DISPLAY_REQ_QUERY;
            break;
        case DISPLAY_ACTION_BEEP:
            s_request = DISPLAY_REQ_BEEP;
            break;
        case DISPLAY_ACTION_PERIOD:
            s_request = DISPLAY_REQ_PERIOD;
            break;
        case DISPLAY_ACTION_SCREEN:
            Display_ShowScreen((s_screen == DISPLAY_SCREEN_DASH) ? DISPLAY_SCREEN_LINK
                                                                : DISPLAY_SCREEN_DASH);
            break;
        default:
            break;
    }
}

/**
 * @brief  Slide from one screen to the other and hand the knob over.
 * @param  screen: DISPLAY_SCREEN_xxx.
 */
static void Display_ShowScreen(uint8_t screen)
{
    if (screen == s_screen)
    {
        return;
    }
    s_screen = screen;
    if (screen == DISPLAY_SCREEN_LINK)
    {
        lv_screen_load_anim(s_scr_link, LV_SCREEN_LOAD_ANIM_MOVE_LEFT, DISPLAY_ANIM_MS, 0, false);
        lv_indev_set_group(s_indev, s_group_link);
    }
    else
    {
        lv_screen_load_anim(s_scr_dash, LV_SCREEN_LOAD_ANIM_MOVE_RIGHT, DISPLAY_ANIM_MS, 0, false);
        lv_indev_set_group(s_indev, s_group_dash);
    }
    if (s_shown_status == DISPLAY_STATUS_NONE)
    {
        lv_label_set_text((screen == DISPLAY_SCREEN_LINK) ? s_footer_link : s_footer_dash,
                          (screen == DISPLAY_SCREEN_LINK) ? DISPLAY_HINT_LINK
                                                          : DISPLAY_HINT_DASH);
    }
}

/**
 * @brief  Read the knob once, the way LVGL asks for it.
 * @note   The turning is handed over as it comes, the switch is not. LVGL has
 *         no way of telling a click from a hold, so the device layer latches
 *         the decision and the tick injects the matching event afterwards.
 * @param  indev: the encoder device, unused.
 * @param  data:  what LVGL has to be told.
 */
static void Display_EncoderRead(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint8_t events;

    (void)indev;
    events = dev_encoder_poll();
    if ((events & DEV_ENC_EV_LONG) != 0U)
    {
        s_back_pending = 1U;
    }
    else if ((events & DEV_ENC_EV_SHORT) != 0U)
    {
        s_click_pending = 1U;
    }

    data->enc_diff = (int16_t)dev_encoder_take_steps();
    data->state = LV_INDEV_STATE_RELEASED;
}

/**
 * @brief  Activate whatever the knob points at.
 */
static void Display_ActivateFocused(void)
{
    lv_group_t *group = (s_screen == DISPLAY_SCREEN_LINK) ? s_group_link : s_group_dash;
    lv_obj_t *focused = lv_group_get_focused(group);

    if (focused != NULL)
    {
        (void)lv_obj_send_event(focused, LV_EVENT_CLICKED, NULL);
    }
}

/**
 * @brief  Draw the values of the node into the tiles and the state badge.
 */
static void Display_Refresh(void)
{
    char text[DISPLAY_TEXT_LEN];
    int32_t value;
    uint8_t online;

    online = Task_Sensor_IsOnline();
    Display_SetText(s_state_dash, (online != 0U) ? "ONLINE" : "OFFLINE");
    Display_SetText(s_state_link, (online != 0U) ? "ONLINE" : "OFFLINE");
    lv_obj_set_style_bg_color(s_badge_dash,
                              (online != 0U) ? DISPLAY_COL_ACCENT : DISPLAY_COL_WARN, 0);
    lv_obj_set_style_bg_color(s_badge_link,
                              (online != 0U) ? DISPLAY_COL_ACCENT : DISPLAY_COL_WARN, 0);

    if (Task_Sensor_Get(LINK_ID_TEMP, &value) != 0U)
    {
        Display_FormatScaled(text, sizeof(text), value, "C");
        Display_SetText(s_lbl_temp, text);
    }
    else
    {
        Display_SetText(s_lbl_temp, "--");
    }

    if (Task_Sensor_Get(LINK_ID_HUMI, &value) != 0U)
    {
        Display_FormatScaled(text, sizeof(text), value, "%");
        Display_SetText(s_lbl_humi, text);
    }
    else
    {
        Display_SetText(s_lbl_humi, "--");
    }

    if (s_screen != DISPLAY_SCREEN_LINK)
    {
        return;
    }

    /* The counters of the link are the counters of the device layer, this is
       the one screen that shows them. */
    dev_stm32_stats_t stats;
    dev_stm32_get_stats(&stats);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.frame_ok);
    Display_SetText(s_lbl_frames_ok, text);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.frame_bad);
    Display_SetText(s_lbl_frames_bad, text);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.tx_ok);
    Display_SetText(s_lbl_tx_ok, text);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.tx_timeout);
    Display_SetText(s_lbl_tx_timeout, text);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.tx_resend);
    Display_SetText(s_lbl_tx_resend, text);
}

/**
 * @brief  Draw the two rows the knob changes.
 * @note   They carry what the interface last asked for, not what the node
 *          confirmed, so they are updated from the request side of the state.
 */
static void Display_UpdateSettings(void)
{
    char text[DISPLAY_TEXT_LEN];

    Display_SetText(s_lbl_buzzer, (s_buzzer_on != 0U) ? "ON" : "OFF");
    lv_snprintf(text, sizeof(text), "%d s", (int)(s_periods[s_period_index] / 1000U));
    Display_SetText(s_lbl_period, text);
}

/**
 * @brief  Show the answer of the node, or the hint of the screen once it aged.
 */
static void Display_UpdateFooter(void)
{
    lv_obj_t *footer = (s_screen == DISPLAY_SCREEN_LINK) ? s_footer_link : s_footer_dash;

    if (s_status_seq != s_shown_seq)
    {
        s_shown_seq = s_status_seq;
        s_shown_status = s_status;
        lv_label_set_text(footer, Display_StatusText(s_shown_status));
        lv_obj_set_style_text_color(footer,
                                    (s_shown_status == DISPLAY_STATUS_NONE)
                                        ? DISPLAY_COL_DIM
                                        : DISPLAY_COL_ACCENT,
                                    0);
        s_status_until = lv_tick_get() + DISPLAY_STATUS_MS;
        return;
    }

    if ((s_shown_status != DISPLAY_STATUS_NONE) &&
        ((int32_t)(lv_tick_get() - s_status_until) >= 0))
    {
        s_shown_status = DISPLAY_STATUS_NONE;
        lv_label_set_text(footer, (s_screen == DISPLAY_SCREEN_LINK) ? DISPLAY_HINT_LINK
                                                                    : DISPLAY_HINT_DASH);
        lv_obj_set_style_text_color(footer, DISPLAY_COL_DIM, 0);
    }
}

/**
 * @brief  Tick of the interface: pending events, then the slow redraw.
 * @param  timer: the timer itself, unused.
 */
static void Display_Tick(lv_timer_t *timer)
{
    uint32_t now;

    (void)timer;

    if (s_back_pending != 0U)
    {
        s_back_pending = 0U;
        Display_ShowScreen((s_screen == DISPLAY_SCREEN_DASH) ? DISPLAY_SCREEN_LINK
                                                            : DISPLAY_SCREEN_DASH);
    }
    if (s_click_pending != 0U)
    {
        s_click_pending = 0U;
        Display_ActivateFocused();
    }

    Display_UpdateSettings();
    Display_UpdateFooter();

    now = lv_tick_get();
    if ((uint32_t)(now - s_last_refresh) < DISPLAY_REFRESH_MS)
    {
        return;
    }
    s_last_refresh = now;
    Display_Refresh();
}

/* ------------------------------------------------------------------
 * The transactions the interface asks the main loop for.
 * ------------------------------------------------------------------ */

/**
 * @brief  Ask the node for its values, without waiting for its next report.
 */
static void Display_RunQuery(void)
{
    dev_item_t items[LINK_QUERY_MAX_ITEMS];
    uint8_t got = 0U;
    esp_err_t err;

    err = Task_Gateway_RequestValues(0U, items, (uint8_t)LINK_QUERY_MAX_ITEMS, &got);
    s_status = (err == ESP_OK) ? DISPLAY_STATUS_QUERY_OK : DISPLAY_STATUS_QUERY_FAIL;
    s_status_seq++;
    LOGI("display query -> %s, %u item(s)", esp_err_to_name(err), (unsigned)got);
}

/**
 * @brief  Move the node to the next report period of the list.
 */
static void Display_RunPeriod(void)
{
    esp_err_t err;
    uint8_t next = (uint8_t)((s_period_index + 1U) % DISPLAY_PERIOD_COUNT);

    err = Task_Gateway_SetReportPeriod(s_periods[next]);
    if (err == ESP_OK)
    {
        s_period_index = next;
    }
    s_status = (err == ESP_OK) ? DISPLAY_STATUS_PERIOD_OK : DISPLAY_STATUS_PERIOD_FAIL;
    s_status_seq++;
    LOGI("display period -> %u ms: %s", (unsigned)s_periods[next], esp_err_to_name(err));
}

/**
 * @brief  Toggle the buzzer of the node.
 */
static void Display_RunBeep(void)
{
    esp_err_t err;
    uint8_t wanted = (s_buzzer_on == 0U) ? 1U : 0U;

    err = Task_Gateway_SetBuzzer(wanted != 0U);
    if (err == ESP_OK)
    {
        s_buzzer_on = wanted;
    }
    s_status = (err == ESP_OK) ? DISPLAY_STATUS_BEEP_OK : DISPLAY_STATUS_BEEP_FAIL;
    s_status_seq++;
    LOGI("display buzzer -> %s: %s", (wanted != 0U) ? "on" : "off", esp_err_to_name(err));
}

/* ------------------------------------------------------------------
 * Bring up and poll.
 * ------------------------------------------------------------------ */

/**
 * @brief  Bring up the panel, the graphics library and the knob.
 * @note   Every step is allowed to fail on its own: without a panel there is
 *         nothing to serve, and without a knob the interface stays readable
 *         but cannot be operated. Neither failure blocks the rest of the boot.
 */
void Task_Display_Init(void)
{
    esp_err_t err;
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_display_cfg_t disp_cfg;

    s_request = DISPLAY_REQ_NONE;
    s_status = DISPLAY_STATUS_NONE;
    s_status_seq = 0U;
    s_screen = DISPLAY_SCREEN_DASH;
    s_click_pending = 0U;
    s_back_pending = 0U;
    s_shown_status = DISPLAY_STATUS_NONE;
    s_shown_seq = 0U;
    s_period_index = 0U;
    s_buzzer_on = 0U;
    s_last_refresh = 0U;
    s_status_until = 0U;

    err = dev_lcd_init();
    if (err != ESP_OK)
    {
        LOGE("no panel, the interface stays dark: %s", esp_err_to_name(err));
        return;
    }

    port_cfg.task_priority = DISPLAY_TASK_PRIORITY;
    port_cfg.task_stack = DISPLAY_TASK_STACK;
    port_cfg.task_max_sleep_ms = DISPLAY_TASK_SLEEP_MS;
    port_cfg.timer_period_ms = DISPLAY_TICK_MS;
    err = lvgl_port_init(&port_cfg);
    if (err != ESP_OK)
    {
        LOGE("lvgl port: %s", esp_err_to_name(err));
        return;
    }

    memset(&disp_cfg, 0, sizeof(disp_cfg));
    disp_cfg.io_handle = dev_lcd_io();
    disp_cfg.panel_handle = dev_lcd_panel();
    disp_cfg.buffer_size = DEV_LCD_H_RES * DISPLAY_BUF_LINES;
    disp_cfg.double_buffer = true;
    disp_cfg.hres = DEV_LCD_H_RES;
    disp_cfg.vres = DEV_LCD_V_RES;
    disp_cfg.monochrome = false;
    disp_cfg.color_format = LV_COLOR_FORMAT_RGB565;
    disp_cfg.flags.buff_dma = 1;
    disp_cfg.flags.swap_bytes = 1;

    if (lvgl_port_lock(0) == false)
    {
        LOGE("lvgl lock failed while adding the display");
        return;
    }
    s_disp = lvgl_port_add_disp(&disp_cfg);
    if (s_disp == NULL)
    {
        lvgl_port_unlock();
        LOGE("the display refused to register with LVGL");
        return;
    }
    Display_BuildDashboard();
    Display_BuildLink();
    lv_label_set_text(s_footer_dash, DISPLAY_HINT_DASH);
    lv_label_set_text(s_footer_link, DISPLAY_HINT_LINK);
    lv_screen_load(s_scr_dash);
    Display_UpdateSettings();
    Display_Refresh();
    lvgl_port_unlock();

    err = dev_encoder_init();
    if (err != ESP_OK)
    {
        LOGE("no knob, the interface stays read only: %s", esp_err_to_name(err));
        return;
    }

    if (lvgl_port_lock(0) == false)
    {
        LOGE("lvgl lock failed while adding the knob");
        return;
    }
    s_indev = lv_indev_create();
    lv_indev_set_type(s_indev, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(s_indev, Display_EncoderRead);
    lv_indev_set_display(s_indev, s_disp);
    lv_indev_set_group(s_indev, s_group_dash);
    (void)lv_timer_create(Display_Tick, DISPLAY_TICK_PERIOD_MS, NULL);
    lvgl_port_unlock();

    LOGI("interface up, %ux%u", (unsigned)DEV_LCD_H_RES, (unsigned)DEV_LCD_V_RES);
}

/**
 * @brief  Serve the display task once, call it every pass of the main loop.
 * @note   Only the request the interface left behind is served here, which is
 *         what keeps a blocking transaction out of the LVGL task.
 * @param  now_ms: current millisecond tick, not needed by this task.
 */
void Task_Display_Poll(uint32_t now_ms)
{
    uint8_t request = s_request;

    (void)now_ms;
    if (request == DISPLAY_REQ_NONE)
    {
        return;
    }
    s_request = DISPLAY_REQ_NONE;

    switch (request)
    {
        case DISPLAY_REQ_QUERY:
            Display_RunQuery();
            break;
        case DISPLAY_REQ_PERIOD:
            Display_RunPeriod();
            break;
        case DISPLAY_REQ_BEEP:
            Display_RunBeep();
            break;
        default:
            break;
    }
}
