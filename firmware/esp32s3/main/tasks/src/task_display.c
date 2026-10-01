/**
  ******************************************************************************
  * @file    task_display.c
  * @brief   Implementation of the local display task: the local interface of the
  *          gateway, a 240x320 ST7789 panel driven by the rotary knob.
 * @note    Four screens share one status bar and one knob:
 *            main     the six cards of the environment monitor
 *            detail   one reading, its extremes and its two thresholds
 *            settings what the knob can change, plus the entry to the counters
  *            link     the counters of the device layer, read only
  * @note    The grid is placed on explicit pixels instead of being left to a
  *          flex layout: the panel is a fixed 240x320, so every card sits on the
  *          pixel it belongs to and the layout is symmetric by construction
  *          rather than by luck. See the arithmetic above UI_STATUS_H.
  * @note    Nothing here touches the node directly. A transaction is asked for by
  *          setting a request flag and is run by Task_Display_Poll(), so a
  *          transaction that blocks for up to a second can never stall the LVGL
  *          task and freeze the interface.
  * @note    The degree sign of the temperature unit is written as the UTF-8
  *          escape "\xC2\xB0" so this file stays pure ASCII; the glyph itself is
  *          part of the built in Montserrat fonts, whose second cmap starts at
  *          U+00B0, see lv_font_montserrat_20.c.
  ******************************************************************************
  */
#include "task_display.h"

#include <stddef.h>
#include <string.h>
#include <time.h>

#include "app_debug.h"
#include "dev_encoder.h"
#include "dev_lcd.h"
#include "dev_stm32.h"
#include "link_protocol_defs.h"
#include "task_gateway.h"
#include "task_net.h"
#include "task_sensor.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#ifdef APP_DEBUG
static const char *TAG = "TASK_DISPLAY";    /* only used by the LOGx macros */
#endif

/* ------------------------------------------------------------------
 * Task sizing and timings.
 * ------------------------------------------------------------------ */

/* The LVGL task sits below the link tasks on purpose: the interface may lag,
   the protocol may not. */
#define UI_TASK_PRIORITY        3
#define UI_TASK_STACK           6144
#define UI_TASK_SLEEP_MS        100
#define UI_TASK_TICK_MS         5

/* Two draw buffers of one band each, internal RAM and DMA capable. One tenth of
   the panel is the sweet spot on a SPI panel: low memory, few transactions. */
#define UI_BUF_LINES            32U

/* The values only move when the node reports, so a slow refresh costs nothing.
   The tick runs four times faster so a click feels immediate. */
#define UI_REFRESH_MS           200U
#define UI_TICK_PERIOD_MS       25U

/* How long a message from the settings screen owns the title slot. */
#define UI_STATUS_MS            3000U

/* Fade of a value that just changed, in milliseconds. */
#define UI_ANIM_MS              180U

/* Longest text of the interface: "NO DATA" is the longest one that is fixed. */
#define UI_TEXT_LEN             24U

/* ------------------------------------------------------------------
 * Grid geometry.
 *
 *   240 x 320 portrait, 24 px status bar on top.
 *
 *   Horizontally the two side margins and the column gap are 6 px:
 *     6 + 111 + 6 + 111 + 6 = 240
 *
 *   Vertically the same 6 px gap leaves 90 px per card, and what is left over is
 *   split evenly above and below the grid:
 *     24 + 7 + 90 + 6 + 90 + 6 + 90 + 7 = 320
 *
 *   Both those strip margins are 7 px and not 6 px on purpose: with 6 px a card
 *   would have to be 90.67 px tall, LVGL would round each of the three rows on
 *   its own and the rows would no longer be the same height. Every card being
 *   exactly 111 x 90 is worth more than the one pixel.
 *
 *   The rows of the detail, the settings and the link screen use the same strip,
 *   so the six of them come out at 42 px each:
 *     24 + 7 + 42 + 6 + ... + 42 + 7 = 320
 * ------------------------------------------------------------------ */
#define UI_STATUS_H             24
#define UI_MARGIN_X             6
#define UI_GRID_X               UI_MARGIN_X
#define UI_GRID_Y               (UI_STATUS_H + 7)
#define UI_GAP                  6
#define UI_CARD_W               111
#define UI_CARD_H               90
#define UI_CARD_RADIUS          12
#define UI_CARD_PAD             8
#define UI_STEP_X               (UI_CARD_W + UI_GAP)
#define UI_STEP_Y               (UI_CARD_H + UI_GAP)

/* The strip the rows of a list screen fill. Six of them fill it exactly, which
   is what the link and the detail screen need; the settings and the WiFi screen
   hold more and their strip scrolls for the rest. */
#define UI_GRID_W               (2 * UI_CARD_W + UI_GAP)
#define UI_ROW_COUNT            7
#define UI_ROW_H                42
#define UI_ROW_RADIUS           12
#define UI_ROW_GAP              UI_GAP
#define UI_ROW_STEP             (UI_ROW_H + UI_ROW_GAP)

/* How much of the panel the scrolling strip covers. The row that leaves a list
   screen is pinned under the strip and not inside it, so the way out can never
   be scrolled out of sight; what is left below that row is the bottom margin. */
#define UI_LIST_H               (DEV_LCD_V_RES - UI_GRID_Y - UI_ROW_H - UI_ROW_GAP - 7)

/* ------------------------------------------------------------------
 * Colours, one place so the look stays consistent. Light, natural, quiet: the
 * background is a warm grey green, the cards are white and only the small state
 * dot carries a signal colour unless a reading is out of its band.
 * ------------------------------------------------------------------ */
#define UI_COL_BG               lv_color_hex(0xF4F6F3)
#define UI_COL_CARD             lv_color_hex(0xFFFFFF)
#define UI_COL_LINE             lv_color_hex(0xB8D0E8)   /* card border, a soft blue */
#define UI_COL_FOCUS            lv_color_hex(0x3D7EBF)   /* focus ring, a clear blue */
#define UI_COL_TEXT             lv_color_hex(0x2B2F36)
#define UI_COL_DIM              lv_color_hex(0x7A828E)
#define UI_COL_IDLE             lv_color_hex(0xC7CDC6)   /* dot with nothing to report */
#define UI_COL_OK               lv_color_hex(0x3FAE8C)
#define UI_COL_WARN             lv_color_hex(0xE8A93A)
#define UI_COL_ALARM            lv_color_hex(0xE76F51)
#define UI_COL_SHADOW           lv_color_hex(0x9AA3AD)

/* A soft shadow, not a heavy one. */
#define UI_SHADOW_W             8
#define UI_SHADOW_OPA           LV_OPA_10

/* ------------------------------------------------------------------
 * Fonts. Montserrat 14 is the default font of LVGL and therefore always there,
 * the other three are turned on in sdkconfig.defaults.
 * ------------------------------------------------------------------ */
#define UI_FONT_TINY            (&lv_font_montserrat_12)
#define UI_FONT_BODY            (&lv_font_montserrat_14)
#define UI_FONT_TITLE           (&lv_font_montserrat_16)
#define UI_FONT_MID             (&lv_font_montserrat_20)

/* Width of the focus marker of a card and of a row, and the padding a row of a
   list screen keeps on both sides of its own text. The marker is the very border
   a card and a row always carry, thickened while the knob rests on them, so a
   card never shows a second ring around the one it already has. The padding
   gives back the pixel the wider border takes, so no text ever moves when the
   focus arrives. */
#define UI_FOCUS_LINE           2
#define UI_ROW_PAD_H            10

/* ------------------------------------------------------------------
 * Screens, levels, requests and actions.
 * ------------------------------------------------------------------ */
#define UI_SCREEN_COUNT         6U
#define UI_SCREEN_MAIN          0U
#define UI_SCREEN_SET           1U
#define UI_SCREEN_LINK          2U
#define UI_SCREEN_DET           3U
#define UI_SCREEN_WIFI          4U
#define UI_SCREEN_KEY           5U

/* State of one reading, in this order from quiet to loud. */
#define UI_LEVEL_UNKNOWN        0U
#define UI_LEVEL_OK             1U
#define UI_LEVEL_WARN           2U
#define UI_LEVEL_ALARM          3U

/* Left behind by the interface and served by the main loop. */
#define UI_REQ_NONE             0U
#define UI_REQ_QUERY            1U
#define UI_REQ_PERIOD           2U
#define UI_REQ_BEEP             3U
#define UI_REQ_READ             4U
#define UI_REQ_SCAN             5U
#define UI_REQ_JOIN             6U

/* Answers of the main loop, rendered by the tick of the interface. */
#define UI_STATUS_NONE          0U
#define UI_STATUS_QUERY_OK      1U
#define UI_STATUS_QUERY_FAIL    2U
#define UI_STATUS_PERIOD_OK     3U
#define UI_STATUS_PERIOD_FAIL   4U
#define UI_STATUS_BEEP_OK       5U
#define UI_STATUS_BEEP_FAIL     6U

/* What a click asks for, handed over as the context of the event callback, so
   no integer is ever cast back into a pointer. */
#define UI_ACT_SET              1U
#define UI_ACT_MAIN             2U
#define UI_ACT_LINK             3U
#define UI_ACT_QUERY            4U
#define UI_ACT_PERIOD           5U
#define UI_ACT_BEEP             6U
#define UI_ACT_BRIGHT           7U
#define UI_ACT_DET_LOW          8U
#define UI_ACT_DET_HIGH         9U
#define UI_ACT_DET_BACK         10U
#define UI_ACT_WIFI             11U
#define UI_ACT_KEY_TURN         12U
#define UI_ACT_KEY_SET          13U
#define UI_ACT_KEY_DELETE       14U
#define UI_ACT_KEY_CONNECT      15U

/* The networks of the WiFi screen carry their slot number instead of an action
   code, the way the six cards do. The range sits above every other code so the
   handler can recognise it before it looks at the cards. */
#define UI_ACT_NET_BASE         0x40U

/* The six cards of the main screen carry their own index in the context instead
   of an action code, so one handler serves them and the rows of the list screens
   alike. */
#define UI_ACT_CARD_BASE        0x20U

/* ------------------------------------------------------------------
 * The six cards of the main screen.
 *
 * One table drives the whole screen: the caption, the unit, the item of the link
 * the value comes from, and the two thresholds the reading is judged against.
 *
 * A threshold is a plain number the knob can walk, from the first to the last
 * value of its range, one step per detent. It lives here only, it is never sent
 * to the node, and it lives in RAM: a reboot brings the defaults back.
 *
 * A reading under the low threshold or over the high one warns; it turns into an
 * alarm once it is margin past it. That leaves one number per side on the screen
 * while the colour still says how bad it got.
 *
 * Values are scaled by 100 exactly like the values on the link, so 3000 is 30.00.
 * A card whose item is 0 has no source on the link yet and shows "NO DATA".
 * ------------------------------------------------------------------ */
#define UI_CARD_COUNT           6U
#define UI_CARD_SYSTEM          5U

/* One threshold: the range the knob walks and what it is set to. */
typedef struct
{
    int32_t min;            /* first value of the range                          */
    int32_t max;            /* last value of the range                           */
    int32_t step;           /* one detent of the knob                            */
    int32_t value;          /* what it is set to, scaled by 100 like a reading   */
} ui_limit_t;

typedef struct
{
    const char *caption;    /* headline of the card                              */
    const char *unit;       /* unit behind the value                             */
    uint8_t     id;         /* LINK_ID_xxx, 0 when the link carries no such item */
    uint8_t     decimals;   /* digits behind the decimal point                   */
    int32_t     margin;     /* how far past a threshold is still a warning       */
    ui_limit_t  low;        /* threshold of the low side                         */
    ui_limit_t  high;       /* threshold of the high side                        */
} ui_card_t;

/* The five readings keep a band, the sixth card is only the way into the
   settings. A card the node has no sensor for carries a band all the same, so it
   is ready the day that item starts to arrive.

   Every threshold can be turned across the whole span of the sensor behind it, so
   no reading the node can produce is out of reach: -40.0 to 80.0 for temperature,
   0.0 to 100.0 % for humidity and for the soil probe, 0 to 5000 ppm for CO2 and
   0 to 100000 lx for the light. The step is a whole number of the unit as the card
   shows it, and every default below sits on that step. */
static ui_card_t s_cards[UI_CARD_COUNT] =
{
    /* caption   unit           id                dec  margin
       low  { min, max, step, value }        high { min, max, step, value } */
    { "TEMP",  "\xC2\xB0" "C", LINK_ID_TEMP,  1U,   500,
      { -4000,   4000,   100,   1000 }, {  -4000,   8000,   100,   3000 } },
    { "HUMI",  "%",            LINK_ID_HUMI,  1U,  1000,
      {     0,  10000,   100,   3000 }, {      0,  10000,   100,   8000 } },
    { "CO2",   "ppm",          LINK_ID_CO2,   0U, 50000,
      {     0, 500000, 10000,  40000 }, {      0, 500000, 10000, 120000 } },
    { "SOIL",  "%",            LINK_ID_SOIL,  1U,  1000,
      {     0,  10000,   100,   2500 }, {      0,  10000,   100,   8000 } },
    { "LIGHT", "lx",           LINK_ID_LIGHT, 0U, 20000,
      {     0, 1000000, 10000,  20000 }, {      0, 10000000, 100000, 200000 } },
    { "SYSTEM", "",            0U,            0U,     0,
      {     0,      0,     0,      0 }, {      0,      0,     0,      0 } }
};

/* ------------------------------------------------------------------
 * Rows of the settings screen and of the link screen.
 * ------------------------------------------------------------------ */
#define UI_SET_BRIGHT           0U
#define UI_SET_PERIOD           1U
#define UI_SET_BEEP             2U
#define UI_SET_QUERY            3U
#define UI_SET_WIFI             4U
#define UI_SET_LINK             5U

#define UI_LINK_FRAME_OK        0U
#define UI_LINK_FRAME_BAD       1U
#define UI_LINK_TX_OK           2U
#define UI_LINK_TX_TIMEOUT      3U
#define UI_LINK_RESEND          4U

/* Rows of the detail screen of one reading: what the card is worth right now,
   the extremes it reached since the gateway came up, and the two thresholds the
   knob can turn. */
#define UI_DET_LIVE             0U
#define UI_DET_MIN              1U
#define UI_DET_MAX              2U
#define UI_DET_LOW              3U
#define UI_DET_HIGH             4U

/* Rows of the WiFi screen: a header that only reports what the radio is doing,
   then the slots the sweep fills. The header takes no focus, and a slot with no
   network behind it is hidden; LVGL does not focus a hidden object, so the knob
   only ever walks networks that are really there and the focus group is built
   once and never touched again. */
#define UI_WIFI_HEADER_ROW      0U
#define UI_WIFI_AP_FIRST        1U
#define UI_WIFI_SLOTS           10U
#define UI_WIFI_ROW_COUNT       (UI_WIFI_AP_FIRST + UI_WIFI_SLOTS)

/* Room the name of an access point gets; the rest of the row is the signal. */
#define UI_WIFI_NAME_W          145

/* Rows of the password screen. */
#define UI_KEY_NETWORK          0U
#define UI_KEY_PASSWORD         1U
#define UI_KEY_LETTER           2U
#define UI_KEY_SET              3U
#define UI_KEY_DELETE           4U
#define UI_KEY_CONNECT          5U

/* The character wheel. Room for the window around the cursor, and the room its
   row leaves for it once the caption is drawn. */
#define UI_WHEEL_LEN            48U
#define UI_WHEEL_W              160

/* How many steps either side of the cursor the window shows. */
#define UI_WHEEL_SPAN           3

/* One passphrase character set per entry, plus the names the Set row shows. */
#define UI_KEYSET_COUNT         4U

/* Backlight steps the knob cycles through, the first entry is what
   dev_lcd_init() leaves behind. */
#define UI_BRIGHT_COUNT         5U

static const uint8_t s_bright[UI_BRIGHT_COUNT] = { 100U, 80U, 60U, 40U, 20U };

/* Report periods the knob cycles through, the node takes 1000 .. 60000 ms. */
#define UI_PERIOD_COUNT         4U

static const uint32_t s_periods[UI_PERIOD_COUNT] = { 2000U, 5000U, 10000U, 30000U };

/* Titles of the screens, the first slot of the status bar on each. The detail
   screen barely uses its own: it names the card it opens instead. */
static const char *s_titles[UI_SCREEN_COUNT] =
{
    "ENV MONITOR", "SETTINGS", "LINK", "DETAIL", "WIFI", "PASSWORD"
};

/* The characters the wheel walks through. Lower case and digits come first
   because that is what most passphrases are made of. Each line carries one
   extra slot at its end, OK, and that slot is how the knob gets back out of the
   wheel and on to the other rows of the screen. */
static const char *s_keysets[UI_KEYSET_COUNT] =
{
    "abcdefghijklmnopqrstuvwxyz",
    "0123456789-_.",
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ",
    "!@#$%^&*()+=:;,.?/"
};

static const char *s_keyset_names[UI_KEYSET_COUNT] =
{
    "abc", "123", "ABC", "#$%"
};

/* Context of the ten network rows, indexed by slot. */
static const uint8_t s_net_ctx[UI_WIFI_SLOTS] =
{
    (uint8_t)(UI_ACT_NET_BASE + 0U), (uint8_t)(UI_ACT_NET_BASE + 1U),
    (uint8_t)(UI_ACT_NET_BASE + 2U), (uint8_t)(UI_ACT_NET_BASE + 3U),
    (uint8_t)(UI_ACT_NET_BASE + 4U), (uint8_t)(UI_ACT_NET_BASE + 5U),
    (uint8_t)(UI_ACT_NET_BASE + 6U), (uint8_t)(UI_ACT_NET_BASE + 7U),
    (uint8_t)(UI_ACT_NET_BASE + 8U), (uint8_t)(UI_ACT_NET_BASE + 9U)
};

/* ------------------------------------------------------------------
 * State.
 * ------------------------------------------------------------------ */

/* The status bar of one screen: the title on the left, the state of the WiFi
   link on the right. */
typedef struct
{
    lv_obj_t *title;
    lv_obj_t *dot;
    lv_obj_t *wifi;
} ui_bar_t;

static lv_display_t *s_disp;
static lv_indev_t   *s_indev;

static lv_obj_t *s_scr[UI_SCREEN_COUNT];
static lv_group_t *s_group[UI_SCREEN_COUNT];
static ui_bar_t    s_bar[UI_SCREEN_COUNT];

/* The object the knob lands on when a screen comes up. */
static lv_obj_t *s_first[UI_SCREEN_COUNT];

/* One set of pointers per card, indexed by UI_CARD_xxx. */
static lv_obj_t *s_card_value[UI_CARD_COUNT];
static lv_obj_t *s_card_state[UI_CARD_COUNT];
static lv_obj_t *s_card_dot[UI_CARD_COUNT];

/* Extremes each reading reached since the gateway came up, and whether it ever
   answered at all. Nothing here is kept across a reboot: it describes one run of
   the gateway, not the history of the node. */
static int32_t s_card_min[UI_CARD_COUNT];
static int32_t s_card_max[UI_CARD_COUNT];
static uint8_t s_card_seen[UI_CARD_COUNT];

/* The sixth card is the way into the settings, and the only place the clock of
   the gateway shows: the date above the time. */
static lv_obj_t *s_sys_date;
static lv_obj_t *s_sys_time;
static lv_obj_t *s_sys_dot;

/* Right hand labels of the two list screens. */
static lv_obj_t *s_set_value[UI_ROW_COUNT];
static lv_obj_t *s_link_value[UI_ROW_COUNT];

/* Right hand labels of the detail screen, indexed by UI_DET_xxx. */
static lv_obj_t *s_det_value[UI_ROW_COUNT];

/* The WiFi screen: the row that asks for a sweep with its right hand label, the
   slots the answer fills, and the way back. */
static lv_obj_t     *s_wifi_row[UI_WIFI_SLOTS];
static lv_obj_t     *s_wifi_status;
static lv_obj_t     *s_wifi_name[UI_WIFI_SLOTS];
static lv_obj_t     *s_wifi_signal[UI_WIFI_SLOTS];
static task_net_ap_t s_wifi_ap[UI_WIFI_SLOTS];

/* The password screen: what was picked, what has been typed, and where the
   wheel stands. The wheel is the one place of the interface where a turn does
   not move the focus, so it carries a flag of its own. */
static lv_obj_t     *s_key_net;
static lv_obj_t     *s_key_text;
static lv_obj_t     *s_key_wheel_lbl;
static lv_obj_t     *s_key_set_lbl;
static char          s_key_ssid[TASK_NET_SSID_LEN];
static char          s_key_pass[TASK_NET_PASS_LEN];
static uint8_t       s_key_len;
static uint8_t       s_key_set;
static uint8_t       s_key_cursor;
static uint8_t       s_key_wheel;       /* 1 while the knob turns the wheel */
static int32_t       s_key_steps;       /* turns latched, applied by the tick */

/* Written by the LVGL task and read by the main loop, one byte either way. */
static volatile uint8_t s_request;
static volatile uint8_t s_status;
static volatile uint8_t s_status_seq;

/* Written by whoever owns the network of the gateway and read by the LVGL
   task, one byte either way. The bars refresh on their own, so nothing has
   to be signalled when this moves. */
static volatile uint8_t s_net_state;

/* Raised by the main loop once a setting it owns has moved. The label of that
   setting is then written by the LVGL task, never by the main loop. */
static volatile uint8_t s_set_dirty;

/* Written and read by the LVGL task only. */
static uint8_t  s_screen;
static uint8_t  s_click_pending;
static uint8_t  s_back_pending;
static uint8_t  s_shown_status;
static uint8_t  s_shown_seq;
static uint8_t  s_period_index;
static uint8_t  s_read_id;
static uint8_t  s_bright_index;
static uint8_t  s_buzzer_on;
static uint8_t  s_worst_level;
static uint32_t s_last_refresh;
static uint32_t s_status_until;
static uint8_t  s_wifi_count;       /* slots the WiFi screen already carries */
static uint8_t  s_wifi_seen;        /* state of the sweep last acted upon   */

/* The detail screen: which card it shows, and the threshold the knob is turning.
   A detent is latched by the input callback and applied by the tick, exactly
   like a click, so the value moves without the focus ring ever moving. */
static uint8_t  s_detail_card;
static uint8_t  s_edit;
static uint8_t  s_edit_side;
static uint8_t  s_edit_click;
static uint8_t  s_edit_back;
static int32_t  s_edit_steps;

/* Handed to the click handler as its context. */
static const uint8_t s_act_main   = UI_ACT_MAIN;
static const uint8_t s_act_link   = UI_ACT_LINK;
static const uint8_t s_act_query  = UI_ACT_QUERY;
static const uint8_t s_act_period = UI_ACT_PERIOD;
static const uint8_t s_act_beep   = UI_ACT_BEEP;
static const uint8_t s_act_bright = UI_ACT_BRIGHT;

/* The three rows of the detail screen the knob can act on. */
static const uint8_t s_act_det_low  = UI_ACT_DET_LOW;
static const uint8_t s_act_det_high = UI_ACT_DET_HIGH;
static const uint8_t s_act_det_back = UI_ACT_DET_BACK;

/* The WiFi screen, reached from the settings screen. */
static const uint8_t s_act_wifi     = UI_ACT_WIFI;

/* The password screen. */
static const uint8_t s_act_key_turn    = UI_ACT_KEY_TURN;
static const uint8_t s_act_key_set     = UI_ACT_KEY_SET;
static const uint8_t s_act_key_delete  = UI_ACT_KEY_DELETE;
static const uint8_t s_act_key_connect = UI_ACT_KEY_CONNECT;

/* Context of the six cards, indexed by UI_CARD_xxx: the first five ask the node for
   their own reading, the sixth opens the settings. */
static const uint8_t s_card_ctx[UI_CARD_COUNT] =
{
    (uint8_t)(UI_ACT_CARD_BASE + 0U), (uint8_t)(UI_ACT_CARD_BASE + 1U),
    (uint8_t)(UI_ACT_CARD_BASE + 2U), (uint8_t)(UI_ACT_CARD_BASE + 3U),
    (uint8_t)(UI_ACT_CARD_BASE + 4U), (uint8_t)(UI_ACT_CARD_BASE + 5U)
};

/* ------------------------------------------------------------------
 * Forward declarations: the builders use the handler and the helpers use the
 * builders, so the whole set is announced here.
 * ------------------------------------------------------------------ */
static void Display_OnAction(lv_event_t *event);
static void Display_BuildMain(void);
static void Display_BuildSettings(void);
static void Display_BuildLink(void);
static void Display_BuildDetail(void);
static void Display_BuildWifi(void);
static void Display_BuildKey(void);
static void Display_ShowScreen(uint8_t screen);
static void Display_Refresh(void);
static void Display_UpdateSettings(void);
static void Display_UpdateStatus(void);
static void Display_UpdateDetail(void);
static void Display_UpdateWifi(void);
static void Display_KeyPrepare(uint8_t slot);
static void Display_KeyRefresh(void);
static void Display_KeyTick(void);
static void Display_KeyClick(void);
static void Display_KeyDelete(void);
static void Display_BeginEdit(uint8_t side);
static void Display_EndEdit(void);
static void Display_EditTick(void);
/* ------------------------------------------------------------------
 * Small helpers.
 * ------------------------------------------------------------------ */

/**
 * @brief  Write a label only when its text really changed.
 * @note   Every change turns into traffic on the panel bus, so the comparison is
 *         worth more than it looks: the refresh runs five times a second.
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
 * @brief  Turn a value of the link into the text of its card.
 * @note   The wire carries values scaled by 100, so 2340 is 23.40. A card with
 *         one decimal therefore shows 23.4 and a card with none shows 23.
 * @param  out:      destination buffer.
 * @param  len:      size of that buffer.
 * @param  scaled:   value as it came off the link.
 * @param  decimals: digits behind the decimal point, 0 or 1.
 */
static void Display_FormatValue(char *out, size_t len, int32_t scaled, uint8_t decimals)
{
    int32_t rounded;
    int32_t whole;
    int32_t frac;

    if (decimals == 0U)
    {
        rounded = (scaled >= 0) ? ((scaled + 50) / 100) : ((scaled - 50) / 100);
        lv_snprintf(out, len, "%d", (int)rounded);
        return;
    }
    rounded = (scaled >= 0) ? ((scaled + 5) / 10) : ((scaled - 5) / 10);
    whole = rounded / 10;
    frac = rounded % 10;
    if (frac < 0)
    {
        frac = -frac;
    }
    if ((scaled < 0) && (whole == 0) && (frac != 0))
    {
        /* -0.4 must not lose its sign to the integer division above. */
        lv_snprintf(out, len, "-%d.%d", (int)whole, (int)frac);
        return;
    }
    lv_snprintf(out, len, "%d.%d", (int)whole, (int)frac);
}

/**
 * @brief  Turn a value of the link into the text of the detail screen.
 * @note   The very number the card shows, with the unit behind it, so a line of
 *         the detail screen can be read on its own.
 * @param  out:    destination buffer.
 * @param  len:    size of that buffer.
 * @param  scaled: value as it came off the link.
 * @param  card:   card the value belongs to.
 */
static void Display_FormatWithUnit(char *out, size_t len, int32_t scaled,
                                   const ui_card_t *card)
{
    char number[UI_TEXT_LEN];

    Display_FormatValue(number, sizeof(number), scaled, card->decimals);
    if (card->unit[0] == '\0')
    {
        lv_snprintf(out, len, "%s", number);
        return;
    }
    lv_snprintf(out, len, "%s %s", number, card->unit);
}

/**
 * @brief  Read the wall clock of the gateway.
 * @note   The chip has no battery backed clock, so the system time only becomes
 *         meaningful once the gateway has a network stack and asks an NTP
 *         server. Until then this reports that there is no time.
 * @param  out: destination of the broken down time.
 * @retval 1 when the clock is set, 0 otherwise.
 */
static uint8_t Display_ReadClock(struct tm *out)
{
    time_t now = time(NULL);
    struct tm broken;

    if ((now == (time_t)-1) || (localtime_r(&now, &broken) == NULL))
    {
        return 0U;
    }
    if ((broken.tm_year + 1900) < 2020)
    {
        return 0U;
    }
    *out = broken;
    return 1U;
}

/**
 * @brief  Clock as HH:MM, or --:-- while the gateway has no time source.
 * @param  out: destination buffer.
 * @param  len: size of that buffer.
 */
static void Display_FormatClock(char *out, size_t len)
{
    struct tm broken;

    if (Display_ReadClock(&broken) == 0U)
    {
        lv_snprintf(out, len, "--:--");
        return;
    }
    lv_snprintf(out, len, "%02d:%02d", (int)broken.tm_hour, (int)broken.tm_min);
}

/** Month names of the date line, three letters each. */
static const char *s_months[12] =
{
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

/**
 * @brief  Date as Mmm DD YYYY, or -- while the gateway has no time source.
 * @note   The day is zero padded so the line is exactly as wide on the first of
 *         a month as on the thirty first, and the card never moves.
 * @param  out: destination buffer.
 * @param  len: size of that buffer.
 */
static void Display_FormatDate(char *out, size_t len)
{
    struct tm broken;
    int32_t month;
    int32_t day;

    if (Display_ReadClock(&broken) == 0U)
    {
        lv_snprintf(out, len, "--");
        return;
    }
    month = broken.tm_mon;
    if ((month < 0) || (month > 11))
    {
        month = 0;
    }
    day = broken.tm_mday;
    if ((day < 1) || (day > 31))
    {
        day = 1;
    }
    lv_snprintf(out, len, "%s %02d %d", s_months[month], (int)day,
                (int)(broken.tm_year + 1900));
}

/**
 * @brief  Colour of one state.
 * @param  level: UI_LEVEL_xxx.
 * @retval the colour of that state.
 */
static lv_color_t Display_LevelColor(uint8_t level)
{
    switch (level)
    {
        case UI_LEVEL_OK:
            return UI_COL_OK;
        case UI_LEVEL_WARN:
            return UI_COL_WARN;
        case UI_LEVEL_ALARM:
            return UI_COL_ALARM;
        default:
            return UI_COL_IDLE;
    }
}

/**
 * @brief  Judge one reading against the band of its card.
 * @param  card:  card the reading belongs to.
 * @param  value: reading, scaled by 100.
 * @param  high:  receives 1 when the upper threshold was crossed, 0 otherwise.
 * @retval UI_LEVEL_xxx.
 * @note   A reading outside the band warns, and the same reading warns louder once
 *         it is the margin of its card past the threshold. One number per side is
 *         all the detail screen has to offer, the colour still says how bad it got.
 */
static uint8_t Display_Judge(const ui_card_t *card, int32_t value, uint8_t *high)
{
    *high = 0U;
    if (value < card->low.value)
    {
        return ((value < (card->low.value - card->margin)) ? UI_LEVEL_ALARM
                                                          : UI_LEVEL_WARN);
    }
    if (value > card->high.value)
    {
        *high = 1U;
        return ((value > (card->high.value + card->margin)) ? UI_LEVEL_ALARM
                                                           : UI_LEVEL_WARN);
    }
    return UI_LEVEL_OK;
}

/**
 * @brief  Text of one state.
 * @param  level: UI_LEVEL_xxx.
 * @param  high:  1 when the upper limit was crossed.
 * @retval the text, never NULL.
 */
static const char *Display_LevelText(uint8_t level, uint8_t high)
{
    if (level == UI_LEVEL_UNKNOWN)
    {
        return "NO DATA";
    }
    if (level == UI_LEVEL_OK)
    {
        return "OK";
    }
    return (high != 0U) ? "HIGH" : "LOW";
}

/**
 * @brief  What the first slot of the status bar of one screen carries.
 * @note   Every screen but the detail one carries its own title. The detail screen
 *         names the card it shows, and says which of its two thresholds the knob
 *         is turning, which is the one place a detent does more than walk the
 *         focus ring.
 * @param  screen: UI_SCREEN_xxx.
 * @retval the text, never NULL.
 */
static const char *Display_BarTitle(uint8_t screen)
{
    if (screen == UI_SCREEN_DET)
    {
        if (s_edit != 0U)
        {
            return (s_edit_side == 0U) ? "EDIT LOW" : "EDIT HIGH";
        }
        return s_cards[s_detail_card].caption;
    }
    if ((screen == UI_SCREEN_KEY) && (s_key_wheel != 0U))
    {
        /* The one place where a turn moves a character and not the focus, so
           the bar says what the knob is doing instead of naming the screen. */
        return "TURN";
    }
    return s_titles[screen];
}

/**
 * @brief  Put that text into the status bar of one screen.
 * @param  screen: UI_SCREEN_xxx.
 */
static void Display_SetBarTitle(uint8_t screen)
{
    lv_label_set_text(s_bar[screen].title, Display_BarTitle(screen));
    lv_obj_set_style_text_color(s_bar[screen].title, UI_COL_TEXT, 0);
}

/**
 * @brief  Apply one step of the fade, called by the animation of the value.
 * @param  obj:   label being faded.
 * @param  value: opacity of this step.
 */
static void Display_AnimOpa(void *obj, int32_t value)
{
    lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)value, 0);
}

/**
 * @brief  Fade a value in when it just changed.
 * @note   One animation at a time per label, a new value restarts it instead of
 *         stacking up.
 * @param  obj: label that carries the value.
 */
static void Display_Pulse(lv_obj_t *obj)
{
    lv_anim_t anim;

    if (obj == NULL)
    {
        return;
    }
    (void)lv_anim_delete(obj, NULL);
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, obj);
    lv_anim_set_exec_cb(&anim, Display_AnimOpa);
    lv_anim_set_values(&anim, LV_OPA_40, LV_OPA_COVER);
    lv_anim_set_duration(&anim, UI_ANIM_MS);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
    (void)lv_anim_start(&anim);
}

/**
 * @brief  Create a container without a background, a border or a padding.
 * @param  parent: object the container belongs to.
 * @retval the container.
 */
static lv_obj_t *Display_CreateBox(lv_obj_t *parent)
{
    lv_obj_t *box = lv_obj_create(parent);

    lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(box, 0, 0);
    lv_obj_set_style_radius(box, 0, 0);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_set_scrollable(box, false);
    return box;
}

/**
 * @brief  Create one state dot.
 * @param  parent: object the dot belongs to.
 * @param  color:  colour of the first frame.
 * @retval the dot.
 */
static lv_obj_t *Display_CreateDot(lv_obj_t *parent, lv_color_t color)
{
    lv_obj_t *dot = lv_obj_create(parent);

    lv_obj_set_size(dot, 6, 6);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, color, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_set_style_pad_all(dot, 0, 0);
    lv_obj_set_scrollable(dot, false);
    return dot;
}

/**
 * @brief  Place a card on the two by three grid.
 * @note   The position comes from the index, so a card can never drift out of
 *         the grid by a wrong constant.
 * @param  card:  card to place.
 * @param  index: UI_CARD_xxx, read as column first.
 */
static void Display_PlaceCard(lv_obj_t *card, uint8_t index)
{
    lv_coord_t x = (lv_coord_t)UI_GRID_X + (lv_coord_t)(index % 2U) * (lv_coord_t)UI_STEP_X;
    lv_coord_t y = (lv_coord_t)UI_GRID_Y + (lv_coord_t)(index / 2U) * (lv_coord_t)UI_STEP_Y;

    lv_obj_set_pos(card, x, y);
}

/**
 * @brief  Give a row the focus marker.
 * @note   The very marker a card wears: the border the row always carries widens
 *         and is painted, and the horizontal padding gives the pixel back, so
 *         neither label slides when the knob arrives. Vertically the content is
 *         centred, so losing one pixel at the top and one at the bottom leaves
 *         its centre where it was.
 * @param  row: row to mark.
 * @param  pad: horizontal padding the row keeps while it is not focused.
 */
static void Display_MarkFocus(lv_obj_t *row, lv_coord_t pad)
{
    lv_obj_set_style_border_width(row, (lv_coord_t)UI_FOCUS_LINE, LV_STATE_FOCUSED);
    lv_obj_set_style_border_color(row, UI_COL_FOCUS, LV_STATE_FOCUSED);
    lv_obj_set_style_pad_hor(row, (lv_coord_t)(pad - (UI_FOCUS_LINE - 1)),
                             LV_STATE_FOCUSED);
    lv_obj_set_style_outline_width(row, 0, LV_STATE_FOCUSED);
}

/**
 * @brief  Create the strip the rows of a list screen live in.
 * @note   The strip scrolls, so a screen may hold more rows than the panel can
 *         show at once. The knob walks the rows and LVGL scrolls the strip by
 *         itself to keep the focused one in sight, which is why nothing here
 *         has to know which row is on screen.
 * @param  scr: the screen the strip belongs to.
 * @retval the strip.
 */
static lv_obj_t *Display_CreateList(lv_obj_t *scr)
{
    lv_obj_t *list = lv_obj_create(scr);

    lv_obj_set_size(list, (lv_coord_t)DEV_LCD_H_RES, (lv_coord_t)UI_LIST_H);
    lv_obj_set_pos(list, 0, (lv_coord_t)UI_GRID_Y);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_radius(list, 0, 0);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    /* A bar that shows itself whenever the strip holds more than the panel can
       show, which is how the interface says there is more below. No elastic and
       no momentum: both only show on a touch panel, and this one is driven by a
       knob that steps one row at a time. */
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_width(list, 3, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(list, LV_RADIUS_CIRCLE, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(list, UI_COL_LINE, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(list, LV_OPA_COVER, LV_PART_SCROLLBAR);
    return list;
}

/**
 * @brief  Place a row inside the strip of a list screen.
 * @param  row:   row to place.
 * @param  index: zero based row number, measured inside the strip.
 */
static void Display_PlaceRow(lv_obj_t *row, uint8_t index)
{
    lv_obj_set_pos(row, (lv_coord_t)UI_GRID_X,
                   (lv_coord_t)index * (lv_coord_t)UI_ROW_STEP);
}

/**
 * @brief  Caption label of a row built by Display_BuildRow().
 * @note   The caption is created first, so it is always the first child of the
 *         row and the right hand value label is the second one. Only the WiFi
 *         screen needs it, to write a name it does not know at build time.
 * @param  row: the row.
 * @retval the caption label.
 */
static lv_obj_t *Display_RowCaption(lv_obj_t *row)
{
    return lv_obj_get_child(row, 0);
}

/**
 * @brief  State of the network link as one short line.
 * @note   These three words are the whole vocabulary of the field: the link
 *         is busy, it works, or it does not. Every status bar reads this one
 *         function, so only it has to change.
 * @retval the text, never NULL.
 */
static const char *Display_WifiText(void)
{
    switch (s_net_state)
    {
        case TASK_DISPLAY_NET_CONNECTING:
            return "WIFI ...";
        case TASK_DISPLAY_NET_OK:
            return "WIFI OK";
        case TASK_DISPLAY_NET_ERR:
            return "WIFI ERR";
        default:
            return "WIFI --";
    }
}

/**
 * @brief  Colour of the dot in the status bar.
 * @note   That dot belongs to the word next to it: it reports whether the
 *         gateway is on the air, not how the readings are doing. The health of
 *         the readings still shows on the cards, each with a dot of its own.
 * @retval the colour of the link.
 */
static lv_color_t Display_NetColor(void)
{
    switch (s_net_state)
    {
        case TASK_DISPLAY_NET_CONNECTING:
            return UI_COL_WARN;
        case TASK_DISPLAY_NET_OK:
            return UI_COL_OK;
        case TASK_DISPLAY_NET_ERR:
            return UI_COL_ALARM;
        default:
            return UI_COL_IDLE;
    }
}

/* ------------------------------------------------------------------
 * The status bar, the same one on all three screens.
 * ------------------------------------------------------------------ */

/**
 * @brief  Create the status bar of one screen.
 * @param  screen: UI_SCREEN_xxx.
 */
static void Display_BuildBar(uint8_t screen)
{
    ui_bar_t *bar = &s_bar[screen];
    lv_obj_t *strip;
    lv_obj_t *right;

    strip = lv_obj_create(s_scr[screen]);
    lv_obj_set_size(strip, (lv_coord_t)DEV_LCD_H_RES, (lv_coord_t)UI_STATUS_H);
    lv_obj_set_pos(strip, 0, 0);
    lv_obj_set_style_radius(strip, 0, 0);
    lv_obj_set_style_bg_color(strip, UI_COL_CARD, 0);
    lv_obj_set_style_bg_opa(strip, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(strip, 1, 0);
    lv_obj_set_style_border_color(strip, UI_COL_LINE, 0);
    lv_obj_set_style_border_side(strip, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_pad_hor(strip, (lv_coord_t)UI_MARGIN_X, 0);
    lv_obj_set_style_pad_ver(strip, 0, 0);
    lv_obj_set_scrollable(strip, false);
    lv_obj_set_flex_flow(strip, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(strip, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    bar->title = lv_label_create(strip);
    Display_SetBarTitle(screen);
    lv_obj_set_style_text_font(bar->title, UI_FONT_TINY, 0);
    lv_obj_set_style_text_letter_space(bar->title, 1, 0);

    right = Display_CreateBox(strip);
    lv_obj_set_size(right, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(right, 6, 0);

    bar->dot = Display_CreateDot(right, UI_COL_IDLE);

    bar->wifi = lv_label_create(right);
    lv_label_set_text(bar->wifi, Display_WifiText());
    lv_obj_set_style_text_font(bar->wifi, UI_FONT_TINY, 0);
    lv_obj_set_style_text_color(bar->wifi, UI_COL_DIM, 0);
}

/**
 * @brief  Bring the status bar of one screen up to date.
 * @param  screen: UI_SCREEN_xxx.
 */
static void Display_UpdateBar(uint8_t screen)
{
    Display_SetText(s_bar[screen].wifi, Display_WifiText());
    lv_obj_set_style_bg_color(s_bar[screen].dot, Display_NetColor(), 0);
}
/* ------------------------------------------------------------------
 * Building blocks of the three screens.
 * ------------------------------------------------------------------ */

/**
 * @brief  Create the root of one screen.
 * @note   No layout is set on a screen: every child is placed on explicit pixels,
 *         which is what makes the grid exactly symmetric.
 * @retval the screen object.
 */
static lv_obj_t *Display_CreateScreen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(scr, UI_COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_set_scrollable(scr, false);
    return scr;
}

/**
 * @brief  Create one card of the main screen.
 * @note   All six cards are built by this one function so they cannot end up
 *         different: same size, same radius, same padding, same three lines.
 *         The sixth one only carries different content.
 * @param  index: UI_CARD_xxx.
 * @retval the card, the main screen keeps the one of the system card.
 */
static lv_obj_t *Display_BuildCard(uint8_t index)
{
    const ui_card_t *desc = &s_cards[index];
    lv_obj_t *card = lv_obj_create(s_scr[UI_SCREEN_MAIN]);
    lv_obj_t *top;
    lv_obj_t *mid;
    lv_obj_t *bottom;
    lv_obj_t *label;

    lv_obj_set_size(card, (lv_coord_t)UI_CARD_W, (lv_coord_t)UI_CARD_H);
    Display_PlaceCard(card, index);
    lv_obj_set_style_radius(card, (lv_coord_t)UI_CARD_RADIUS, 0);
    lv_obj_set_style_bg_color(card, UI_COL_CARD, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, UI_COL_LINE, 0);
    lv_obj_set_style_pad_all(card, (lv_coord_t)UI_CARD_PAD, 0);
    lv_obj_set_style_shadow_width(card, (lv_coord_t)UI_SHADOW_W, 0);
    lv_obj_set_style_shadow_opa(card, (lv_opa_t)UI_SHADOW_OPA, 0);
    lv_obj_set_style_shadow_color(card, UI_COL_SHADOW, 0);
    lv_obj_set_style_shadow_offset_y(card, 1, 0);
    lv_obj_set_scrollable(card, false);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    /* First line: the name of the reading, and its state as a dot. */
    top = Display_CreateBox(card);
    lv_obj_set_width(top, LV_PCT(100));
    lv_obj_set_height(top, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    label = lv_label_create(top);
    lv_label_set_text(label, desc->caption);
    lv_obj_set_style_text_font(label, UI_FONT_BODY, 0);
    lv_obj_set_style_text_color(label, UI_COL_TEXT, 0);
    lv_obj_set_style_text_letter_space(label, 1, 0);

    s_card_dot[index] = Display_CreateDot(top, UI_COL_IDLE);

    /* Second line: the reading, with its unit on the baseline. */
    mid = Display_CreateBox(card);
    lv_obj_set_width(mid, LV_PCT(100));
    lv_obj_set_height(mid, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(mid, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(mid, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
    lv_obj_set_style_pad_column(mid, 3, 0);

    if (index == UI_CARD_SYSTEM)
    {
        s_sys_date = lv_label_create(mid);
        lv_label_set_text(s_sys_date, "--");
        /* One size below the value of the other five cards: "Oct 01 2026" needs
           81.6 px in Montserrat 14 and 93.4 px in 16, and a card only has 93 px
           inside its border and its padding. */
        lv_obj_set_style_text_font(s_sys_date, UI_FONT_BODY, 0);
        lv_obj_set_style_text_color(s_sys_date, UI_COL_TEXT, 0);
    }
    else
    {
        s_card_value[index] = lv_label_create(mid);
        lv_label_set_text(s_card_value[index], "--");
        lv_obj_set_style_text_font(s_card_value[index], UI_FONT_MID, 0);
        lv_obj_set_style_text_color(s_card_value[index], UI_COL_TEXT, 0);

        label = lv_label_create(mid);
        lv_label_set_text(label, desc->unit);
        lv_obj_set_style_text_font(label, UI_FONT_TINY, 0);
        lv_obj_set_style_text_color(label, UI_COL_DIM, 0);
    }

    /* Third line: the state of the reading, quiet until it is not. */
    bottom = Display_CreateBox(card);
    lv_obj_set_width(bottom, LV_PCT(100));
    lv_obj_set_height(bottom, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(bottom, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bottom, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    if (index == UI_CARD_SYSTEM)
    {
        s_sys_time = lv_label_create(bottom);
        lv_label_set_text(s_sys_time, "--:--");
        lv_obj_set_style_text_font(s_sys_time, UI_FONT_TINY, 0);
        lv_obj_set_style_text_color(s_sys_time, UI_COL_DIM, 0);

        label = lv_label_create(bottom);
        lv_label_set_text(label, "SET");
        lv_obj_set_style_text_font(label, UI_FONT_TINY, 0);
        lv_obj_set_style_text_color(label, UI_COL_OK, 0);

        s_sys_dot = s_card_dot[index];
    }
    else
    {
        s_card_state[index] = lv_label_create(bottom);
        lv_label_set_text(s_card_state[index], "NO DATA");
        lv_obj_set_style_text_font(s_card_state[index], UI_FONT_TINY, 0);
        lv_obj_set_style_text_color(s_card_state[index], UI_COL_DIM, 0);
    }

    /* Every card is on the focus ring, so turning the knob walks the six of them
       and a press acts on the one under the cursor: the five readings open their
       own detail screen and ask the node for a fresh value on the way in, the
       sixth opens the settings. */
    lv_obj_set_clickable(card, true);
    (void)lv_obj_add_event_cb(card, Display_OnAction, LV_EVENT_CLICKED,
                              (void *)&s_card_ctx[index]);

    /* The marker is the border the card always carries, widened and painted, with
       the padding handed back so that no line of the card moves when the knob
       arrives. There is no outline: one card, one line. */
    lv_obj_set_style_border_width(card, (lv_coord_t)UI_FOCUS_LINE, LV_STATE_FOCUSED);
    lv_obj_set_style_border_color(card, UI_COL_FOCUS, LV_STATE_FOCUSED);
    lv_obj_set_style_pad_all(card, (lv_coord_t)(UI_CARD_PAD - (UI_FOCUS_LINE - 1)),
                             LV_STATE_FOCUSED);
    lv_obj_set_style_outline_width(card, 0, LV_STATE_FOCUSED);
    lv_group_add_obj(s_group[UI_SCREEN_MAIN], card);

    return card;
}
/**
 * @brief  Create one row of the settings or the link screen.
 * @param  parent:    screen the row belongs to.
 * @param  index:     zero based row number, it decides the pixel it sits on.
 * @param  caption:   text on the left.
 * @param  value_out: receives the right hand label, may be NULL.
 * @param  group:     focus group that should hold the row, NULL keeps it out.
 * @param  action:    context handed to the click handler, NULL for a row that is
 *                    only there to be read.
 * @retval the row.
 */
static lv_obj_t *Display_BuildRow(lv_obj_t *parent, uint8_t index, const char *caption,
                                  lv_obj_t **value_out, lv_group_t *group,
                                  const uint8_t *action)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_t *label;

    lv_obj_set_size(row, (lv_coord_t)UI_GRID_W, (lv_coord_t)UI_ROW_H);
    Display_PlaceRow(row, index);
    lv_obj_set_style_radius(row, (lv_coord_t)UI_ROW_RADIUS, 0);
    lv_obj_set_style_bg_color(row, UI_COL_CARD, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, UI_COL_LINE, 0);
    lv_obj_set_style_pad_hor(row, (lv_coord_t)UI_ROW_PAD_H, 0);
    lv_obj_set_style_pad_ver(row, 0, 0);
    lv_obj_set_style_shadow_width(row, (lv_coord_t)UI_SHADOW_W, 0);
    lv_obj_set_style_shadow_opa(row, (lv_opa_t)UI_SHADOW_OPA, 0);
    lv_obj_set_style_shadow_color(row, UI_COL_SHADOW, 0);
    lv_obj_set_style_shadow_offset_y(row, 1, 0);
    lv_obj_set_scrollable(row, false);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    label = lv_label_create(row);
    lv_label_set_text(label, caption);
    lv_obj_set_style_text_font(label, UI_FONT_BODY, 0);
    lv_obj_set_style_text_color(label, UI_COL_TEXT, 0);

    if (value_out != NULL)
    {
        *value_out = lv_label_create(row);
        lv_label_set_text(*value_out, "");
        lv_obj_set_style_text_font(*value_out, UI_FONT_BODY, 0);
        lv_obj_set_style_text_color(*value_out, UI_COL_DIM, 0);
    }

    if ((group != NULL) && (action != NULL))
    {
        lv_obj_set_clickable(row, true);
        (void)lv_obj_add_event_cb(row, Display_OnAction, LV_EVENT_CLICKED, (void *)action);

        Display_MarkFocus(row, (lv_coord_t)UI_ROW_PAD_H);
        /* LVGL brings a row into sight when it takes the focus, but only if the
           row itself is marked for it: the flag is read from the object that
           gets the focus, never from its parent. Without this the rows below
           the visible part of the strip can never be reached. */
        lv_obj_set_scroll_on_focus(row, true);
        lv_group_add_obj(group, row);
    }

    return row;
}

/**
 * @brief  Build the row that leaves a list screen.
 * @note   It is pinned under the strip instead of being put inside it, so it is
 *         on screen however far the list has been scrolled: a way out must
 *         never scroll away. That is also why the strip is one row shorter than
 *         the panel would allow.
 * @param  scr:    the screen the row belongs to.
 * @param  group:  focus group that should hold the row.
 * @param  action: context handed to the click handler.
 * @retval the row.
 */
static lv_obj_t *Display_BuildBackRow(lv_obj_t *scr, lv_group_t *group,
                                      const uint8_t *action)
{
    lv_obj_t *row = Display_BuildRow(scr, 0U, "Back", NULL, group, action);

    lv_obj_set_pos(row, (lv_coord_t)UI_GRID_X,
                   (lv_coord_t)(UI_GRID_Y + UI_LIST_H + UI_ROW_GAP));
    return row;
}

/**
 * @brief  Main screen: the status bar and the six cards.
 */
static void Display_BuildMain(void)
{
    uint8_t index;

    s_scr[UI_SCREEN_MAIN] = Display_CreateScreen();
    s_group[UI_SCREEN_MAIN] = lv_group_create();
    lv_group_set_wrap(s_group[UI_SCREEN_MAIN], true);
    Display_BuildBar(UI_SCREEN_MAIN);

    for (index = 0U; index < UI_CARD_COUNT; index++)
    {
        lv_obj_t *card = Display_BuildCard(index);

        if (index == 0U)
        {
            s_first[UI_SCREEN_MAIN] = card;
        }
    }
}

/**
 * @brief  Settings screen: what the knob can change, and the way to the counters.
 */
static void Display_BuildSettings(void)
{
    lv_obj_t *list;

    s_scr[UI_SCREEN_SET] = Display_CreateScreen();
    s_group[UI_SCREEN_SET] = lv_group_create();
    lv_group_set_wrap(s_group[UI_SCREEN_SET], true);
    Display_BuildBar(UI_SCREEN_SET);
    list = Display_CreateList(s_scr[UI_SCREEN_SET]);

    s_first[UI_SCREEN_SET] = Display_BuildRow(list, UI_SET_BRIGHT,
                                              "Brightness", &s_set_value[UI_SET_BRIGHT],
                                              s_group[UI_SCREEN_SET], &s_act_bright);
    Display_BuildRow(list, UI_SET_PERIOD, "Report",
                     &s_set_value[UI_SET_PERIOD], s_group[UI_SCREEN_SET], &s_act_period);
    Display_BuildRow(list, UI_SET_BEEP, "Buzzer",
                     &s_set_value[UI_SET_BEEP], s_group[UI_SCREEN_SET], &s_act_beep);
    Display_BuildRow(list, UI_SET_QUERY, "Read now", NULL,
                     s_group[UI_SCREEN_SET], &s_act_query);
    Display_BuildRow(list, UI_SET_WIFI, "WiFi", &s_set_value[UI_SET_WIFI],
                     s_group[UI_SCREEN_SET], &s_act_wifi);
    Display_BuildRow(list, UI_SET_LINK, "Link stats", NULL,
                     s_group[UI_SCREEN_SET], &s_act_link);
    /* Six rows do not fit the strip at once, so it scrolls; the seventh does not
       go in at all, it is pinned below, where it is always on screen. */
    (void)Display_BuildBackRow(s_scr[UI_SCREEN_SET], s_group[UI_SCREEN_SET],
                               &s_act_main);
}

/**
 * @brief  Link screen: the counters of the device layer, read only.
 */
static void Display_BuildLink(void)
{
    lv_obj_t *list;

    s_scr[UI_SCREEN_LINK] = Display_CreateScreen();
    s_group[UI_SCREEN_LINK] = lv_group_create();
    lv_group_set_wrap(s_group[UI_SCREEN_LINK], true);
    Display_BuildBar(UI_SCREEN_LINK);
    list = Display_CreateList(s_scr[UI_SCREEN_LINK]);

    Display_BuildRow(list, UI_LINK_FRAME_OK, "Frames ok",
                     &s_link_value[UI_LINK_FRAME_OK], NULL, NULL);
    Display_BuildRow(list, UI_LINK_FRAME_BAD, "Frames bad",
                     &s_link_value[UI_LINK_FRAME_BAD], NULL, NULL);
    Display_BuildRow(list, UI_LINK_TX_OK, "TX ok",
                     &s_link_value[UI_LINK_TX_OK], NULL, NULL);
    Display_BuildRow(list, UI_LINK_TX_TIMEOUT, "TX timeout",
                     &s_link_value[UI_LINK_TX_TIMEOUT], NULL, NULL);
    Display_BuildRow(list, UI_LINK_RESEND, "Resends",
                     &s_link_value[UI_LINK_RESEND], NULL, NULL);
    s_first[UI_SCREEN_LINK] = Display_BuildBackRow(s_scr[UI_SCREEN_LINK],
                                                   s_group[UI_SCREEN_LINK],
                                                   &s_act_main);
}

/**
 * @brief  Detail screen: one reading, its extremes and its two thresholds.
 * @note   The three lines the knob cannot change are built first and are only ever
 *         written by Display_UpdateDetail(). The two below them are the ones the
 *         knob acts on: a press on one starts turning it instead of walking the
 *         focus ring, and the way out is the same long press that leaves every
 *         other screen.
 */
static void Display_BuildDetail(void)
{
    lv_obj_t *list;

    s_scr[UI_SCREEN_DET] = Display_CreateScreen();
    s_group[UI_SCREEN_DET] = lv_group_create();
    lv_group_set_wrap(s_group[UI_SCREEN_DET], true);
    Display_BuildBar(UI_SCREEN_DET);
    list = Display_CreateList(s_scr[UI_SCREEN_DET]);

    Display_BuildRow(list, UI_DET_LIVE, "Live",
                     &s_det_value[UI_DET_LIVE], NULL, NULL);
    Display_BuildRow(list, UI_DET_MIN, "Min",
                     &s_det_value[UI_DET_MIN], NULL, NULL);
    Display_BuildRow(list, UI_DET_MAX, "Max",
                     &s_det_value[UI_DET_MAX], NULL, NULL);

    /* A reading is written in the colour of the text, a threshold is dim because
       it is the one the knob can move. */
    lv_obj_set_style_text_color(s_det_value[UI_DET_LIVE], UI_COL_TEXT, 0);
    lv_obj_set_style_text_color(s_det_value[UI_DET_MIN], UI_COL_TEXT, 0);
    lv_obj_set_style_text_color(s_det_value[UI_DET_MAX], UI_COL_TEXT, 0);

    s_first[UI_SCREEN_DET] = Display_BuildRow(list, UI_DET_LOW,
                                             "Low limit", &s_det_value[UI_DET_LOW],
                                             s_group[UI_SCREEN_DET], &s_act_det_low);
    Display_BuildRow(list, UI_DET_HIGH, "High limit",
                     &s_det_value[UI_DET_HIGH], s_group[UI_SCREEN_DET],
                     &s_act_det_high);
    (void)Display_BuildBackRow(s_scr[UI_SCREEN_DET], s_group[UI_SCREEN_DET],
                               &s_act_det_back);
}

/**
 * @brief  WiFi screen: what the radio can see around the gateway.
 * @note   The sweep starts on the way in, like the one of a phone, so there is
 *         no button for it; the header at the top only reports what the radio is
 *         doing and takes no focus. The slots under it are filled by the answer,
 *         one that carries nothing stays hidden, and since LVGL does not focus a
 *         hidden object the knob walks the networks that are really there and
 *         nothing else.
 * @note   The name of an access point is whatever its owner typed, and plenty of
 *         them carry Chinese. The built in Montserrat fonts have no CJK glyph at
 *         all, so those names would come out blank; the row is drawn with the
 *         Source Han Sans subset that is compiled in for exactly this.
 */
static void Display_BuildWifi(void)
{
    lv_obj_t *list;
    lv_obj_t *row;
    uint8_t   i;

    s_scr[UI_SCREEN_WIFI] = Display_CreateScreen();
    s_group[UI_SCREEN_WIFI] = lv_group_create();
    lv_group_set_wrap(s_group[UI_SCREEN_WIFI], true);
    Display_BuildBar(UI_SCREEN_WIFI);
    list = Display_CreateList(s_scr[UI_SCREEN_WIFI]);

    (void)Display_BuildRow(list, UI_WIFI_HEADER_ROW, "WiFi", &s_wifi_status,
                           NULL, NULL);

    for (i = 0U; i < (uint8_t)UI_WIFI_SLOTS; i++)
    {
        /* A slot carries its own number, so a press on it opens the password
           screen for that network. */
        row = Display_BuildRow(list, (uint8_t)(UI_WIFI_AP_FIRST + i), "--",
                               &s_wifi_signal[i], s_group[UI_SCREEN_WIFI],
                               &s_net_ctx[i]);
        s_wifi_row[i] = row;
        s_wifi_name[i] = Display_RowCaption(row);
        /* The driver allows 32 characters of name and the row has room for about
           ten of them, so the rest is cut off with a dot. */
        lv_label_set_long_mode(s_wifi_name[i], LV_LABEL_LONG_DOT);
        lv_obj_set_width(s_wifi_name[i], (lv_coord_t)UI_WIFI_NAME_W);
        lv_obj_set_style_text_font(s_wifi_name[i],
                                   &lv_font_source_han_sans_sc_14_cjk, 0);
        lv_obj_set_hidden(row, true);
    }

    /* Focus starts on the way out: on the way in the list is still empty, so
       that is the only stop that is really there. One turn reaches the first
       network as soon as the sweep lands. */
    s_first[UI_SCREEN_WIFI] = Display_BuildBackRow(s_scr[UI_SCREEN_WIFI],
                                                   s_group[UI_SCREEN_WIFI],
                                                   &s_act_main);
}
/* ------------------------------------------------------------------
 * Behaviour of the interface.
 * ------------------------------------------------------------------ */

/**
 * @brief  Step the backlight to the next level.
 * @note   Purely local: the LEDC duty is set and nothing waits for an answer, so
 *         this runs right here instead of being handed to the main loop, and the
 *         line that reports the new level is written right here as well.
 */
static void Display_NextBrightness(void)
{
    s_bright_index = (uint8_t)((s_bright_index + 1U) % UI_BRIGHT_COUNT);
    (void)dev_lcd_backlight(s_bright[s_bright_index]);
    LOGI("backlight -> %u%%", (unsigned)s_bright[s_bright_index]);
    Display_UpdateSettings();
}

/**
 * @brief  Note what a click asks for.
 * @note   A transaction is never run here. The interface only sets the request,
 *         the main loop serves it, so a transaction can never stall the LVGL task.
 *         Opening a screen, turning the backlight and starting to turn a
 *         threshold are local and happen right here.
 * @param  event: the click event of the card or the row.
 */
static void Display_OnAction(lv_event_t *event)
{
    const uint8_t *action = (const uint8_t *)lv_event_get_user_data(event);
    uint8_t code;

    if (action == NULL)
    {
        return;
    }
    code = *action;
    if (code >= UI_ACT_NET_BASE)
    {
        /* A network of the WiFi screen: open its password screen. */
        Display_KeyPrepare((uint8_t)(code - UI_ACT_NET_BASE));
        Display_ShowScreen(UI_SCREEN_KEY);
        return;
    }
    if (code >= UI_ACT_CARD_BASE)
    {
        /* A card of the main screen. */
        uint8_t card = (uint8_t)(code - UI_ACT_CARD_BASE);

        if (card == UI_CARD_SYSTEM)
        {
            Display_ShowScreen(UI_SCREEN_SET);
        }
        else if (s_cards[card].id != 0U)
        {
            /* The detail screen of that one card, and a fresh value asked for on
               the way in so the screen and the card behind it agree. */
            s_detail_card = card;
            s_read_id = s_cards[card].id;
            s_request = UI_REQ_READ;
            Display_ShowScreen(UI_SCREEN_DET);
            Display_UpdateDetail();
        }
        return;
    }
    switch (code)
    {
        case UI_ACT_SET:
            Display_ShowScreen(UI_SCREEN_SET);
            break;
        case UI_ACT_MAIN:
            Display_ShowScreen(UI_SCREEN_MAIN);
            break;
        case UI_ACT_LINK:
            Display_ShowScreen(UI_SCREEN_LINK);
            break;
        case UI_ACT_DET_LOW:
            Display_BeginEdit(0U);
            break;
        case UI_ACT_DET_HIGH:
            Display_BeginEdit(1U);
            break;
        case UI_ACT_DET_BACK:
            Display_ShowScreen(UI_SCREEN_MAIN);
            break;
        case UI_ACT_QUERY:
            s_request = UI_REQ_QUERY;
            break;
        case UI_ACT_PERIOD:
            s_request = UI_REQ_PERIOD;
            break;
        case UI_ACT_BEEP:
            s_request = UI_REQ_BEEP;
            break;
        case UI_ACT_BRIGHT:
            Display_NextBrightness();
            break;
        case UI_ACT_KEY_TURN:
            /* The knob now belongs to the wheel until its closing slot is
               pressed; the title says so. */
            s_key_wheel = 1U;
            s_key_steps = 0;
            Display_SetBarTitle(UI_SCREEN_KEY);
            Display_KeyRefresh();
            break;
        case UI_ACT_KEY_SET:
            s_key_set = (uint8_t)((s_key_set + 1U) % (uint8_t)UI_KEYSET_COUNT);
            s_key_cursor = 0U;
            Display_KeyRefresh();
            break;
        case UI_ACT_KEY_DELETE:
            Display_KeyDelete();
            Display_KeyRefresh();
            break;
        case UI_ACT_KEY_CONNECT:
            s_request = UI_REQ_JOIN;
            break;
        case UI_ACT_WIFI:
            /* Open the screen and sweep on the way in, the way a phone does it:
               there is no button for it, a second visit sweeps again. */
            s_request = UI_REQ_SCAN;
            Display_ShowScreen(UI_SCREEN_WIFI);
            break;
        default:
            break;
    }
}

/**
 * @brief  Slide from one screen to the next and hand the knob over.
 * @param  screen: UI_SCREEN_xxx.
 */
static void Display_ShowScreen(uint8_t screen)
{
    if ((screen >= UI_SCREEN_COUNT) || (screen == s_screen))
    {
        return;
    }

    /* A message that owns the title of the screen we leave behind has to go
       first, nobody would ever restore that title afterwards. */
    if (s_shown_status != UI_STATUS_NONE)
    {
        Display_SetBarTitle(s_screen);
        s_shown_status = UI_STATUS_NONE;
    }

    lv_screen_load_anim(s_scr[screen],
                        (screen > s_screen) ? LV_SCREEN_LOAD_ANIM_MOVE_LEFT
                                            : LV_SCREEN_LOAD_ANIM_MOVE_RIGHT,
                        UI_ANIM_MS, 0, false);
    s_screen = screen;

    /* A turn of the knob that was moving a threshold never survives a change of
       screen, and the title of the screen that comes up is written here: it is
       the name of the card on the detail screen and the title of the bar
       everywhere else. */
    s_edit = 0U;
    s_edit_steps = 0;
    s_edit_click = 0U;
    s_edit_back = 0U;
    s_key_wheel = 0U;              /* a turn of the wheel never crosses screens */
    s_key_steps = 0;
    Display_SetBarTitle(screen);

    if (s_indev != NULL)
    {
        lv_indev_set_group(s_indev, s_group[screen]);
    }
    if (s_first[screen] != NULL)
    {
        lv_group_focus_obj(s_first[screen]);
    }
}

/**
 * @brief  One step back on the stack of screens.
 * @note   The main screen is the bottom of the stack, so holding the knob there
 *         does nothing.
 */
static void Display_Back(void)
{
    switch (s_screen)
    {
        case UI_SCREEN_DET:
            Display_ShowScreen(UI_SCREEN_MAIN);
            break;
        case UI_SCREEN_LINK:
            Display_ShowScreen(UI_SCREEN_SET);
            break;
        case UI_SCREEN_WIFI:
            Display_ShowScreen(UI_SCREEN_SET);
            break;
        case UI_SCREEN_KEY:
            Display_ShowScreen(UI_SCREEN_WIFI);
            break;
        case UI_SCREEN_SET:
            Display_ShowScreen(UI_SCREEN_MAIN);
            break;
        default:
            break;
    }
}

/**
 * @brief  Hand the knob a threshold of the card the detail screen shows.
 * @note   From here on a detent moves the value instead of the focus ring, and the
 *         title of the bar says so.
 * @param  side: 0 for the low threshold, 1 for the high one.
 */
static void Display_BeginEdit(uint8_t side)
{
    s_edit = 1U;
    s_edit_side = side;
    s_edit_steps = 0;
    s_edit_click = 0U;
    s_edit_back = 0U;
    Display_SetBarTitle(UI_SCREEN_DET);
    Display_UpdateDetail();
}

/**
 * @brief  Give the knob back to the focus ring.
 */
static void Display_EndEdit(void)
{
    s_edit = 0U;
    s_edit_steps = 0;
    s_edit_click = 0U;
    s_edit_back = 0U;
    if (s_screen == UI_SCREEN_DET)
    {
        Display_SetBarTitle(UI_SCREEN_DET);
        Display_UpdateDetail();
    }
}

/**
 * @brief  Apply what the knob did while it was turning a threshold.
 * @note   The detents are applied in one go, so a fast turn cannot outrun the
 *         redraw. A press ends the turn and keeps the value, a hold ends it and
 *         walks one screen back, which is the way out of every other screen too.
 */
static void Display_EditTick(void)
{
    ui_card_t *card;
    ui_limit_t *limit;
    int32_t value;

    if (s_edit == 0U)
    {
        return;
    }
    if (s_edit_back != 0U)
    {
        s_edit_back = 0U;
        Display_EndEdit();
        Display_Back();
        return;
    }
    if (s_edit_click != 0U)
    {
        s_edit_click = 0U;
        Display_EndEdit();
        return;
    }
    if ((s_edit_steps == 0) || (s_screen != UI_SCREEN_DET))
    {
        return;
    }

    card = &s_cards[s_detail_card];
    limit = (s_edit_side == 0U) ? &card->low : &card->high;
    value = limit->value + ((int32_t)s_edit_steps * limit->step);
    s_edit_steps = 0;

    /* The range of this threshold first, then the pair of them: a band is never
       allowed to turn inside out, so the low threshold stops at the high one and
       the high one at the low. */
    if (value < limit->min)
    {
        value = limit->min;
    }
    if (value > limit->max)
    {
        value = limit->max;
    }
    if (s_edit_side == 0U)
    {
        if (value > card->high.value)
        {
            value = card->high.value;
        }
    }
    else if (value < card->low.value)
    {
        value = card->low.value;
    }

    if (value != limit->value)
    {
        limit->value = value;
        Display_UpdateDetail();
    }
}

/**
 * @brief  Read the knob once, the way LVGL asks for it.
 * @note   The turning is handed over as it comes, the switch is not: LVGL cannot
 *         tell a click from a hold, so the device layer latches the decision and
 *         the tick injects the matching event afterwards.
 * @param  indev: the encoder device, unused.
 * @param  data:  what LVGL has to be told.
 */
static void Display_EncoderRead(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint8_t events;
    int steps;

    (void)indev;
    events = dev_encoder_poll();
    steps = dev_encoder_take_steps();

    if (s_edit != 0U)
    {
        /* A threshold is being turned: the detents belong to the value, the focus
           ring stays where it is, and nothing at all is handed to LVGL. */
        s_edit_steps = s_edit_steps + (int32_t)steps;
        if ((events & DEV_ENC_EV_LONG) != 0U)
        {
            s_edit_back = 1U;
        }
        else if ((events & DEV_ENC_EV_SHORT) != 0U)
        {
            s_edit_click = 1U;
        }
        data->enc_diff = 0;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    if ((s_screen == UI_SCREEN_KEY) && (s_key_wheel != 0U))
    {
        /* The wheel owns the knob: turns move the character and nothing is
           handed to LVGL. */
        s_key_steps = s_key_steps + (int32_t)steps;
        data->enc_diff = 0;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    if ((events & DEV_ENC_EV_LONG) != 0U)
    {
        s_back_pending = 1U;
    }
    else if ((events & DEV_ENC_EV_SHORT) != 0U)
    {
        s_click_pending = 1U;
    }

    data->enc_diff = (int16_t)steps;
    data->state = LV_INDEV_STATE_RELEASED;
}

/**
 * @brief  Activate whatever the knob points at.
 */
static void Display_ActivateFocused(void)
{
    lv_obj_t *focused = lv_group_get_focused(s_group[s_screen]);

    if (focused != NULL)
    {
        (void)lv_obj_send_event(focused, LV_EVENT_CLICKED, NULL);
    }
}

/**
 * @brief  Draw one state into the dot and the state line of a card.
 * @note   A reading that is fine keeps a grey state line and only shows itself in
 *         the colour of the dot. Only a reading out of its band paints the line,
 *         which is what keeps the screen quiet.
 * @param  index: UI_CARD_xxx.
 * @param  level: UI_LEVEL_xxx.
 * @param  high:  1 when the upper limit was crossed.
 */
static void Display_SetCardState(uint8_t index, uint8_t level, uint8_t high)
{
    lv_color_t color = Display_LevelColor(level);

    lv_obj_set_style_bg_color(s_card_dot[index], color, 0);
    Display_SetText(s_card_state[index], Display_LevelText(level, high));
    lv_obj_set_style_text_color(s_card_state[index],
                                (level == UI_LEVEL_OK) ? UI_COL_DIM : color, 0);
}

/**
 * @brief  Draw the card the detail screen shows: value, extremes and thresholds.
 * @note   The live line carries the colour of its state, so the detail screen
 *         answers the same question as the card it came from, and the threshold
 *         the knob is turning is the only line in the focus colour.
 * @note   The extremes belong to the run of the gateway, not to this instant: a
 *         node that went quiet does not erase what it already reported.
 */
static void Display_UpdateDetail(void)
{
    const ui_card_t *card = &s_cards[s_detail_card];
    char text[UI_TEXT_LEN];
    int32_t value = 0;
    uint8_t level = UI_LEVEL_UNKNOWN;
    uint8_t high = 0U;

    if ((Task_Sensor_IsOnline() != 0U) && (Task_Sensor_Get(card->id, &value) != 0U))
    {
        Display_FormatWithUnit(text, sizeof(text), value, card);
        level = Display_Judge(card, value, &high);
        lv_obj_set_style_text_color(s_det_value[UI_DET_LIVE],
                                    (level == UI_LEVEL_OK) ? UI_COL_TEXT
                                                           : Display_LevelColor(level), 0);
    }
    else
    {
        lv_snprintf(text, sizeof(text), "--");
        lv_obj_set_style_text_color(s_det_value[UI_DET_LIVE], UI_COL_DIM, 0);
    }
    (void)high;                 /* the colour already says low or high */
    Display_SetText(s_det_value[UI_DET_LIVE], text);

    if (s_card_seen[s_detail_card] != 0U)
    {
        Display_FormatWithUnit(text, sizeof(text), s_card_min[s_detail_card], card);
        Display_SetText(s_det_value[UI_DET_MIN], text);
        Display_FormatWithUnit(text, sizeof(text), s_card_max[s_detail_card], card);
        Display_SetText(s_det_value[UI_DET_MAX], text);
    }
    else
    {
        Display_SetText(s_det_value[UI_DET_MIN], "--");
        Display_SetText(s_det_value[UI_DET_MAX], "--");
    }

    Display_FormatWithUnit(text, sizeof(text), card->low.value, card);
    Display_SetText(s_det_value[UI_DET_LOW], text);
    lv_obj_set_style_text_color(s_det_value[UI_DET_LOW],
                                ((s_edit != 0U) && (s_edit_side == 0U)) ? UI_COL_FOCUS
                                                                       : UI_COL_DIM, 0);

    Display_FormatWithUnit(text, sizeof(text), card->high.value, card);
    Display_SetText(s_det_value[UI_DET_HIGH], text);
    lv_obj_set_style_text_color(s_det_value[UI_DET_HIGH],
                                ((s_edit != 0U) && (s_edit_side == 1U)) ? UI_COL_FOCUS
                                                                       : UI_COL_DIM, 0);
}

/**
 * @brief  Draw one reading into its card.
 * @param  index:  UI_CARD_xxx, always a card with a reading.
 * @param  online: 1 while the node is reporting.
 * @retval 1 when the card carries a reading, 0 when it has nothing to show.
 * @note   A reading that came in is also what the extremes of the detail screen are
 *         taken from, which is why they are noted here and not there.
 */
static uint8_t Display_UpdateCard(uint8_t index, uint8_t online)
{
    const ui_card_t *desc = &s_cards[index];
    char text[UI_TEXT_LEN];
    int32_t value = 0;
    uint8_t level;
    uint8_t high = 0U;

    /* A node that went quiet makes its last values stale rather than wrong, so
       they are dropped instead of being shown as if they were live. */
    if ((online == 0U) || (desc->id == 0U) || (Task_Sensor_Get(desc->id, &value) == 0U))
    {
        Display_SetText(s_card_value[index], "--");
        Display_SetCardState(index, UI_LEVEL_UNKNOWN, 0U);
        return 0U;
    }

    Display_FormatValue(text, sizeof(text), value, desc->decimals);
    if (strcmp(lv_label_get_text(s_card_value[index]), text) != 0)
    {
        lv_label_set_text(s_card_value[index], text);
        Display_Pulse(s_card_value[index]);
    }

    /* The highest and the lowest the reading ever was, which is what the detail
       screen reports. The first value seen opens both ends of the range. */
    if (s_card_seen[index] == 0U)
    {
        s_card_seen[index] = 1U;
        s_card_min[index] = value;
        s_card_max[index] = value;
    }
    else
    {
        if (value < s_card_min[index])
        {
            s_card_min[index] = value;
        }
        if (value > s_card_max[index])
        {
            s_card_max[index] = value;
        }
    }

    level = Display_Judge(desc, value, &high);
    Display_SetCardState(index, level, high);
    if (level > s_worst_level)
    {
        s_worst_level = level;
    }
    return 1U;
}

/**
 * @brief  Draw the sixth card: the clock of the gateway and the way into settings.
 */
static void Display_UpdateSystemCard(void)
{
    char text[UI_TEXT_LEN];

    Display_FormatDate(text, sizeof(text));
    Display_SetText(s_sys_date, text);
    Display_FormatClock(text, sizeof(text));
    Display_SetText(s_sys_time, text);
    lv_obj_set_style_bg_color(s_sys_dot, Display_LevelColor(s_worst_level), 0);
}

/**
 * @brief  Draw the readings of the node, the state colour and the status bars.
 */
static void Display_Refresh(void)
{
    char text[UI_TEXT_LEN];
    uint8_t online = Task_Sensor_IsOnline();
    uint8_t known = 0U;
    uint8_t index;
    uint8_t screen;

    s_worst_level = UI_LEVEL_UNKNOWN;
    for (index = 0U; index < UI_CARD_SYSTEM; index++)
    {
        known = (uint8_t)(known + Display_UpdateCard(index, online));
    }

    /* One reading of six missing makes the whole screen worth a look, but it must
       not pass for a healthy system: a green dot needs every card to answer. */
    if ((online == 0U) || (known < UI_CARD_SYSTEM) || (s_worst_level == UI_LEVEL_UNKNOWN))
    {
        s_worst_level = UI_LEVEL_UNKNOWN;
    }

    Display_UpdateSystemCard();

    for (screen = 0U; screen < UI_SCREEN_COUNT; screen++)
    {
        Display_UpdateBar(screen);
    }

    /* The row of the settings screen that leads to the scan. It carries the
       state of the radio, which moves on its own, so it is written here and not
       only when a setting the interface owns has moved. The name of a stored
       network would say less: the gateway only ever joins one. */
    Display_SetText(s_set_value[UI_SET_WIFI], Display_WifiText());

    if (s_screen == UI_SCREEN_DET)
    {
        Display_UpdateDetail();
        return;
    }
    if (s_screen == UI_SCREEN_WIFI)
    {
        Display_UpdateWifi();
        return;
    }
    if (s_screen == UI_SCREEN_KEY)
    {
        return;                    /* every label of it is written on change */
    }
    if (s_screen != UI_SCREEN_LINK)
    {
        return;
    }

    /* The counters belong to the device layer, this is the one screen that shows
       them, so they are read here instead of being cached. */
    dev_stm32_stats_t stats;

    dev_stm32_get_stats(&stats);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.frame_ok);
    Display_SetText(s_link_value[UI_LINK_FRAME_OK], text);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.frame_bad);
    Display_SetText(s_link_value[UI_LINK_FRAME_BAD], text);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.tx_ok);
    Display_SetText(s_link_value[UI_LINK_TX_OK], text);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.tx_timeout);
    Display_SetText(s_link_value[UI_LINK_TX_TIMEOUT], text);
    lv_snprintf(text, sizeof(text), "%d", (int)stats.tx_resend);
    Display_SetText(s_link_value[UI_LINK_RESEND], text);
}

/**
 * @brief  Draw the three rows of the settings screen the interface owns itself.
 * @note   They carry what the interface last asked for, not what the node
 *         confirmed, so they are updated from the request side of the state.
 */
static void Display_UpdateSettings(void)
{
    char text[UI_TEXT_LEN];

    lv_snprintf(text, sizeof(text), "%u%%", (unsigned)s_bright[s_bright_index]);
    Display_SetText(s_set_value[UI_SET_BRIGHT], text);

    lv_snprintf(text, sizeof(text), "%u s", (unsigned)(s_periods[s_period_index] / 1000U));
    Display_SetText(s_set_value[UI_SET_PERIOD], text);

    Display_SetText(s_set_value[UI_SET_BEEP], (s_buzzer_on != 0U) ? "ON" : "OFF");
}

/**
 * @brief  What the row that asks for a sweep reports.
 * @param  state: TASK_NET_SCAN_xxx.
 * @retval the text, never NULL.
 */
static const char *Display_ScanText(uint8_t state)
{
    switch (state)
    {
        case TASK_NET_SCAN_RUNNING:
            return "scanning ...";
        case TASK_NET_SCAN_FAILED:
            return "scan failed";
        case TASK_NET_SCAN_DONE:
            return "done";
        default:
            return "waiting";
    }
}

/**
 * @brief  Move what the last sweep found into the slots of the screen.
 * @param  count: networks to show, at most UI_WIFI_SLOTS.
 */
static void Display_WifiFill(uint8_t count)
{
    char text[UI_TEXT_LEN];
    uint8_t i;

    for (i = 0U; i < (uint8_t)UI_WIFI_SLOTS; i++)
    {
        if (i < count)
        {
            Display_SetText(s_wifi_name[i], s_wifi_ap[i].ssid);
            lv_snprintf(text, sizeof(text), "%d dBm", (int)s_wifi_ap[i].rssi);
            Display_SetText(s_wifi_signal[i], text);
            lv_obj_set_hidden(s_wifi_row[i], false);
        }
        else
        {
            lv_obj_set_hidden(s_wifi_row[i], true);
        }
    }
    s_wifi_count = count;
}

/**
 * @brief  Draw the WiFi screen: what the radio is doing and what it found.
 * @note   The answer is copied once per sweep and not once per refresh: the
 *         copy walks the list the last sweep left behind, so repeating it would
 *         only rewrite the same slots.
 */
static void Display_UpdateWifi(void)
{
    uint8_t state = Task_Net_ScanState();
    char text[UI_TEXT_LEN];

    if (state == TASK_NET_SCAN_DONE)
    {
        lv_snprintf(text, sizeof(text), "%u networks", (unsigned)s_wifi_count);
    }
    else
    {
        lv_snprintf(text, sizeof(text), "%s", Display_ScanText(state));
    }
    Display_SetText(s_wifi_status, text);

    if ((state == TASK_NET_SCAN_DONE) && (s_wifi_seen != TASK_NET_SCAN_DONE))
    {
        Display_WifiFill(Task_Net_ScanResults(s_wifi_ap, (uint8_t)UI_WIFI_SLOTS));
    }
    s_wifi_seen = state;
}

/**
 * @brief  Prepare the password screen for one network of the list.
 * @note   The passphrase is kept when the same network is picked again, so a
 *         join that failed on a typo does not have to be typed from the start.
 * @param  slot: index into the list the last sweep left behind.
 */
static void Display_KeyPrepare(uint8_t slot)
{
    if (slot >= s_wifi_count)
    {
        return;
    }
    if (strcmp(s_key_ssid, s_wifi_ap[slot].ssid) != 0)
    {
        s_key_len = 0U;
        s_key_pass[0] = '\0';
        strncpy(s_key_ssid, s_wifi_ap[slot].ssid, sizeof(s_key_ssid) - 1U);
        s_key_ssid[sizeof(s_key_ssid) - 1U] = '\0';
    }
    s_key_set = 0U;
    s_key_cursor = 0U;
    s_key_wheel = 0U;
    s_key_steps = 0;
}

/**
 * @brief  Build the window of the wheel that is shown around the cursor.
 * @note   The line wraps, and its last slot leaves the wheel, so that slot is
 *         never more than one turn away from the first character.
 * @param  out: destination buffer.
 * @param  len: size of that buffer.
 */
static void Display_KeyWheel(char *out, size_t len)
{
    const char *set = s_keysets[s_key_set];
    int32_t     total = (int32_t)strlen(set) + 1;
    size_t      pos = 0U;
    int32_t     i;

    out[0] = '\0';
    for (i = -(int32_t)UI_WHEEL_SPAN; i <= (int32_t)UI_WHEEL_SPAN; i++)
    {
        int32_t at = ((int32_t)s_key_cursor + i) % total;
        int     written;

        while (at < 0)
        {
            at += total;
        }
        if (at == (total - 1))
        {
            written = lv_snprintf(&out[pos], len - pos, "%sOK%s ",
                                  (i == 0) ? "[" : "", (i == 0) ? "]" : "");
        }
        else
        {
            written = lv_snprintf(&out[pos], len - pos, "%s%c%s ",
                                  (i == 0) ? "[" : "", set[at], (i == 0) ? "]" : "");
        }
        if (written < 0)
        {
            break;
        }
        if ((size_t)written >= (len - pos))
        {
            pos = len - 1U;
            break;
        }
        pos += (size_t)written;
    }
}

/**
 * @brief  Write the three labels of the password screen.
 */
static void Display_KeyRefresh(void)
{
    char wheel[UI_WHEEL_LEN];

    Display_SetText(s_key_net, s_key_ssid);
    Display_SetText(s_key_text, (s_key_len > 0U) ? s_key_pass : "(empty)");
    Display_SetText(s_key_set_lbl, s_keyset_names[s_key_set]);
    Display_KeyWheel(wheel, sizeof(wheel));
    Display_SetText(s_key_wheel_lbl, wheel);
}

/**
 * @brief  Add one character to the passphrase.
 * @param  c: the character.
 */
static void Display_KeyAppend(char c)
{
    if ((size_t)(s_key_len + 1U) >= sizeof(s_key_pass))
    {
        LOGW("the passphrase is as long as the field allows");
        return;
    }
    s_key_pass[s_key_len] = c;
    s_key_len++;
    s_key_pass[s_key_len] = '\0';
}

/**
 * @brief  Drop the last character of the passphrase.
 */
static void Display_KeyDelete(void)
{
    if (s_key_len == 0U)
    {
        return;
    }
    s_key_len--;
    s_key_pass[s_key_len] = '\0';
}

/**
 * @brief  Apply the turns the knob made while the wheel owns it.
 * @note   The turns are applied in one go so a fast spin cannot outrun the
 *         redraw, the same way the thresholds of the detail screen are served.
 */
static void Display_KeyTick(void)
{
    int32_t total;
    int32_t at;

    if (s_key_steps == 0)
    {
        return;
    }
    total = (int32_t)strlen(s_keysets[s_key_set]) + 1;
    at = (int32_t)s_key_cursor + s_key_steps;
    while (at < 0)
    {
        at += total;
    }
    s_key_cursor = (uint8_t)(at % total);
    s_key_steps = 0;
    Display_KeyRefresh();
}

/**
 * @brief  Serve a press while the wheel owns the knob.
 * @note   On a character it appends and stays, so a passphrase can be typed one
 *         press after another; on the closing slot it hands the knob back to
 *         the rows.
 */
static void Display_KeyClick(void)
{
    const char *set = s_keysets[s_key_set];
    int32_t     total = (int32_t)strlen(set) + 1;

    if ((int32_t)s_key_cursor == (total - 1))
    {
        s_key_wheel = 0U;
        s_key_steps = 0;
        Display_SetBarTitle(UI_SCREEN_KEY);
    }
    else
    {
        Display_KeyAppend(set[s_key_cursor]);
    }
    Display_KeyRefresh();
}

/**
 * @brief  Password screen: pick a network and type its passphrase.
 * @note   The wheel is the one row of the interface that takes the knob for
 *         itself: while it is on, a turn moves the character and not the focus.
 *         That is the same trade the detail screen makes for a threshold, and it
 *         is why the wheel carries a closing slot: without a way back out, the
 *         knob could never reach Connect.
 */
static void Display_BuildKey(void)
{
    lv_obj_t *list;
    lv_obj_t *row;

    s_scr[UI_SCREEN_KEY] = Display_CreateScreen();
    s_group[UI_SCREEN_KEY] = lv_group_create();
    lv_group_set_wrap(s_group[UI_SCREEN_KEY], true);
    Display_BuildBar(UI_SCREEN_KEY);
    list = Display_CreateList(s_scr[UI_SCREEN_KEY]);

    Display_BuildRow(list, UI_KEY_NETWORK, "Network", &s_key_net, NULL, NULL);
    Display_BuildRow(list, UI_KEY_PASSWORD, "Password", &s_key_text, NULL, NULL);

    row = Display_BuildRow(list, UI_KEY_LETTER, "Letter", &s_key_wheel_lbl,
                           s_group[UI_SCREEN_KEY], &s_act_key_turn);
    /* The wheel is the only value of the interface that is longer than a word,
       so its row is the only one whose label is given a width of its own. */
    lv_obj_set_width(s_key_wheel_lbl, (lv_coord_t)UI_WHEEL_W);
    lv_label_set_long_mode(s_key_wheel_lbl, LV_LABEL_LONG_DOT);
    s_first[UI_SCREEN_KEY] = row;

    Display_BuildRow(list, UI_KEY_SET, "Set", &s_key_set_lbl,
                     s_group[UI_SCREEN_KEY], &s_act_key_set);
    Display_BuildRow(list, UI_KEY_DELETE, "Delete", NULL,
                     s_group[UI_SCREEN_KEY], &s_act_key_delete);
    Display_BuildRow(list, UI_KEY_CONNECT, "Connect", NULL,
                     s_group[UI_SCREEN_KEY], &s_act_key_connect);

    (void)Display_BuildBackRow(s_scr[UI_SCREEN_KEY], s_group[UI_SCREEN_KEY],
                               &s_act_wifi);
    Display_KeyRefresh();
}

/**
 * @brief  Text of one answer of the main loop.
 * @param  code: UI_STATUS_xxx.
 * @retval the text, empty for UI_STATUS_NONE.
 */
static const char *Display_StatusText(uint8_t code)
{
    switch (code)
    {
        case UI_STATUS_QUERY_OK:
            return "READ OK";
        case UI_STATUS_QUERY_FAIL:
            return "READ FAILED";
        case UI_STATUS_PERIOD_OK:
            return "REPORT SET";
        case UI_STATUS_PERIOD_FAIL:
            return "REPORT REJECTED";
        case UI_STATUS_BEEP_OK:
            return "BUZZER SET";
        case UI_STATUS_BEEP_FAIL:
            return "BUZZER REJECTED";
        default:
            return "";
    }
}

/**
 * @brief  Show the answer of the main loop in the title slot, then the title again.
 * @note   The title slot is the one place that is free on every screen and the
 *         one a message cannot collide with, so no toast is laid over the cards.
 */
static void Display_UpdateStatus(void)
{
    lv_obj_t *title = s_bar[s_screen].title;
    uint8_t good;

    if (s_status_seq != s_shown_seq)
    {
        s_shown_seq = s_status_seq;
        s_shown_status = s_status;
        if (s_shown_status == UI_STATUS_NONE)
        {
            Display_SetBarTitle(s_screen);
            return;
        }
        good = ((s_shown_status == UI_STATUS_QUERY_OK) ||
                (s_shown_status == UI_STATUS_PERIOD_OK) ||
                (s_shown_status == UI_STATUS_BEEP_OK)) ? 1U : 0U;
        lv_label_set_text(title, Display_StatusText(s_shown_status));
        lv_obj_set_style_text_color(title, (good != 0U) ? UI_COL_OK : UI_COL_ALARM, 0);
        s_status_until = lv_tick_get() + UI_STATUS_MS;
        return;
    }

    if ((s_shown_status != UI_STATUS_NONE) &&
        ((int32_t)(lv_tick_get() - s_status_until) >= 0))
    {
        s_shown_status = UI_STATUS_NONE;
        Display_SetBarTitle(s_screen);
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

    /* While a threshold is being turned the knob belongs to the value, so that is
       settled first: those detents never reach the focus ring. */
    Display_EditTick();
    Display_KeyTick();

    if (s_back_pending != 0U)
    {
        s_back_pending = 0U;
        if ((s_screen == UI_SCREEN_KEY) && (s_key_wheel != 0U))
        {
            /* Holding the knob in the wheel strikes the last character; the way
               out of the wheel is its closing slot, not the hold. */
            Display_KeyDelete();
            Display_KeyRefresh();
        }
        else
        {
            Display_Back();
        }
    }
    if (s_click_pending != 0U)
    {
        s_click_pending = 0U;
        if ((s_screen == UI_SCREEN_KEY) && (s_key_wheel != 0U))
        {
            Display_KeyClick();
        }
        else
        {
            Display_ActivateFocused();
        }
    }

    /* A setting the main loop moved has to reach its label from the LVGL task, and
       from nowhere else, so the main loop only raises a flag and this writes the
       label. */
    if (s_set_dirty != 0U)
    {
        s_set_dirty = 0U;
        Display_UpdateSettings();
    }

    Display_UpdateStatus();

    now = lv_tick_get();
    if ((uint32_t)(now - s_last_refresh) < UI_REFRESH_MS)
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
 * @brief  Read values off the node without waiting for its next report, and keep
 *         what came back.
 * @note   The device layer hands the answer of a QUERY back to the asker and
 *         never to the event handler, so it is merged into the cache here.
 *         Without that the card would only change on the next report the node
 *         pushes on its own and a press would look like it did nothing.
 * @param  id: item to read, 0 asks for every item the node has.
 */
static void Display_Read(uint8_t id)
{
    dev_item_t items[LINK_QUERY_MAX_ITEMS];
    uint8_t got = 0U;
    uint8_t i;
    esp_err_t err;

    err = Task_Gateway_RequestValues(id, items, (uint8_t)LINK_QUERY_MAX_ITEMS, &got);
    if (err == ESP_OK)
    {
        for (i = 0U; i < got; i++)
        {
            (void)Task_Sensor_Set(items[i].id, items[i].value);
        }
    }
    s_status = (err == ESP_OK) ? UI_STATUS_QUERY_OK : UI_STATUS_QUERY_FAIL;
    s_status_seq++;
    LOGI("display read id %u -> %s, %u item(s)", (unsigned)id, esp_err_to_name(err),
         (unsigned)got);
}

/**
 * @brief  Move the node to the next report period of the list.
 */
static void Display_RunPeriod(void)
{
    esp_err_t err;
    uint8_t next = (uint8_t)((s_period_index + 1U) % UI_PERIOD_COUNT);

    err = Task_Gateway_SetReportPeriod(s_periods[next]);
    if (err == ESP_OK)
    {
        s_period_index = next;
    }
    s_status = (err == ESP_OK) ? UI_STATUS_PERIOD_OK : UI_STATUS_PERIOD_FAIL;
    s_status_seq++;
    s_set_dirty = 1U;              /* the label is written by the LVGL task */
    LOGI("display period -> %u ms: %s", (unsigned)s_periods[next], esp_err_to_name(err));
}

/**
 * @brief  Ask the radio for one sweep of what is around the gateway.
 * @note   The sweeps the interface asked for are served here and not in the LVGL
 *         task, exactly like the CONTROL and QUERY transactions: the radio can
 *         be busy joining an access point, and the answer is only read back by
 *         Task_Net_Poll() on one of the next passes.
 */
static void Display_RunScan(void)
{
    (void)Task_Net_ScanStart();
}

/**
 * @brief  Join the network that was picked, with the passphrase that was typed.
 * @note   A request and not a call from the interface: it writes the settings
 *         area, which is flash, and it rebuilds the driver configuration.
 */
static void Display_RunJoin(void)
{
    if (Task_Net_Join(s_key_ssid, s_key_pass) == 0U)
    {
        LOGW("the join was refused");
    }
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
    s_status = (err == ESP_OK) ? UI_STATUS_BEEP_OK : UI_STATUS_BEEP_FAIL;
    s_status_seq++;
    s_set_dirty = 1U;              /* the label is written by the LVGL task */
    LOGI("display buzzer -> %s: %s", (wanted != 0U) ? "on" : "off", esp_err_to_name(err));
}

/* ------------------------------------------------------------------
 * What the interface is told from outside.
 * ------------------------------------------------------------------ */

/**
 * @brief  Tell the interface what the network link is doing.
 * @param  state: one of TASK_DISPLAY_NET_xxx.
 * @note   One byte, written by whoever owns the link and read by the LVGL
 *         task. The status bars are redrawn on their own refresh, so there
 *         is nothing to signal and nothing to lock: the worst case is one
 *         stale bar for one refresh period.
 */
void Task_Display_SetNet(task_display_net_t state)
{
    s_net_state = (uint8_t)state;
}

/* ------------------------------------------------------------------
 * Bring up and poll.
 * ------------------------------------------------------------------ */

/**
 * @brief  Bring up the panel, the graphics library and the knob.
 * @note   Every step is allowed to fail on its own: without a panel there is
 *         nothing to serve, and without a knob the interface stays readable but
 *         cannot be operated. Neither failure blocks the rest of the boot.
 */
void Task_Display_Init(void)
{
    esp_err_t err;
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_display_cfg_t disp_cfg;

    s_request = UI_REQ_NONE;
    s_status = UI_STATUS_NONE;
    s_status_seq = 0U;
    s_screen = UI_SCREEN_MAIN;
    s_click_pending = 0U;
    s_back_pending = 0U;
    s_shown_status = UI_STATUS_NONE;
    s_shown_seq = 0U;
    s_period_index = 0U;
    s_read_id = 0U;
    s_bright_index = 1U;              /* what dev_lcd_init() leaves behind */
    s_buzzer_on = 0U;
    s_worst_level = UI_LEVEL_UNKNOWN;
    s_last_refresh = 0U;
    s_status_until = 0U;
    s_detail_card = 0U;
    s_edit = 0U;
    s_edit_side = 0U;
    s_edit_click = 0U;
    s_edit_back = 0U;
    s_edit_steps = 0;
    s_set_dirty = 0U;
    s_wifi_count = 0U;
    s_wifi_seen = TASK_NET_SCAN_IDLE;
    s_key_ssid[0] = '\0';
    s_key_pass[0] = '\0';
    s_key_len = 0U;
    s_key_set = 0U;
    s_key_cursor = 0U;
    s_key_wheel = 0U;
    s_key_steps = 0;
    memset(s_card_min, 0, sizeof(s_card_min));
    memset(s_card_max, 0, sizeof(s_card_max));
    memset(s_card_seen, 0, sizeof(s_card_seen));

    err = dev_lcd_init();
    if (err != ESP_OK)
    {
        LOGE("no panel, the interface stays dark: %s", esp_err_to_name(err));
        return;
    }

    port_cfg.task_priority = UI_TASK_PRIORITY;
    port_cfg.task_stack = UI_TASK_STACK;
    port_cfg.task_max_sleep_ms = UI_TASK_SLEEP_MS;
    port_cfg.timer_period_ms = UI_TASK_TICK_MS;
    err = lvgl_port_init(&port_cfg);
    if (err != ESP_OK)
    {
        LOGE("lvgl port: %s", esp_err_to_name(err));
        return;
    }

    memset(&disp_cfg, 0, sizeof(disp_cfg));
    disp_cfg.io_handle = dev_lcd_io();
    disp_cfg.panel_handle = dev_lcd_panel();
    disp_cfg.buffer_size = DEV_LCD_H_RES * UI_BUF_LINES;
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
    Display_BuildMain();
    Display_BuildSettings();
    Display_BuildLink();
    Display_BuildDetail();
    Display_BuildWifi();
    Display_BuildKey();
    lv_screen_load(s_scr[UI_SCREEN_MAIN]);
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
    lv_indev_set_group(s_indev, s_group[UI_SCREEN_MAIN]);
    if (s_first[UI_SCREEN_MAIN] != NULL)
    {
        lv_group_focus_obj(s_first[UI_SCREEN_MAIN]);
    }
    (void)lv_timer_create(Display_Tick, UI_TICK_PERIOD_MS, NULL);
    lvgl_port_unlock();

    LOGI("interface up, %ux%u", (unsigned)DEV_LCD_H_RES, (unsigned)DEV_LCD_V_RES);
}

/**
 * @brief  Serve the display task once, call it every pass of the main loop.
 * @note   Only the request the interface left behind is served here, which is what
 *         keeps a blocking transaction out of the LVGL task.
 * @param  now_ms: current millisecond tick, not needed by this task.
 */
void Task_Display_Poll(uint32_t now_ms)
{
    uint8_t request = s_request;

    (void)now_ms;
    if (request == UI_REQ_NONE)
    {
        return;
    }
    s_request = UI_REQ_NONE;

    switch (request)
    {
        case UI_REQ_QUERY:
            Display_Read(0U);
            break;
        case UI_REQ_READ:
            Display_Read(s_read_id);
            break;
        case UI_REQ_PERIOD:
            Display_RunPeriod();
            break;
        case UI_REQ_BEEP:
            Display_RunBeep();
            break;
        case UI_REQ_SCAN:
            Display_RunScan();
            break;
        case UI_REQ_JOIN:
            Display_RunJoin();
            break;
        default:
            break;
    }
}
