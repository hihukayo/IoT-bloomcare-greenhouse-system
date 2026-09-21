#ifndef __APP_DEBUG_H
#define __APP_DEBUG_H

/* ------------------------------------------------------------------
 * One switch for every log line of the gateway.
 *
 *   #define APP_DEBUG      -> LOGI / LOGW / LOGE call ESP_LOGx
 *                          -> the three macros expand to nothing: no UART
 *                          output, no format strings in flash, no cost
 *
 * LINK_DEBUG on the STM32F103 side plays exactly the same role.
 *
 * A .c file that logs needs two extra lines:
 *
 *   #include "app_debug.h"
 *   #ifdef APP_DEBUG
 *   static const char *TAG = "LINK";
 *   #endif
 *
 * and then calls LOGI / LOGW / LOGE without repeating the TAG.
 *
 * Note: #ifdef only asks whether the macro is defined, so "comment the
 * line out" is the way to switch it off, do not write "#define APP_DEBUG 0".
 * ------------------------------------------------------------------ */
#define APP_DEBUG

#ifdef APP_DEBUG
#include "esp_log.h"
#define LOGI(...)   ESP_LOGI(TAG, __VA_ARGS__)
#define LOGW(...)   ESP_LOGW(TAG, __VA_ARGS__)
#define LOGE(...)   ESP_LOGE(TAG, __VA_ARGS__)
#else
/* do { } while (0) keeps "LOGI(...);" valid everywhere, even in a single
   statement if / else without braces */
#define LOGI(...)   do { } while (0)
#define LOGW(...)   do { } while (0)
#define LOGE(...)   do { } while (0)
#endif

#endif /* __APP_DEBUG_H */
