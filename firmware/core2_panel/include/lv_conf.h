/* LVGL 8.4 configuration for the M5Stack Core2 control surface.
 * Trimmed hard: this device shows six words, two travel rails and a fault
 * line. Every feature left enabled below is one the UI actually uses.
 */
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

#define LV_COLOR_DEPTH          16
/* Let M5GFX do the byte swap in writePixels(..., true). Setting this to 1 as
 * well swaps twice: the panel then renders scrambled channels -- near-black
 * fills came out pink and every glyph edge fringed, which reads as a blurry
 * display rather than the colour bug it actually is. */
#define LV_COLOR_16_SWAP        0

/* The full instrument UI deliberately keeps all workspaces resident so STOP,
 * navigation and state refresh never allocate a whole view while the operator
 * is moving a camera.  The old fixed 40 KiB arena was no longer large enough
 * once the standalone device manager and P1..P6 editor were added.  Keep
 * LVGL's deterministic TLSF allocator, but give it a bounded PSRAM-backed pool
 * with enough headroom for label refreshes and the remaining R&D surfaces.
 * M5.begin() initializes PSRAM before lv_init(); main.cpp verifies it while
 * allocating the two display buffers immediately before this pool is made. */
#define LV_MEM_CUSTOM           0
#define LV_MEM_SIZE             (128U * 1024U)
#define LV_MEM_POOL_INCLUDE     <esp_heap_caps.h>
#define LV_MEM_POOL_ALLOC(size) \
    heap_caps_malloc((size), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

#define LV_DISP_DEF_REFR_PERIOD 25
#define LV_INDEV_DEF_READ_PERIOD 25
#define LV_TICK_CUSTOM          1
#define LV_TICK_CUSTOM_INCLUDE  "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

#define LV_USE_PERF_MONITOR     0
#define LV_USE_MEM_MONITOR      0
#define LV_USE_ASSERT_NULL      1
#define LV_USE_ASSERT_MALLOC    1
#define LV_USE_LOG              0

/* Canvas typography uses LVGL's built-in Montserrat family. Odd design sizes
 * are rounded upward in main.cpp (9->10, 11->12, 13->14, 15->16) so the
 * firmware keeps a reproducible font toolchain while preserving hierarchy. */
#define LV_FONT_MONTSERRAT_10   1
#define LV_FONT_MONTSERRAT_12   1
#define LV_FONT_MONTSERRAT_14   1
#define LV_FONT_MONTSERRAT_16   1
#define LV_FONT_MONTSERRAT_18   1
#define LV_FONT_MONTSERRAT_20   1
#define LV_FONT_MONTSERRAT_22   1
#define LV_FONT_MONTSERRAT_24   1
#define LV_FONT_MONTSERRAT_26   1
#define LV_FONT_MONTSERRAT_28   1
#define LV_FONT_MONTSERRAT_30   1
#define LV_FONT_MONTSERRAT_40   1
#define LV_FONT_MONTSERRAT_48   0
#define LV_FONT_DEFAULT         &lv_font_montserrat_14

/* Widgets in use. */
#define LV_USE_LABEL            1
#define LV_USE_BAR              1
#define LV_USE_BTN              1
#define LV_USE_BTNMATRIX        1
#define LV_USE_SWITCH           1
#define LV_USE_SLIDER           1
#define LV_USE_TABVIEW          1
#define LV_USE_ROLLER           1
#define LV_USE_LINE             1
#define LV_USE_OBJ              1

/* Not used -- keep the binary small. */
#define LV_USE_ANIMIMG          0
#define LV_USE_CANVAS           0
#define LV_USE_CHART            0
#define LV_USE_CALENDAR         0
#define LV_USE_COLORWHEEL       0
#define LV_USE_IMGBTN           0
#define LV_USE_KEYBOARD         0
#define LV_USE_LED              0
#define LV_USE_LIST             0
#define LV_USE_METER            0
#define LV_USE_MSGBOX           0
#define LV_USE_SPINBOX          0
#define LV_USE_SPINNER          0
#define LV_USE_TEXTAREA         0
#define LV_USE_TILEVIEW         1
#define LV_USE_WIN              0
#define LV_USE_TABLE            0
#define LV_USE_CHECKBOX         0
#define LV_USE_DROPDOWN         0
#define LV_USE_ARC              1
#define LV_USE_SPAN             0

#define LV_USE_THEME_DEFAULT    1
#define LV_THEME_DEFAULT_DARK   1
#define LV_THEME_DEFAULT_GROW   0
#define LV_USE_THEME_BASIC      0

#define LV_USE_FLEX             1
#define LV_USE_GRID             1

#endif /* LV_CONF_H */
