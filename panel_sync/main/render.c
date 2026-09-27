// Display Rendering module. See render.h for what this file owns.
// This first version is a straight move of the drawing code out of panel_sync.c:
// same timing, same core, same output. Improvements come as separate commits.

#include <math.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "frames.h"
#include "render.h"
#include "render_limits.h"

#define LED_GPIO          14
#define PANEL_W           8
#define PANEL_H           8
#define LED_COUNT         (PANEL_W * PANEL_H)
#define BRIGHT            8           // 0-255 brightness scale for content (dim())
#define POWER_BUDGET_MA   60          // max estimated LED current per panel; frames above it are scaled down
#define MARKER_US         2000000     // show slot number for 2s after a layout change
#define REFRESH_MS        10          // pause between refreshes

// Render task placement. Core 0 + priority 1 is exactly where app_main's loop ran
// before the move, so behaviour is unchanged. Pinning to core 1 is the next step.
#define RENDER_CORE       0
#define RENDER_PRIO       1
#define RENDER_STACK      4096
#define STATS_EVERY_US    5000000     // print timing/power stats every 5s

// If the picture looks scrambled (every other row reversed), set this to 1
#define MATRIX_SERPENTINE 0

// Colour byte order the LEDs expect. 0 = RGB (Ben's board: red showed as green on GRB).
// Set to 1 if red shows as green on YOUR board.
#define LED_ORDER_GRB     0

_Static_assert(BRIGHT <= BRIGHTNESS_HARD_MAX, "BRIGHT must never exceed the hard ceiling (8)");

static const char *TAG = "render";
static led_strip_handle_t strip;
static render_state_fn get_state;
static int master_fps;
static frame_set_t anim;           // what to play (frames.bin partition or built-in)

// Framebuffer: drawing code writes here in logical (x, y) order. flush() applies the
// power cap + hard ceiling, maps to the physical LED order and pushes it out.
static uint8_t fb[LED_COUNT][3];
static uint32_t power_peak_ua;     // highest pre-limit estimate since last stats print
static int      power_capped;      // frames scaled down since last stats print

/* ---------- pixel helpers ---------- */
static inline uint8_t dim(uint8_t v) { return (uint16_t)v * BRIGHT / 255; }

static void px(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t *p = fb[y * PANEL_W + x];
    p[0] = r; p[1] = g; p[2] = b;
}

static void clear_all(void)
{
    memset(fb, 0, sizeof fb);
}

// The ONLY place pixels reach the LEDs: limit, map, serialise, send.
static void flush(void)
{
    int capped;
    uint32_t est = limit_frame(&fb[0][0], LED_COUNT, POWER_BUDGET_MA * 1000u, &capped);
    if (est > power_peak_ua) power_peak_ua = est;
    power_capped += capped;

    for (int y = 0; y < PANEL_H; y++) {
        for (int x = 0; x < PANEL_W; x++) {
            const uint8_t *p = fb[y * PANEL_W + x];
            int lx = x;
#if MATRIX_SERPENTINE
            if (y & 1) lx = PANEL_W - 1 - x;
#endif
            led_strip_set_pixel(strip, y * PANEL_W + lx, p[0], p[1], p[2]);
        }
    }
    led_strip_refresh(strip);
}

/* ---------- GIF crop ---------- */
// Draw this panel's piece of animation frame idx.
// The wall is a (cols*8) x (rows*8) canvas. The animation is scaled to COVER it (fill + crop),
// centred, and this panel samples the 8x8 block at its slot.
static void draw_gif(int idx, int slot, int cols, int rows)
{
    const float W = (float)(cols * PANEL_W);
    const float H = (float)(rows * PANEL_H);
    const float s = fmaxf(W / anim.w, H / anim.h);         // cover scale
    const float ox = (anim.w * s - W) / 2.0f;              // crop offsets (centre)
    const float oy = (anim.h * s - H) / 2.0f;
    const int col = slot % cols;
    const int row = slot / cols;

    for (int y = 0; y < PANEL_H; y++) {
        for (int x = 0; x < PANEL_W; x++) {
            float cx = col * PANEL_W + x + 0.5f;           // pixel centre on the canvas
            float cy = row * PANEL_H + y + 0.5f;
            int sx = (int)((cx + ox) / s);
            int sy = (int)((cy + oy) / s);
            if (sx < 0) sx = 0;
            if (sx >= anim.w) sx = anim.w - 1;
            if (sy < 0) sy = 0;
            if (sy >= anim.h) sy = anim.h - 1;
            const uint8_t *c = frame_px(&anim, idx, sx, sy);
            px(x, y, dim(c[0]), dim(c[1]), dim(c[2]));
        }
    }
}

/* ---------- slot marker ---------- */
// 3x5 digits, bit 2 = left column
static const uint8_t font3x5[10][5] = {
    {7,5,5,5,7}, {2,6,2,2,7}, {7,1,7,4,7}, {7,1,7,1,7}, {5,5,7,1,1},
    {7,4,7,1,7}, {7,4,7,5,7}, {7,1,1,1,1}, {7,5,7,5,7}, {7,5,7,1,7},
};

// Slot marker: big digit (slot number, starting at 1) inside a coloured frame
static void draw_slot_number(int number)
{
    clear_all();
    uint8_t fr = dim(0), fg = dim(120), fb = dim(255);        // blue frame
    for (int i = 0; i < PANEL_W; i++) {
        px(i, 0, fr, fg, fb);
        px(i, PANEL_H - 1, fr, fg, fb);
        px(0, i, fr, fg, fb);
        px(PANEL_W - 1, i, fr, fg, fb);
    }
    const uint8_t *g = font3x5[number % 10];
    for (int y = 0; y < 5; y++)
        for (int x = 0; x < 3; x++)
            if (g[y] & (4 >> x)) px(3 + x, 1 + y, dim(255), dim(255), dim(255));
}

/* ---------- render task ---------- */
static void render_task(void *arg)
{
    int     shown_slot = -2, shown_cols = 0, shown_rows = 0;
    int64_t marker_until = 0;

    // Timing stats: gap = time between consecutive frames sent to the LEDs,
    // work = time spent drawing + sending one frame.
    int64_t last_send = 0, stats_start = esp_timer_get_time();
    int64_t gap_min = INT64_MAX, gap_max = 0, gap_sum = 0, work_max = 0;
    int     frames = 0;

    while (1) {
        int64_t loop_start = esp_timer_get_time();
        render_state_t st;
        get_state(&st);
        int64_t now = esp_timer_get_time();

        if (!st.synced) {
            // blinking red = no beacon from the Pico yet
            clear_all();
            if ((now / 500000) % 2) px(0, 0, BRIGHT, 0, 0);
        } else if (st.slot < 0) {
            // blinking blue = hearing the Pico, waiting to be given a slot
            clear_all();
            if ((now / 250000) % 2) px(0, 0, 0, 0, BRIGHT);
        } else {
            // Only show the number when THIS panel's slot or the grid shape changes,
            // not every time some other panel blips in and out.
            if (st.slot != shown_slot || st.cols != shown_cols || st.rows != shown_rows) {
                shown_slot = st.slot; shown_cols = st.cols; shown_rows = st.rows;
                marker_until = now + MARKER_US;
                ESP_LOGI(TAG, "layout v%d: %d panel(s) in %dx%d, I am slot %d (row %d, col %d)",
                         st.layout_ver, st.count, st.cols, st.rows,
                         st.slot + 1, st.slot / st.cols + 1, st.slot % st.cols + 1);
            }

            if (now < marker_until) {
                draw_slot_number(st.slot + 1);
            } else {
                uint64_t frame = st.frame_no + (uint64_t)(now - st.frame_rx_us) * master_fps / 1000000;
                int idx = (int)((frame * anim.fps / master_fps) % anim.count);
                draw_gif(idx, st.slot, st.cols, st.rows);
            }
            if (now - st.frame_rx_us > 1000000) px(0, 0, BRIGHT, 0, 0);   // red corner = lost the Pico
        }

        int64_t send = esp_timer_get_time();
        flush();
        int64_t done = esp_timer_get_time();

        if (last_send) {
            int64_t gap = send - last_send;
            if (gap < gap_min) gap_min = gap;
            if (gap > gap_max) gap_max = gap;
            gap_sum += gap;
            frames++;
        }
        last_send = send;
        if (done - loop_start > work_max) work_max = done - loop_start;

        if (done - stats_start >= STATS_EVERY_US && frames > 0) {
            ESP_LOGI(TAG, "stats: core %d | frame gap ms min %.2f avg %.2f max %.2f | work max %.2f ms | "
                          "power peak %" PRIu32 " mA, capped %d/%d frames",
                     xPortGetCoreID(), gap_min / 1000.0, gap_sum / 1000.0 / frames, gap_max / 1000.0,
                     work_max / 1000.0, power_peak_ua / 1000, power_capped, frames);
            gap_min = INT64_MAX; gap_max = gap_sum = work_max = 0; frames = 0;
            power_peak_ua = 0; power_capped = 0;
            stats_start = done;
        }

        vTaskDelay(pdMS_TO_TICKS(REFRESH_MS));
    }
}

esp_err_t render_start(render_state_fn state_fn, int fps)
{
    if (!state_fn || fps <= 0) return ESP_ERR_INVALID_ARG;
    get_state = state_fn;
    master_fps = fps;

    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
#if LED_ORDER_GRB
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
#else
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
#endif
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "brightness: content %d/255, hard ceiling %d, power budget %d mA",
             BRIGHT, BRIGHTNESS_HARD_MAX, POWER_BUDGET_MA);

    frames_load(&anim);
    ESP_LOGI(TAG, "GIF: %d frames, %dx%d, %d fps (%s)", anim.count, anim.w, anim.h, anim.fps, anim.source);

    if (xTaskCreatePinnedToCore(render_task, "render", RENDER_STACK, NULL,
                                RENDER_PRIO, NULL, RENDER_CORE) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
