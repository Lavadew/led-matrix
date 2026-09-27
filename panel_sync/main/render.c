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
#include "render_timing.h"
#include "esp_rom_sys.h"

#define LED_GPIO          14
#define PANEL_W           8
#define PANEL_H           8
#define LED_COUNT         (PANEL_W * PANEL_H)
#define BRIGHT            8           // 0-255 brightness scale for content (dim())
#define POWER_BUDGET_MA   60          // max estimated LED current per panel; frames above it are scaled down
#define MARKER_US         2000000     // show slot number for 2s after a layout change

// Render task placement. WiFi and the network tasks run on core 0, so the renderer
// gets core 1 to itself and can't be delayed by bursts of packets.
// Set RENDER_CORE 0 to reproduce the old behaviour for before/after measurements.
#define RENDER_CORE       1
#define RENDER_PRIO       5
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

// Framebuffer: drawing code writes here in logical (x, y) order. prepare() applies the
// power cap + hard ceiling and maps to the physical LED order; send() pushes it out.
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

// The ONLY place pixels reach the LEDs, in two halves so the slow part (limiting +
// serialising) happens BEFORE the frame boundary and only the send happens on it.
static void prepare(void)
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
}

static void send(void)
{
    led_strip_refresh(strip);
}

// Sleep until time t (esp_timer microseconds): tick-sleep most of the way, then
// spin the last ~1ms for sub-millisecond accuracy. Only runs on core 1.
static void wait_until(int64_t t)
{
    for (;;) {
        int64_t d = t - esp_timer_get_time();
        if (d <= 0) return;
        TickType_t ticks = (TickType_t)((d - 1000) / (1000 * portTICK_PERIOD_MS));
        if (ticks >= 1) vTaskDelay(ticks);
        else { esp_rom_delay_us((uint32_t)d); return; }
    }
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
// Draws what the panel should show for frame F at time `at` into the framebuffer.
static void compose(const render_state_t *st, uint64_t F, int64_t at)
{
    static int     shown_slot = -2, shown_cols = 0, shown_rows = 0;
    static int64_t marker_until = 0;

    if (!st->synced) {
        // blinking red = no beacon from the Pico yet
        clear_all();
        if ((at / 500000) % 2) px(0, 0, BRIGHT, 0, 0);
    } else if (st->slot < 0) {
        // blinking blue = hearing the Pico, waiting to be given a slot
        clear_all();
        if ((at / 250000) % 2) px(0, 0, 0, 0, BRIGHT);
    } else {
        // Only show the number when THIS panel's slot or the grid shape changes,
        // not every time some other panel blips in and out.
        if (st->slot != shown_slot || st->cols != shown_cols || st->rows != shown_rows) {
            shown_slot = st->slot; shown_cols = st->cols; shown_rows = st->rows;
            marker_until = at + MARKER_US;
            ESP_LOGI(TAG, "layout v%d: %d panel(s) in %dx%d, I am slot %d (row %d, col %d)",
                     st->layout_ver, st->count, st->cols, st->rows,
                     st->slot + 1, st->slot / st->cols + 1, st->slot % st->cols + 1);
        }

        if (at < marker_until) {
            draw_slot_number(st->slot + 1);
        } else {
            int idx = (int)((F * anim.fps / master_fps) % anim.count);
            draw_gif(idx, st->slot, st->cols, st->rows);
        }
        if (at - st->frame_rx_us > 1000000) px(0, 0, BRIGHT, 0, 0);   // red corner = lost the Pico
    }
}

static void render_task(void *arg)
{
    const int64_t period_us = 1000000 / master_fps;
    int64_t local_next = 0;                       // used only before the first beacon

    // Stats. gap = time between frames sent; late = how far past the frame boundary
    // the send actually started (the renderer's contribution to sync error).
    int64_t last_send = 0, stats_start = esp_timer_get_time();
    int64_t gap_min = INT64_MAX, gap_max = 0, gap_sum = 0, work_max = 0;
    int64_t late_min = INT64_MAX, late_max = 0, late_sum = 0;
    int     frames = 0;

    while (1) {
        render_state_t st;
        get_state(&st);
        int64_t now = esp_timer_get_time();

        // When is the next frame due?
        uint64_t F; int64_t target;
        if (st.synced) {
            next_frame_boundary(st.frame_no, st.frame_rx_us, now, master_fps, &F, &target);
        } else {
            F = 0;
            target = (local_next > now) ? local_next : now + period_us;
        }
        local_next = target + period_us;

        // Draw + limit + serialise ahead of time, then send exactly on the boundary.
        compose(&st, F, target);
        prepare();
        int64_t ready = esp_timer_get_time();
        wait_until(target);
        int64_t sent = esp_timer_get_time();
        send();

        if (ready - now > work_max) work_max = ready - now;
        if (last_send) {
            int64_t late = sent - target;
            if (late < late_min) late_min = late;
            if (late > late_max) late_max = late;
            late_sum += late;
            int64_t gap = sent - last_send;
            if (gap < gap_min) gap_min = gap;
            if (gap > gap_max) gap_max = gap;
            gap_sum += gap;
            frames++;
        }
        last_send = sent;

        if (sent - stats_start >= STATS_EVERY_US && frames > 0) {
            ESP_LOGI(TAG, "stats: core %d | %.1f fps | frame gap ms min %.2f avg %.2f max %.2f | "
                          "late us min %lld avg %lld max %lld | work max %.2f ms | "
                          "power peak %" PRIu32 " mA, capped %d/%d",
                     xPortGetCoreID(), frames * 1e6 / (double)(sent - stats_start),
                     gap_min / 1000.0, gap_sum / 1000.0 / frames, gap_max / 1000.0,
                     (long long)late_min, (long long)(late_sum / frames), (long long)late_max,
                     work_max / 1000.0, power_peak_ua / 1000, power_capped, frames);
            gap_min = late_min = INT64_MAX; gap_max = gap_sum = late_max = late_sum = work_max = 0;
            frames = 0; power_peak_ua = 0; power_capped = 0;
            stats_start = sent;
        }
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
    ESP_LOGI(TAG, "timing: %d fps frame clock, sending on frame boundaries, core %d, tick %d Hz",
             master_fps, RENDER_CORE, configTICK_RATE_HZ);
    if (configTICK_RATE_HZ < 1000)
        ESP_LOGW(TAG, "tick is %d Hz: frame timing will be coarse. Delete sdkconfig and rebuild "
                      "so sdkconfig.defaults (1000 Hz) applies", configTICK_RATE_HZ);

    frames_load(&anim);
    int loop_ms = anim.count * 1000 / anim.fps;
    ESP_LOGI(TAG, "GIF: %d frames, %dx%d, %d fps, %d.%02d s loop, %d KB (%s)",
             anim.count, anim.w, anim.h, anim.fps, loop_ms / 1000, (loop_ms % 1000) / 10,
             anim.count * anim.w * anim.h * 3 / 1024, anim.source);

    if (xTaskCreatePinnedToCore(render_task, "render", RENDER_STACK, NULL,
                                RENDER_PRIO, NULL, RENDER_CORE) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
