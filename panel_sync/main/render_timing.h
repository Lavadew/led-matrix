// Frame-boundary scheduling. Pure C, no ESP-IDF, so it can be unit-tested on a PC.
//
// The Pico's beacon says "frame frame_no started at rx_us (on my clock)". Every panel
// works out when the NEXT frame starts and sends its LEDs exactly then, so all panels
// flip on the same beat and every frame is on screen for the same time (no judder).
#pragma once
#include <stdint.h>

// Given the last beacon (frame_no seen at rx_us) and the current time, returns the next
// frame number F and the time t it starts. Always t > now, and t - now <= one frame.
// Computed from the beacon each time (not by adding periods) so it never drifts.
static inline void next_frame_boundary(uint32_t frame_no, int64_t rx_us, int64_t now,
                                       int fps, uint64_t *F, int64_t *t)
{
    int64_t since = now - rx_us;
    if (since < 0) since = 0;
    uint64_t k = (uint64_t)since * (uint64_t)fps / 1000000u + 1;   // frames after frame_no
    int64_t t_k = rx_us + (int64_t)(k * 1000000u / (uint64_t)fps);
    if (t_k <= now) {                     // rounding put us exactly on a boundary: take the next
        k++;
        t_k = rx_us + (int64_t)(k * 1000000u / (uint64_t)fps);
    }
    *F = (uint64_t)frame_no + k;
    *t = t_k;
}

/* ---------- smoothed frame clock ----------
 * Beacons say "frame N is current" but not how far into frame N the Pico is, and
 * WiFi adds a few ms of random delay. Rebuilding the schedule from every beacon
 * (next_frame_boundary above) makes frames jump by up to a whole frame.
 *
 * frame_clock_t keeps its own estimate of when frames start and treats each beacon
 * as a hint: it moves the estimate 1/8 of the way toward the beacon, never more than
 * 1 ms per beacon. Big disagreements (Pico restart, long dropout) snap straight to
 * the beacon instead. Result: steady frame gaps, same beat on every panel.
 */
#define FC_GAIN_DIV     8        // move 1/8 of the error per beacon
#define FC_MAX_STEP_US  1000     // but never more than 1 ms per beacon
#define FC_SNAP_FRAMES  3        // error bigger than this many frames -> snap

typedef struct {
    int      valid;              // have we seen a beacon yet
    uint64_t aF;                 // anchor: frame aF starts at local time at
    int64_t  at;
    int64_t  last_rx;            // rx time of the last beacon used
    int      have_last;          // last_F is meaningful
    uint64_t last_F;             // last frame number handed out
    int64_t  last_err;           // most recent beacon-vs-estimate error (us)
    int      snaps;              // times we snapped instead of smoothing
} frame_clock_t;

static inline int64_t fc_start(const frame_clock_t *c, uint64_t G, int fps)
{
    int64_t d = (int64_t)G - (int64_t)c->aF;
    return c->at + d * 1000000 / fps;
}

// Feed a beacon (frame_no seen at rx_us). Safe to call with the same beacon repeatedly.
static inline void frame_clock_update(frame_clock_t *c, uint32_t frame_no, int64_t rx_us, int fps)
{
    if (c->valid && rx_us == c->last_rx) return;          // already used this beacon
    c->last_rx = rx_us;
    if (!c->valid) {
        c->valid = 1; c->aF = frame_no; c->at = rx_us; c->have_last = 0; c->last_err = 0;
        return;
    }
    int64_t predicted = fc_start(c, frame_no, fps);
    int64_t e = rx_us - predicted;
    c->last_err = e;
    int64_t snap = (int64_t)FC_SNAP_FRAMES * 1000000 / fps;
    if (e > snap || e < -snap) {
        c->aF = frame_no; c->at = rx_us; c->have_last = 0; c->snaps++;
        return;
    }
    int64_t step = e / FC_GAIN_DIV;
    if (step >  FC_MAX_STEP_US) step =  FC_MAX_STEP_US;
    if (step < -FC_MAX_STEP_US) step = -FC_MAX_STEP_US;
    c->aF = frame_no;                                     // re-anchor near "now" to keep numbers small
    c->at = predicted + step;
}

// Next frame to show after `now` and when it starts. Never repeats a frame number.
static inline void frame_clock_next(frame_clock_t *c, int64_t now, int fps, uint64_t *F, int64_t *t)
{
    int64_t since = now - c->at;
    int64_t k = since >= 0 ? since * fps / 1000000 + 1 : 0;
    int64_t tk = c->at + k * 1000000 / fps;
    if (tk <= now) { k++; tk = c->at + k * 1000000 / fps; }
    uint64_t G = c->aF + (uint64_t)k;
    if (c->have_last && G <= c->last_F) {                 // estimate slid later: don't show a frame twice
        G = c->last_F + 1;
        tk = fc_start(c, G, fps);
    }
    c->last_F = G; c->have_last = 1;
    *F = G; *t = tk;
}
