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
