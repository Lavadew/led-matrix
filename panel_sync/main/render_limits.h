// Brightness + power limiting for one panel's framebuffer.
// Pure C, no ESP-IDF, so it can be unit-tested on a PC.
//
// Two independent safety layers, applied to every frame right before it goes to the LEDs:
//   1. Power cap: estimate the LED current for this frame; if it's over budget,
//      scale the whole frame down evenly (colours keep their ratios).
//   2. Hard ceiling: clamp every channel to BRIGHTNESS_HARD_MAX, no matter what.
#pragma once
#include <stdint.h>

#define BRIGHTNESS_HARD_MAX 8     // NO channel value above this ever reaches the LEDs
#define MA_PER_CHANNEL      20    // WS2812: ~20mA per colour channel at full (255) drive

// Estimated LED drive current in microamps for n RGB pixels (idle current not included).
static inline uint32_t frame_current_ua(const uint8_t *rgb, int n)
{
    uint32_t sum = 0;
    for (int i = 0; i < n * 3; i++) sum += rgb[i];
    return sum * (MA_PER_CHANNEL * 1000u) / 255u;
}

// Applies both layers in place. Returns the estimated current BEFORE limiting (uA),
// and sets *capped to 1 if the power cap had to scale the frame down.
static inline uint32_t limit_frame(uint8_t *rgb, int n, uint32_t budget_ua, int *capped)
{
    for (int i = 0; i < n * 3; i++)
        if (rgb[i] > BRIGHTNESS_HARD_MAX) rgb[i] = BRIGHTNESS_HARD_MAX;

    uint32_t est = frame_current_ua(rgb, n);
    *capped = 0;
    if (est > budget_ua) {
        // Round DOWN so the scaled frame can only land at or under the budget.
        for (int i = 0; i < n * 3; i++) rgb[i] = (uint8_t)((uint32_t)rgb[i] * budget_ua / est);
        *capped = 1;
    }
    return est;
}
