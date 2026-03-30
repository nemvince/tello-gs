#pragma once

#include "tracker.h"   /* HandLandmarks */
#include <stdint.h>

typedef enum {
    GESTURE_NONE       = 0,
    GESTURE_OPEN_PALM  = 1,   /* all fingers extended — enables tracking */
    GESTURE_THUMBS_UP  = 2,   /* thumb up only — takeoff */
    GESTURE_THUMBS_DOWN= 3,   /* thumb down only — land */
} GestureID;

/*
 * Classify landmarks into a gesture.
 * Caller provides the time of detection (SDL_GetTicks() in ms) for debounce.
 * When a debounced command gesture (thumbs-up / thumbs-down) fires its
 * command, the function calls drone_send() internally.
 */
GestureID gesture_classify(const HandLandmarks *lm, uint32_t now_ms);

/* Returns human-readable name for a gesture (for HUD). */
const char *gesture_name(GestureID g);
