#pragma once

#include "tracker.h" /* HandLandmarks */
#include <stdint.h>

typedef enum
{
  GESTURE_NONE = 0,
  GESTURE_OPEN_PALM = 1,   /* all fingers extended — enables tracking */
  GESTURE_THUMBS_UP = 2,   /* thumb up only — takeoff */
  GESTURE_THUMBS_DOWN = 3, /* thumb down only — land */
} GestureID;

/*
 * Classify landmarks into a gesture with debounce.
 * Caller provides the time of detection (SDL_GetTicks() in ms) for debounce.
 * Returns the debounced gesture. Does NOT send any drone commands —
 * the caller is responsible for acting on command gestures.
 */
GestureID gesture_classify(const HandLandmarks *lm, uint32_t now_ms);

/* Read-only shape detection — no debounce, no drone_send(). Safe for HUD. */
GestureID gesture_identify(const HandLandmarks *lm);

/* Returns human-readable name for a gesture (for HUD). */
const char *gesture_name(GestureID g);
