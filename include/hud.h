#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdbool.h>

#include "tracker.h"  /* HandLandmarks */

/*
 * Search common system font directories for a usable monospace font.
 * Returns a path string (static storage) or NULL if none found.
 */
const char *hud_find_font(void);

/*
 * Render the HUD overlay onto the given renderer.
 * ww/wh are the current window dimensions.
 * gamepad indicates whether a gamepad is active (shown in status bar).
 * lm: latest hand landmarks (NULL or !lm->valid = no overlay drawn).
 * tracking: whether tracking mode is enabled (affects mode indicator).
 */
void hud_draw(SDL_Renderer *r, TTF_Font *font, TTF_Font *font_sm,
              int ww, int wh, bool gamepad,
              const HandLandmarks *lm, bool tracking);
