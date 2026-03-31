#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdbool.h>

/* Returns true while the settings overlay is visible. */
bool config_menu_is_open(void);

/* Open / close / toggle the overlay. */
void config_menu_open(void);
void config_menu_close(void);
void config_menu_toggle(void);

/*
 * Feed an SDL event into the menu.
 * Returns true if the event was consumed (caller should not process it).
 */
bool config_menu_handle_event(const SDL_Event *ev);

/* Draw the menu overlay (call after the HUD, before SDL_RenderPresent). */
void config_menu_draw(SDL_Renderer *r, TTF_Font *font, TTF_Font *font_sm,
                      int ww, int wh);
