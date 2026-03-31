#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "config_menu.h"
#include "settings.h"

#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ================================================================
 *  Internal helpers (mirrors hud.c helpers - kept local)
 * ================================================================ */

static void cm_text(SDL_Renderer *r, TTF_Font *font,
                    int x, int y, SDL_Color col,
                    const char *fmt, ...)
{
  if (!font)
    return;
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (!buf[0])
    return;

  SDL_Surface *s = TTF_RenderText_Blended(font, buf, col);
  if (!s)
    return;
  SDL_Texture *t = SDL_CreateTextureFromSurface(r, s);
  if (t)
  {
    SDL_Rect dst = {x, y, s->w, s->h};
    SDL_RenderCopy(r, t, NULL, &dst);
    SDL_DestroyTexture(t);
  }
  SDL_FreeSurface(s);
}

static void cm_rect(SDL_Renderer *r,
                    int x, int y, int w, int h,
                    Uint8 cr, Uint8 cg, Uint8 cb, Uint8 ca)
{
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, cr, cg, cb, ca);
  SDL_Rect rc = {x, y, w, h};
  SDL_RenderFillRect(r, &rc);
}

static void cm_outline(SDL_Renderer *r,
                       int x, int y, int w, int h,
                       Uint8 cr, Uint8 cg, Uint8 cb, Uint8 ca)
{
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, cr, cg, cb, ca);
  SDL_Rect rc = {x, y, w, h};
  SDL_RenderDrawRect(r, &rc);
}

/* ================================================================
 *  Menu item definitions
 * ================================================================ */

typedef enum
{
  MK_FLOAT,
  MK_INT,
  MK_SEP
} MenuItemKind;

typedef struct
{
  MenuItemKind kind;
  const char *label;
  const char *unit; /* e.g. "ms", "" */
  void *ptr;        /* &g_settings.field */
  float min_val;
  float max_val;
  float step;
} MenuItem;

/* clang-format off */
static MenuItem items[] = {
    {MK_SEP,   "-- Control --",           "",    NULL,                            0,       0,      0    },
    {MK_INT,   "RC rate",                 "ms",  &g_settings.rc_rate_ms,          20,      500,    5    },
    {MK_FLOAT, "Deadzone",                "",    &g_settings.deadzone,            0.0f,    0.5f,   0.01f},
    {MK_FLOAT, "Expo curve",              "",    &g_settings.expo,                1.0f,    5.0f,   0.1f },

    {MK_SEP,   "-- Tracking gains --",    "",    NULL,                            0,       0,      0    },
    {MK_FLOAT, "Roll Kp",                 "",    &g_settings.gain_roll,           0.0f,    400.0f, 5.0f },
    {MK_FLOAT, "Roll Kd",                 "",    &g_settings.deriv_roll,          0.0f,    150.0f, 2.0f },
    {MK_FLOAT, "Throttle Kp",             "",    &g_settings.gain_throttle,       0.0f,    400.0f, 5.0f },
    {MK_FLOAT, "Throttle Kd",             "",    &g_settings.deriv_throttle,      0.0f,    150.0f, 2.0f },
    {MK_FLOAT, "Pitch Kp",                "",    &g_settings.gain_pitch,          0.0f,    200.0f, 5.0f },
    {MK_FLOAT, "Pitch Kd",                "",    &g_settings.deriv_pitch,         0.0f,    100.0f, 2.0f },
    {MK_FLOAT, "Target palm size",        "",    &g_settings.target_palm_size,    0.02f,   0.40f,  0.01f},
    {MK_INT,   "Lost-frame threshold",    "fr",  &g_settings.lost_frames,         5,       150,    5    },

    {MK_SEP,   "-- Gesture --",           "",    NULL,                            0,       0,      0    },
    {MK_INT,   "Debounce",                "ms",  &g_settings.gesture_debounce_ms, 50,      3000,   50   },
};
/* clang-format on */

#define N_ITEMS ((int)(sizeof(items) / sizeof(items[0])))

/* ================================================================
 *  State
 * ================================================================ */

static struct
{
  bool open;
  int sel; /* index of currently selected (non-separator) item */
} ms = {false, 1};

/* ================================================================
 *  Navigation helpers
 * ================================================================ */

static bool is_selectable(int i)
{
  return i >= 0 && i < N_ITEMS && items[i].kind != MK_SEP;
}

static int next_sel(int from, int dir)
{
  int i = from + dir;
  while (i >= 0 && i < N_ITEMS && !is_selectable(i))
    i += dir;
  if (i < 0 || i >= N_ITEMS)
    return from;
  return i;
}

static void adjust(int idx, float delta)
{
  MenuItem *it = &items[idx];
  if (it->kind == MK_FLOAT)
  {
    float *p = (float *)it->ptr;
    float v = *p + delta * it->step;
    if (v < it->min_val)
      v = it->min_val;
    if (v > it->max_val)
      v = it->max_val;
    /* Round to step to avoid floating-point drift */
    if (it->step >= 0.001f)
      v = roundf(v / it->step) * it->step;
    *p = v;
  }
  else if (it->kind == MK_INT)
  {
    int *p = (int *)it->ptr;
    int v = *p + (int)(delta * it->step);
    int mn = (int)it->min_val;
    int mx = (int)it->max_val;
    if (v < mn)
      v = mn;
    if (v > mx)
      v = mx;
    *p = v;
  }
}

/* ================================================================
 *  Public API
 * ================================================================ */

bool config_menu_is_open(void) { return ms.open; }

void config_menu_open(void)
{
  ms.open = true;
  /* Make sure selection starts on a valid item */
  if (!is_selectable(ms.sel))
    ms.sel = next_sel(0, 1);
}

void config_menu_close(void)
{
  ms.open = false;
  settings_save();
}

void config_menu_toggle(void)
{
  if (ms.open)
    config_menu_close();
  else
    config_menu_open();
}

bool config_menu_handle_event(const SDL_Event *ev)
{
  if (!ms.open)
    return false;

  if (ev->type == SDL_KEYDOWN)
  {
    switch (ev->key.keysym.sym)
    {
    case SDLK_ESCAPE:
    case SDLK_F1:
      config_menu_close();
      return true;

    case SDLK_UP:
      ms.sel = next_sel(ms.sel, -1);
      return true;

    case SDLK_DOWN:
      ms.sel = next_sel(ms.sel, +1);
      return true;

    case SDLK_LEFT:
    case SDLK_MINUS:
      adjust(ms.sel, -1.0f);
      return true;

    case SDLK_RIGHT:
    case SDLK_EQUALS: /* = key, same as + on most boards */
      adjust(ms.sel, +1.0f);
      return true;

    case SDLK_r:
      settings_reset_defaults();
      return true;

    case SDLK_RETURN:
    case SDLK_KP_ENTER:
      config_menu_close();
      return true;

    default:
      return true; /* consume all keys when menu is open */
    }
  }

  if (ev->type == SDL_CONTROLLERBUTTONDOWN ||
      ev->type == SDL_CONTROLLERAXISMOTION)
  {
    if (ev->type == SDL_CONTROLLERBUTTONDOWN)
    {
      switch (ev->cbutton.button)
      {
      case SDL_CONTROLLER_BUTTON_DPAD_UP:
        ms.sel = next_sel(ms.sel, -1);
        break;
      case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
        ms.sel = next_sel(ms.sel, +1);
        break;
      case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
        adjust(ms.sel, -1.0f);
        break;
      case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
        adjust(ms.sel, +1.0f);
        break;
      case SDL_CONTROLLER_BUTTON_B:
      case SDL_CONTROLLER_BUTTON_START:
        config_menu_close();
        break;
      default:
        break;
      }
    }
    return true;
  }

  return false;
}

/* ================================================================
 *  Rendering
 * ================================================================ */

void config_menu_draw(SDL_Renderer *r, TTF_Font *font, TTF_Font *font_sm,
                      int ww, int wh)
{
  if (!ms.open)
    return;

  TTF_Font *small = font_sm ? font_sm : font;

  /* --- Measure font height for layout --- */
  int fh = 20; /* fallback */
  if (font)
    TTF_SizeText(font, "M", NULL, &fh);
  int fh_sm = fh;
  if (small)
    TTF_SizeText(small, "M", NULL, &fh_sm);

  const int pad = 16;
  const int row_h = fh + 8;
  const int row_sm = fh_sm + 8;
  const int val_w = 80;
  const int unit_w = 36;
  const int panel_w = 440;

  /* Compute exact content height matching the rendering pass below */
  int n_sep = 0, n_reg = 0;
  for (int i = 0; i < N_ITEMS; i++)
    (items[i].kind == MK_SEP) ? n_sep++ : n_reg++;

  int content_h = pad                    /* top padding */
                  + (row_h + 4) + 6      /* title + separator line */
                  + n_sep * (4 + row_sm) /* separator section headers */
                  + n_reg * row_h        /* regular value rows */
                  + 4 + 4 + row_sm       /* footer line + hint text */
                  + pad;                 /* bottom padding */
  if (content_h > wh - 40)
    content_h = wh - 40;

  int px = (ww - panel_w) / 2;
  int py = (wh - content_h) / 2;

  /* --- Panel background --- */
  cm_rect(r, px - 2, py - 2, panel_w + 4, content_h + 4,
          60, 60, 90, 240);
  cm_rect(r, px, py, panel_w, content_h,
          20, 20, 40, 230);
  cm_outline(r, px, py, panel_w, content_h,
             100, 140, 255, 200);

  /* Clip all further rendering to the panel so nothing overflows */
  SDL_Rect clip = {px, py, panel_w, content_h};
  SDL_RenderSetClipRect(r, &clip);

  SDL_Color white = {255, 255, 255, 255};
  SDL_Color gray = {160, 160, 180, 255};
  SDL_Color cyan = {0, 200, 255, 255};
  SDL_Color yellow = {255, 220, 0, 255};
  SDL_Color sel_bg = {40, 80, 160, 200};
  SDL_Color sep_c = {120, 180, 255, 255};
  SDL_Color dim = {80, 80, 100, 255};

  int cx = px + pad;
  int cy = py + pad;
  int lbl_w = panel_w - pad * 2 - val_w - unit_w - 8;

  /* --- Title row --- */
  cm_rect(r, px, cy - 4, panel_w, row_h + 4, 30, 50, 120, 200);
  cm_text(r, font, cx, cy, cyan, "SETTINGS");
  cm_text(r, font, px + panel_w - pad - 80, cy, gray, "[F1] close");
  cy += row_h + 4;

  /* Thin separator line */
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, 100, 140, 255, 180);
  SDL_RenderDrawLine(r, px + 4, cy, px + panel_w - 4, cy);
  cy += 6;

  /* --- Items --- */
  for (int i = 0; i < N_ITEMS; i++)
  {
    MenuItem *it = &items[i];
    bool sel = (i == ms.sel);

    if (it->kind == MK_SEP)
    {
      cy += 4;
      cm_text(r, small, cx, cy, sep_c, "%s", it->label);
      cy += row_sm;
      continue;
    }

    /* Highlight selected row */
    if (sel)
      cm_rect(r, px + 4, cy - 2, panel_w - 8, row_h,
              sel_bg.r, sel_bg.g, sel_bg.b, sel_bg.a);

    SDL_Color lbl_col = sel ? white : gray;

    /* Label */
    cm_text(r, font, cx, cy, lbl_col, "%s", it->label);

    /* Value */
    char vbuf[32];
    if (it->kind == MK_FLOAT)
    {
      float v = *(float *)it->ptr;
      if (it->step < 0.02f)
        snprintf(vbuf, sizeof(vbuf), "%.3f", v);
      else if (it->step < 0.2f)
        snprintf(vbuf, sizeof(vbuf), "%.2f", v);
      else
        snprintf(vbuf, sizeof(vbuf), "%.1f", v);
    }
    else
    {
      snprintf(vbuf, sizeof(vbuf), "%d", *(int *)it->ptr);
    }

    int vx = px + pad + lbl_w;

    /* Arrows for selected item */
    if (sel)
    {
      cm_text(r, font, vx - fh - 2, cy, yellow, "<");
      cm_text(r, font, vx + val_w + 2, cy, yellow, ">");
    }

    /* Value box */
    cm_rect(r, vx, cy - 1, val_w, fh + 2, 10, 10, 30, 200);
    cm_outline(r, vx, cy - 1, val_w, fh + 2,
               sel ? 100 : 50, sel ? 140 : 70, sel ? 255 : 100, 200);
    {
      int tw = 0;
      if (font)
        TTF_SizeText(font, vbuf, &tw, NULL);
      cm_text(r, font, vx + (val_w - tw) / 2, cy,
              sel ? yellow : white, "%s", vbuf);
    }

    /* Unit label */
    if (it->unit && it->unit[0])
      cm_text(r, font, vx + val_w + (sel ? fh + 6 : 4), cy, dim, "%s", it->unit);

    cy += row_h;
  }

  /* --- Footer --- */
  cy += 4;
  SDL_SetRenderDrawColor(r, 100, 140, 255, 120);
  SDL_RenderDrawLine(r, px + 4, cy, px + panel_w - 4, cy);
  cy += 4;
  cm_text(r, small, cx, cy, gray,
          "Up/Down: navigate  Left/Right: adjust  R: reset");

  /* Restore clip */
  SDL_RenderSetClipRect(r, NULL);
}
