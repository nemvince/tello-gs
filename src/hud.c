#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "hud.h"
#include "telemetry.h"
#include "drone.h"
#include "gesture.h"
#include "tracker.h"
#include "config.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>

/* ---- Font search ---- */

static const char *font_paths[] = {
    "/usr/share/fonts/TTF/0xProtoNerdFontMono-Regular.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    "/usr/share/fonts/liberation-mono/LiberationMono-Regular.ttf",
    "/usr/share/fonts/noto/NotoSansMono-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
    "/usr/share/fonts/google-noto/NotoSansMono-Regular.ttf",
    "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
    NULL};

const char *hud_find_font(void)
{
  for (int i = 0; font_paths[i]; i++)
  {
    if (access(font_paths[i], R_OK) == 0)
      return font_paths[i];
  }
  return NULL;
}

/* ---- Internal helpers ---- */

static void draw_text(SDL_Renderer *r, TTF_Font *font,
                      int x, int y, SDL_Color col, const char *fmt, ...)
{
  if (!font)
    return;
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (buf[0] == '\0')
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

static void fill_rect(SDL_Renderer *r, int x, int y, int w, int h,
                      Uint8 cr, Uint8 cg, Uint8 cb, Uint8 ca)
{
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, cr, cg, cb, ca);
  SDL_Rect rc = {x, y, w, h};
  SDL_RenderFillRect(r, &rc);
}

/* ---- HUD drawing ---- */

/* MediaPipe hand skeleton connection pairs (indices into HandLandmarks.pts) */
static const int HAND_CONNECTIONS[][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 4}, {0, 5}, {5, 6}, {6, 7}, {7, 8}, {0, 9}, {9, 10}, {10, 11}, {11, 12}, {0, 13}, {13, 14}, {14, 15}, {15, 16}, {0, 17}, {17, 18}, {18, 19}, {19, 20}, {5, 9}, {9, 13}, {13, 17}};
#define N_CONNECTIONS ((int)(sizeof(HAND_CONNECTIONS) / sizeof(HAND_CONNECTIONS[0])))

void hud_draw(SDL_Renderer *r, TTF_Font *font, TTF_Font *font_sm,
              int ww, int wh, bool gamepad,
              const HandLandmarks *lm, bool tracking)
{
  Telemetry t;
  telemetry_get(&t);

  float sx = (float)ww / WIN_W;
  float sy = (float)wh / WIN_H;
#define SX(v) ((int)((v) * sx))
#define SY(v) ((int)((v) * sy))

  SDL_Color white = {255, 255, 255, 255};
  SDL_Color green = {0, 220, 80, 255};
  SDL_Color red = {255, 50, 50, 255};
  SDL_Color cyan = {0, 220, 255, 255};
  SDL_Color yellow = {255, 220, 0, 255};
  SDL_Color orange = {255, 160, 0, 255};

  /* --- Top bar --- */
  fill_rect(r, 0, 0, ww, SY(32), 0, 0, 0, 160);
  SDL_Color bat_col = t.bat > 50 ? green : (t.bat > 20 ? yellow : red);
  draw_text(r, font, SX(10), SY(6), bat_col, "BAT %d%%", t.bat);
  draw_text(r, font, ww / 2 - SX(50), SY(6), white, "TIME %02d:%02d",
            t.flight_time / 60, t.flight_time % 60);
  draw_text(r, font, ww - SX(160), SY(6), white, "TEMP %d-%dC",
            t.templ, t.temph);

  /* --- Speed panel (left) --- */
  fill_rect(r, SX(6), SY(50), SX(140), SY(55), 0, 0, 0, 160);
  float spd = sqrtf((float)(t.vgx * t.vgx + t.vgy * t.vgy + t.vgz * t.vgz));
  draw_text(r, font, SX(12), SY(52), green, "SPD %.1f", spd);
  draw_text(r, font_sm, SX(12), SY(76), white,
            "X%+d Y%+d Z%+d", t.vgx, t.vgy, t.vgz);

  /* --- Altitude bar (right) --- */
  int bar_x = ww - SX(44), bar_y = SY(50), bar_h = SY(220);
  fill_rect(r, bar_x - SX(6), bar_y - SY(6), SX(44), bar_h + SY(32), 0, 0, 0, 160);

  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, 255, 255, 255, 200);
  SDL_Rect alt_out = {bar_x, bar_y, SX(24), bar_h};
  SDL_RenderDrawRect(r, &alt_out);

  int fill = (int)(bar_h * fminf(1.0f, fmaxf(0.0f, t.height / 300.0f)));
  if (fill > 0)
    fill_rect(r, bar_x + 1, bar_y + bar_h - fill, SX(24) - 2, fill,
              0, 220, 255, 200);
  draw_text(r, font_sm, bar_x - SX(4), bar_y + bar_h + SY(4), cyan,
            "%dcm", t.height);

  /* --- Attitude indicator (center) --- */
  int cx = ww / 2, cy = wh / 2, rad = SX(55);
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, 255, 255, 255, 100);
  for (int i = 0; i < 64; i++)
  {
    float a1 = i / 64.0f * 2.0f * (float)M_PI;
    float a2 = (i + 1) / 64.0f * 2.0f * (float)M_PI;
    SDL_RenderDrawLine(r,
                       cx + (int)(cosf(a1) * rad), cy + (int)(sinf(a1) * rad),
                       cx + (int)(cosf(a2) * rad), cy + (int)(sinf(a2) * rad));
  }
  float roll_r = t.roll * (float)M_PI / 180.0f;
  int pitch_off = (int)(t.pitch * rad / 45.0f);
  int dx = (int)(cosf(-roll_r) * rad * 0.8f);
  int dy = (int)(sinf(-roll_r) * rad * 0.8f);
  SDL_SetRenderDrawColor(r, 0, 220, 80, 220);
  SDL_RenderDrawLine(r, cx - dx, cy + pitch_off - dy,
                     cx + dx, cy + pitch_off + dy);
  /* crosshair */
  SDL_SetRenderDrawColor(r, 255, 220, 0, 220);
  SDL_RenderDrawLine(r, cx - SX(8), cy, cx + SX(8), cy);
  SDL_RenderDrawLine(r, cx, cy - SY(8), cx, cy + SY(8));

  /* --- Compass --- */
  static const char *dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int yaw_pos = ((t.yaw % 360) + 360) % 360;
  int di = ((yaw_pos + 22) / 45) % 8;
  fill_rect(r, ww / 2 - SX(60), wh - SY(68), SX(120), SY(24), 0, 0, 0, 160);
  draw_text(r, font, ww / 2 - SX(48), wh - SY(66), orange,
            "%03d %s", yaw_pos, dirs[di]);

  /* --- Bottom bar --- */
  fill_rect(r, 0, wh - SY(28), ww, SY(28), 0, 0, 0, 160);
  draw_text(r, font_sm, SX(10), wh - SY(24),
            drone_connected ? green : red,
            drone_connected ? "DRONE CONNECTED" : "DISCONNECTED");
  draw_text(r, font_sm, ww / 2 - SX(35), wh - SY(24),
            gamepad ? green : yellow, gamepad ? "GAMEPAD" : "KEYBOARD");
  draw_text(r, font_sm, ww - SX(110), wh - SY(24), white,
            "TOF %dcm", t.tof);

  /* --- Tracking mode indicator + debug --- */
  TrackerDebug dbg = tracker_get_debug();
  {
    SDL_Color trk_col = tracking ? green : yellow;
    draw_text(r, font_sm, SX(10), SY(110), trk_col,
              tracking ? "TRACKING ON" : "TRACKING OFF");
    draw_text(r, font_sm, SX(10), SY(126), white,
              "PALM %.2f(raw %.2f) PRES %.2f LOST %d",
              dbg.palm_score, dbg.palm_best_raw, dbg.hand_presence, dbg.lost_frames);
    const char *gname = gesture_name((GestureID)dbg.gesture);
    draw_text(r, font_sm, SX(10), SY(142), cyan,
              "GESTURE: %s", gname[0] ? gname : "none");
    float fps = dbg.inference_ms > 0.0f ? 1000.0f / dbg.inference_ms : 0.0f;
    draw_text(r, font_sm, SX(10), SY(158), white,
              "INFER %.1fms (%.0f FPS)", dbg.inference_ms, fps);
  }

  /* Palm bounding box — always drawn when a palm is detected (green = landmarks ok, orange = pending) */
  if (dbg.palm_score > 0.0f)
  {
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    int bx = (int)((dbg.box_cx - dbg.box_w * 0.5f) * ww);
    int by = (int)((dbg.box_cy - dbg.box_h * 0.5f) * wh);
    int bw = (int)(dbg.box_w * ww);
    int bh = (int)(dbg.box_h * wh);
    SDL_Color box_col = (lm && lm->valid) ? green : orange;
    SDL_SetRenderDrawColor(r, box_col.r, box_col.g, box_col.b, 200);
    SDL_Rect box_rect = {bx, by, bw, bh};
    SDL_RenderDrawRect(r, &box_rect);
    /* draw a second pixel-offset rect for thickness */
    SDL_Rect box_rect2 = {bx + 1, by + 1, bw - 2, bh - 2};
    SDL_RenderDrawRect(r, &box_rect2);
  }

  /* --- Hand skeleton overlay --- */
  if (lm && lm->valid)
  {
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);

    /* Connections */
    SDL_SetRenderDrawColor(r, 0, 200, 255, 200);
    for (int k = 0; k < N_CONNECTIONS; k++)
    {
      const Landmark *a = &lm->pts[HAND_CONNECTIONS[k][0]];
      const Landmark *b = &lm->pts[HAND_CONNECTIONS[k][1]];
      SDL_RenderDrawLine(r,
                         (int)(a->x * ww), (int)(a->y * wh),
                         (int)(b->x * ww), (int)(b->y * wh));
    }

    /* Landmark dots */
    for (int k = 0; k < 21; k++)
    {
      int lx = (int)(lm->pts[k].x * ww);
      int ly = (int)(lm->pts[k].y * wh);
      SDL_SetRenderDrawColor(r, 255, 80, 80, 230);
      SDL_Rect dot = {lx - 3, ly - 3, 7, 7};
      SDL_RenderFillRect(r, &dot);
    }

    /* Gesture label above wrist */
    {
      GestureID g = gesture_identify(lm);
      const char *gname = gesture_name(g);
      int wx = (int)(lm->pts[0].x * ww);
      int wy = (int)(lm->pts[0].y * wh) - SY(22);
      draw_text(r, font_sm, wx - SX(30), wy, cyan, "%s",
                gname[0] ? gname : "HAND");
    }
  }

#undef SX
#undef SY
#undef N_CONNECTIONS
}
